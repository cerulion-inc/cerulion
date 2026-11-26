// SPDX-License-Identifier: AGPL-3.0-only
//! Raw delivery and native state/refusal parity through both SDK backends.
//! Prerequisites: build test_node_macro_cdylib and test_node_state_panic_cdylib.
//! Run with --test-threads=1; the SDK parity cases also load shared fixture libraries.
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

mod unwired_native {
    use cerulion_core::prelude::*;
    use native_ros2_messages::geometry_msgs::Vector3;
    #[cerulion_node(period_ms = 1)]
    pub struct UnwiredProbe {
        #[output]
        pub cmd_out: Vector3,
    }
    #[cerulion_node_impl]
    impl UnwiredProbe {
        fn tick(&mut self) -> Result<(), NodeError> {
            self.cmd_out.x = 1.0;
            Ok(())
        }
    }
}

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
        .with_raw_publisher_budget(4)
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
    for _ in 0..2 {
        for (factory, topic) in [
            (&native_factory, "abi/raw/sdk/native"),
            (&plugin_factory, "abi/raw/sdk/plugin"),
        ] {
            let run_scope = manager.exclude_remote_topic(topic).unwrap();
            let mut publisher = factory
                .create_publisher(
                    topic,
                    Vector3::SCHEMA_HASH,
                    MaxSliceLen::try_new(64).unwrap(),
                    FullQueueSend::Discard,
                )
                .unwrap();
            let mut receiver = manager.create_subscriber(topic).unwrap();
            drop(run_scope);
            assert!(
                manager.acquire_remote_topic_lease(topic).is_err(),
                "raw publisher must retain private scope after the run ends"
            );
            publisher.register_egress().unwrap();
            assert_eq!(manager.dynamic_egress_record_hash_for_test(topic), None);
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
            drop(receiver);
            drop(publisher);
            assert!(
                manager
                    .remote_topic_allowed(topic, cerulion_core::transport::TeardownWait::Bounded)
                    .unwrap(),
                "destroyed raw port must release its private scope while the host remains live"
            );
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
    for debug_filter in [true, false] {
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
            .with_env_filter(tracing_subscriber::EnvFilter::new(if debug_filter {
                "cerulion_core::transport::output_proxy=debug"
            } else {
                "info"
            }))
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
            .with_env_filter(tracing_subscriber::EnvFilter::new(if debug_filter {
                "cerulion_core::transport::output_proxy=debug"
            } else {
                "info"
            }))
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
        let native_text =
            std::string::String::from_utf8(native_log.lock().unwrap().clone()).unwrap();
        let stable_text =
            std::string::String::from_utf8(stable_log.lock().unwrap().clone()).unwrap();
        for text in [&native_text, &stable_text] {
            assert_eq!(text.matches("OutputProxy dropped without writing all declared variable fields; skipping publish").count(), 1);
            assert_eq!(
                text.matches("OutputProxy discard suppressed: incomplete output")
                    .count(),
                cerulion_core::testing::debug_lines_expected(usize::from(debug_filter))
            );
            for level in ["WARN", "INFO", "ERROR"] {
                assert_eq!(
                    text.lines()
                        .filter(|line| {
                            cerulion_core::testing::line_level(line) == Some(level)
                                && line
                                    .contains("OutputProxy discard suppressed: incomplete output")
                        })
                        .count(),
                    0
                );
            }
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
}

#[test]
fn unwired_rust_outputs_preserve_native_init_and_named_tick_refusal() {
    use cerulion_core::graph::node::NodeEntry;
    use cerulion_core::graph::node_host_abi::HostPortSpec;
    use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};
    let clock = Arc::new(VirtualClock::new());
    clock.set(777);
    let env = Arc::new(HashMap::new());
    let native = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        clock.clone(),
        ShutdownSignal::new(),
        env.clone(),
    );
    assert!(native.publisher("out").is_none());
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
        value: register_context(HostContext::new(native, specs, env.clone(), 4).unwrap()).unwrap(),
    };
    with_context(context.value, |host| host.begin_node_call()).unwrap();
    // SAFETY: actual checked immutable host table and registered context are
    // retained until the node-local context and its clients have been dropped.
    let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
    let info = DylibNodeEntry::parse_info_json_labeled(&format!(r#"{{"inputs":[],"outputs":[{{"name":"out","schema_hash":{}}}],"policy":{{"period_ms":1}}}}"#, Vector3::SCHEMA_HASH), "unwired Rust").unwrap();
    let plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();
    assert!(plugin.publisher("out").is_none());
    assert!(!client.has_failure());
    assert_eq!(
        with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
        0
    );
    drop(plugin);
    drop(client);
    with_context(context.value, |host| host.end_node_call()).unwrap();
    remove_context(context.value).unwrap();
    for _ in 0..2 {
        let empty = || {
            NodeContext::with_runtime_env(
                IndexMap::new(),
                IndexMap::new(),
                clock.clone(),
                ShutdownSignal::new(),
                env.clone(),
            )
        };
        let mut native = unwired_native::UnwiredProbeEntry::new();
        native.init(empty()).unwrap();
        let path = cerulion_core::testing::find_fixture_cdylib("test_node_macro_cdylib");
        let mut stable = DylibNodeEntry::load(&path).unwrap();
        stable.init(empty()).unwrap();
        for error in [native.tick().unwrap_err(), stable.tick().unwrap_err()] {
            let cerulion_core::error::TransportError::NodeError { reason, .. } = error else {
                panic!("unwired tick must keep the native untyped node error class: {error}");
            };
            assert_eq!(
                reason,
                "zero-copy tick: missing publisher for output `cmd_out`"
            );
        }
        native.shutdown().unwrap();
        stable.shutdown().unwrap();
    }
}

#[test]
fn host_sink_panic_keeps_native_minus_six_detail_without_latching() {
    use cerulion_core::graph::node::NodeEntry;
    use cerulion_core::state::{SinkFull, StateSink};
    struct PanicSink {
        writes: usize,
        refused: usize,
    }
    impl StateSink for PanicSink {
        fn write(&mut self, _: &[u8]) -> Result<(), SinkFull> {
            self.writes += 1;
            panic!("host sink panic oracle");
        }
        fn refuse(&mut self) {
            self.refused += 1;
        }
    }
    extern "C" fn panicking_native_sink(_: *mut std::ffi::c_void, _: *const u8, _: usize) -> i32 {
        match std::panic::catch_unwind(|| panic!("native host sink panic oracle")) {
            Ok(()) => 0,
            Err(_) => -1,
        }
    }
    let path = cerulion_core::testing::find_fixture_cdylib("test_node_state_panic_cdylib");
    let detail =
        "cerulion_node_capture_state: state sink is full: the capture exceeds the boundary budget";
    for _ in 0..2 {
        let empty = || {
            NodeContext::with_runtime_env(
                IndexMap::new(),
                IndexMap::new(),
                Arc::new(VirtualClock::new()),
                ShutdownSignal::new(),
                Arc::new(HashMap::new()),
            )
        };
        // SAFETY (U01/U03/U12): trusted same-compiler native control fixture,
        // exact generated signatures, transferred boxed context, library kept
        // live through shutdown and diagnostic copy/free; callback retains no bytes.
        unsafe {
            let library = libloading::Library::new(&path).unwrap();
            let init = library
                .get::<extern "C" fn(*mut NodeContext) -> u64>(b"cerulion_node_init\0")
                .unwrap();
            type SinkFn = extern "C" fn(*mut std::ffi::c_void, *const u8, usize) -> i32;
            let capture = library
                .get::<extern "C" fn(u64, Option<SinkFn>, *mut std::ffi::c_void, u64) -> i32>(
                    b"cerulion_node_capture_state\0",
                )
                .unwrap();
            let take = library
                .get::<extern "C" fn() -> *mut std::ffi::c_char>(b"cerulion_take_last_error\0")
                .unwrap();
            let free = library
                .get::<unsafe extern "C" fn(*mut std::ffi::c_char)>(b"cerulion_free_error\0")
                .unwrap();
            let shutdown = library
                .get::<extern "C" fn(u64) -> i32>(b"cerulion_node_shutdown\0")
                .unwrap();
            let handle = init(Box::into_raw(Box::new(empty())));
            assert_ne!(handle, 0);
            assert_eq!(
                capture(
                    handle,
                    Some(panicking_native_sink),
                    std::ptr::null_mut(),
                    u64::MAX
                ),
                -6
            );
            let text = take();
            assert!(!text.is_null());
            assert_eq!(std::ffi::CStr::from_ptr(text).to_str().unwrap(), detail);
            free(text);
            assert_eq!(shutdown(handle), 0);
        }
        let mut node = DylibNodeEntry::load(&path).unwrap();
        node.init(empty()).unwrap();
        let mut sink = PanicSink {
            writes: 0,
            refused: 0,
        };
        let error = node.capture_state(&mut sink).unwrap_err();
        let cerulion_core::error::TransportError::GraphError { reason } = error else {
            panic!("sink panic must preserve native state error class: {error}");
        };
        assert!(
            reason.contains("state capture failed (code -6):"),
            "{reason}"
        );
        assert!(reason.ends_with(detail), "{reason}");
        assert_eq!((sink.writes, sink.refused), (1, 0));
        node.shutdown().unwrap();
    }
}

#[test]
fn raw_frame_second_commit_on_block_output_returns_named_error_and_preserves_depth() {
    use cerulion_core::credit::CreditWord;
    use cerulion_core::error::TransportError;
    use cerulion_core::graph::node::AnyPublisher;
    use cerulion_core::graph::node_host_abi::HostPortSpec;
    use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};

    type Text = native_ros2_messages::std_msgs::String;

    let clock = Arc::new(VirtualClock::new());
    clock.set(777);
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "node_plugin_raw_block_dup".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        iceoryx_test_config(),
    )
    .unwrap();

    let word_a = CreditWord::local(1);
    let word_b = CreditWord::local(1);
    let word_spill = CreditWord::local(2);

    let mut pub_a = manager
        .create_publisher("abi/raw/block/a", MaxSliceLen::try_new(64).unwrap(), 0)
        .unwrap();
    pub_a.register_block_outstanding_named_for_test(word_a.clone(), "consumer", "inp_a");
    let mut pub_b = manager
        .create_publisher("abi/raw/block/b", MaxSliceLen::try_new(64).unwrap(), 0)
        .unwrap();
    pub_b.register_block_outstanding_named_for_test(word_b.clone(), "consumer", "inp_b");
    let pub_free = manager
        .create_publisher("abi/raw/free", MaxSliceLen::try_new(64).unwrap(), 0)
        .unwrap();
    let mut pub_spill = manager
        .create_publisher(
            "abi/raw/block/spill",
            MaxSliceLen::try_new(4096).unwrap(),
            0,
        )
        .unwrap();
    pub_spill.register_block_outstanding_named_for_test(
        word_spill.clone(),
        "spill_consumer",
        "text_in",
    );

    let mut sub_a = manager.create_subscriber("abi/raw/block/a").unwrap();
    let mut sub_b = manager.create_subscriber("abi/raw/block/b").unwrap();
    let sub_free = manager.create_subscriber("abi/raw/free").unwrap();
    let mut sub_spill = manager.create_subscriber("abi/raw/block/spill").unwrap();

    let mut publishers = IndexMap::new();
    publishers.insert("blk_a".into(), AnyPublisher::Ipc(pub_a));
    publishers.insert("blk_b".into(), AnyPublisher::Ipc(pub_b));
    publishers.insert("free_out".into(), AnyPublisher::Ipc(pub_free));
    publishers.insert("spill_blk".into(), AnyPublisher::Ipc(pub_spill));

    let env = Arc::new(HashMap::new());
    let mut native = NodeContext::with_runtime_env(
        publishers,
        IndexMap::new(),
        clock,
        ShutdownSignal::new(),
        env.clone(),
    );
    native.set_transport(manager.clone());
    native.set_node_id_for_test("raw_producer");

    let vec_layout = PluginWireLayout {
        schema_hash: Vector3::SCHEMA_HASH,
        fixed_size: 24,
        variable_fields: 0,
        alignment: 8,
    };
    let text_layout = PluginWireLayout {
        schema_hash: Text::SCHEMA_HASH,
        fixed_size: Text::WIRE_FIXED_SIZE as u32,
        variable_fields: Text::VARIABLE_FIELD_COUNT as u32,
        alignment: 8,
    };
    let specs = vec![
        HostPortSpec {
            name: "blk_a".into(),
            direction: LeaseDirection::Write,
            layout: vec_layout,
            read_mode: 0,
            accumulate_cap: None,
        },
        HostPortSpec {
            name: "blk_b".into(),
            direction: LeaseDirection::Write,
            layout: vec_layout,
            read_mode: 0,
            accumulate_cap: None,
        },
        HostPortSpec {
            name: "free_out".into(),
            direction: LeaseDirection::Write,
            layout: vec_layout,
            read_mode: 0,
            accumulate_cap: None,
        },
        HostPortSpec {
            name: "spill_blk".into(),
            direction: LeaseDirection::Write,
            layout: text_layout,
            read_mode: 0,
            accumulate_cap: None,
        },
    ];
    let context = ContextHandle {
        value: register_context(HostContext::new(native, specs, env, 4).unwrap()).unwrap(),
    };
    // SAFETY (U05): process HOST_API and registered context outlive client and plugin.
    let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
    let info = DylibNodeEntry::parse_info_json_labeled(
        &format!(
            r#"{{"inputs":[],"outputs":[{{"name":"blk_a","schema_hash":{vec},"max_slice_len_default":64}},{{"name":"blk_b","schema_hash":{vec},"max_slice_len_default":64}},{{"name":"free_out","schema_hash":{vec},"max_slice_len_default":64}},{{"name":"spill_blk","schema_hash":{text},"max_slice_len_default":4096}}],"policy":{{"period_ms":1}}}}"#,
            vec = Vector3::SCHEMA_HASH,
            text = Text::SCHEMA_HASH
        ),
        "raw block dup",
    )
    .unwrap();
    with_context(context.value, HostContext::begin_node_call).unwrap();
    let mut plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();

    let write_vec3 = |bytes: &mut [u8], vals: [f64; 3]| {
        let mut header = WireHeader::new(Vector3::SCHEMA_HASH, 0, 0);
        header.total_size = 64;
        header.offset_table_offset = 64;
        header.write_to_buf(bytes);
        for (i, v) in vals.into_iter().enumerate() {
            bytes[40 + i * 8..48 + i * 8].copy_from_slice(&v.to_le_bytes());
        }
    };

    // Fire 1: one commit on blk_a, one commit on distinct block output blk_b,
    // three commits on non-block free_out, plus refused 2nd and 3rd commits on blk_a.
    with_context(context.value, HostContext::begin_fire).unwrap();
    {
        let mut loan = plugin
            .publisher_mut("blk_a")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        write_vec3(loan.bytes_mut(), [1.0, 2.0, 3.0]);
        loan.commit().unwrap();
    }
    assert_eq!(word_a.outstanding(), 1);

    // Distinct block output on the same node in the same fire succeeds.
    {
        let mut loan = plugin
            .publisher_mut("blk_b")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        write_vec3(loan.bytes_mut(), [10.0, 20.0, 30.0]);
        loan.commit().unwrap();
    }
    assert_eq!(word_b.outstanding(), 1);

    // Non-block output on the same node commits multiple frames in one fire unchanged.
    for k in 0..3 {
        let mut loan = plugin
            .publisher_mut("free_out")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        let base = (k as f64) + 100.0;
        write_vec3(loan.bytes_mut(), [base, base + 1.0, base + 2.0]);
        loan.commit().unwrap();
    }

    // Second and adversarial third commit on blk_a in the same fire fail with named error.
    let expected_a_err = "node 'raw_producer' output 'blk_a' on topic 'abi/raw/block/a' already committed a frame in this fire; block consumer 'consumer.inp_a' allows at most one commit per output per fire";
    for bad in [[9.0, 9.0, 9.0], [8.0, 8.0, 8.0]] {
        let mut loan = plugin
            .publisher_mut("blk_a")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        write_vec3(loan.bytes_mut(), bad);
        let err = loan.commit().unwrap_err();
        let TransportError::HostServiceFailed { reason } = err else {
            panic!("expected HostServiceFailed, got {err:?}");
        };
        assert_eq!(reason, expected_a_err);
        assert!(!client.has_failure());
        assert_eq!(
            with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
            0
        );
        assert_eq!(word_a.outstanding(), 1);
    }

    // Verify exact delivered frames: blk_a received only frame 1, blk_b received its frame,
    // free_out received all 3 frames.
    let view_a = sub_a
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .expect("blk_a frame 1 delivered");
    assert_eq!(view_a.header().sequence, 0);
    assert_eq!(&view_a[40..48], &1.0f64.to_le_bytes());
    assert_eq!(&view_a[48..56], &2.0f64.to_le_bytes());
    assert_eq!(&view_a[56..64], &3.0f64.to_le_bytes());
    drop(view_a);
    assert!(sub_a
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .is_none());
    word_a.record_drained(1);

    let view_b = sub_b
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .expect("blk_b frame delivered");
    assert_eq!(view_b.header().sequence, 0);
    assert_eq!(&view_b[40..48], &10.0f64.to_le_bytes());
    drop(view_b);
    word_b.record_drained(1);

    let mut free_frames = Vec::new();
    let drained_free = sub_free
        .try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            free_frames.push((msg.header().sequence, x));
        })
        .unwrap();
    assert_eq!(drained_free, 3);
    assert_eq!(free_frames, vec![(0, 100.0), (1, 101.0), (2, 102.0)]);

    // Fire 2: discarding a loan before committing once on blk_a in the same fire succeeds.
    with_context(context.value, HostContext::begin_fire).unwrap();
    {
        let discarded = plugin
            .publisher_mut("blk_a")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        drop(discarded);
    }
    {
        let mut loan = plugin
            .publisher_mut("blk_a")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        write_vec3(loan.bytes_mut(), [4.0, 5.0, 6.0]);
        loan.commit().unwrap();
    }
    assert_eq!(word_a.outstanding(), 1);
    let view_a2 = sub_a
        .view_raw_expecting(Vector3::SCHEMA_HASH)
        .unwrap()
        .expect("blk_a fire 2 frame delivered");
    assert_eq!(view_a2.header().sequence, 1);
    assert_eq!(&view_a2[40..48], &4.0f64.to_le_bytes());
    assert_eq!(&view_a2[48..56], &5.0f64.to_le_bytes());
    assert_eq!(&view_a2[56..64], &6.0f64.to_le_bytes());
    drop(view_a2);
    word_a.record_drained(1);

    // Adaptive spill refusal path through RawFrameLoan::commit: warm up sizer
    // with small frames across fires, then commit 1 small + 1 large spill in one fire.
    let write_text_header = |bytes: &mut [u8], total_size: u32| {
        let mut header = WireHeader::new(Text::SCHEMA_HASH, 0, 0);
        header.total_size = total_size;
        header.offset_table_offset = WireHeader::SIZE as u32;
        header.offset_table_count = Text::VARIABLE_FIELD_COUNT as u32;
        header.write_to_buf(bytes);
    };
    let warmup = cerulion_core::transport::adaptive_sizer::WINDOW_SIZE as u32;
    for seq in 0..warmup {
        with_context(context.value, HostContext::begin_fire).unwrap();
        let mut loan = plugin
            .publisher_mut("spill_blk")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Text>(), 48)
            .unwrap();
        write_text_header(loan.bytes_mut(), 48);
        loan.commit().unwrap();
        let view = sub_spill
            .view_raw_expecting(Text::SCHEMA_HASH)
            .unwrap()
            .expect("spill warmup frame");
        assert_eq!(view.header().sequence, seq);
        drop(view);
        word_spill.record_drained(1);
    }

    with_context(context.value, HostContext::begin_fire).unwrap();
    {
        let mut first = plugin
            .publisher_mut("spill_blk")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Text>(), 56)
            .unwrap();
        write_text_header(first.bytes_mut(), 56);
        first.bytes_mut()[40..44].copy_from_slice(&8u32.to_le_bytes());
        first.bytes_mut()[44..48].copy_from_slice(&8u32.to_le_bytes());
        first.bytes_mut()[48..56].copy_from_slice(b"keep-one");
        first.commit().unwrap();
    }
    assert_eq!(word_spill.outstanding(), 1);
    {
        let mut spill = plugin
            .publisher_mut("spill_blk")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Text>(), 3040)
            .unwrap();
        let bytes = spill.bytes_mut();
        bytes[40..44].copy_from_slice(&8u32.to_le_bytes());
        bytes[44..48].copy_from_slice(&2992u32.to_le_bytes());
        bytes[48..3040].fill(0x5a);
        let err = spill.commit().unwrap_err();
        let TransportError::HostServiceFailed { reason } = err else {
            panic!("expected HostServiceFailed on duplicate spill, got {err:?}");
        };
        assert_eq!(
            reason,
            "node 'raw_producer' output 'spill_blk' on topic 'abi/raw/block/spill' already committed a frame in this fire; block consumer 'spill_consumer.text_in' allows at most one commit per output per fire"
        );
    }
    assert!(!client.has_failure());
    assert_eq!(
        with_context(context.value, |host| Ok(host.outstanding_leases())).unwrap(),
        0
    );
    assert_eq!(
        with_context(context.value, |host| Ok(host.invariant_failures())).unwrap(),
        0
    );
    assert_eq!(word_spill.outstanding(), 1);
    let kept = sub_spill
        .view_raw_expecting(Text::SCHEMA_HASH)
        .unwrap()
        .expect("first spill_blk frame preserved");
    assert_eq!(kept.header().sequence, warmup);
    assert_eq!(&kept[48..56], b"keep-one");
    drop(kept);
    assert!(sub_spill
        .view_raw_expecting(Text::SCHEMA_HASH)
        .unwrap()
        .is_none());

    drop(plugin);
    drop(client);
    with_context(context.value, HostContext::end_node_call).unwrap();
    remove_context(context.value).unwrap();
}

#[test]
fn raw_frame_duplicate_block_commit_refusal_is_deterministic() {
    use cerulion_core::credit::CreditWord;
    use cerulion_core::error::TransportError;
    use cerulion_core::graph::node::AnyPublisher;
    use cerulion_core::graph::node_host_abi::{HostLeaseCounters, HostPortSpec};
    use cerulion_core::transport::plugin_lease::{LeaseDirection, PluginWireLayout};

    let run_once = || -> (String, Vec<u8>, u64, HostLeaseCounters) {
        let clock = Arc::new(VirtualClock::new());
        clock.set(1_234_567);
        let manager = TransportManager::init_for_test(
            TransportConfig {
                node_name: "node_plugin_raw_block_det".into(),
                clock: clock.clone(),
                subscriber_buffer_size: 8,
                network: None,
            },
            iceoryx_test_config(),
        )
        .unwrap();
        let word = CreditWord::local(1);
        let mut publisher = manager
            .create_publisher("abi/raw/block/det", MaxSliceLen::try_new(64).unwrap(), 0)
            .unwrap();
        publisher.register_block_outstanding_named_for_test(word.clone(), "sink", "in_vec");
        let mut subscriber = manager.create_subscriber("abi/raw/block/det").unwrap();
        let mut publishers = IndexMap::new();
        publishers.insert("out".into(), AnyPublisher::Ipc(publisher));
        let env = Arc::new(HashMap::new());
        let mut native = NodeContext::with_runtime_env(
            publishers,
            IndexMap::new(),
            clock,
            ShutdownSignal::new(),
            env.clone(),
        );
        native.set_transport(manager.clone());
        native.set_node_id_for_test("det_producer");
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
            value: register_context(HostContext::new(native, specs, env, 2).unwrap()).unwrap(),
        };
        // SAFETY (U05): process HOST_API and registered context outlive client and plugin.
        let client = unsafe { HostClient::connect(&HOST_API, context, 0) }.unwrap();
        let info = DylibNodeEntry::parse_info_json_labeled(
            &format!(
                r#"{{"inputs":[],"outputs":[{{"name":"out","schema_hash":{},"max_slice_len_default":64}}],"policy":{{"period_ms":1}}}}"#,
                Vector3::SCHEMA_HASH
            ),
            "raw block det",
        )
        .unwrap();
        with_context(context.value, HostContext::begin_node_call).unwrap();
        let mut plugin = PluginContext::new(HostClient::share(&client), &info).unwrap();
        with_context(context.value, HostContext::begin_fire).unwrap();

        let write_vec3_header = |bytes: &mut [u8]| {
            let mut header = WireHeader::new(Vector3::SCHEMA_HASH, 0, 0);
            header.total_size = 64;
            header.offset_table_offset = 64;
            header.write_to_buf(bytes);
        };
        let mut first = plugin
            .publisher_mut("out")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        write_vec3_header(first.bytes_mut());
        first.bytes_mut()[40..48].copy_from_slice(&1.5f64.to_le_bytes());
        first.bytes_mut()[48..56].copy_from_slice(&(-2.5f64).to_le_bytes());
        first.bytes_mut()[56..64].copy_from_slice(&3.5f64.to_le_bytes());
        first.commit().unwrap();

        let mut second = plugin
            .publisher_mut("out")
            .unwrap()
            .loan_raw_frame(HostClient::request::<Vector3>(), 64)
            .unwrap();
        write_vec3_header(second.bytes_mut());
        second.bytes_mut()[40..48].copy_from_slice(&99.0f64.to_le_bytes());
        let TransportError::HostServiceFailed { reason } = second.commit().unwrap_err() else {
            panic!("expected HostServiceFailed");
        };

        let frame = subscriber
            .view_raw_expecting(Vector3::SCHEMA_HASH)
            .unwrap()
            .expect("first frame delivered");
        let delivered_bytes = frame[..].to_vec();
        drop(frame);
        let outstanding = word.outstanding();
        let counters = with_context(context.value, |host| Ok(host.counters())).unwrap();
        drop(plugin);
        drop(client);
        with_context(context.value, HostContext::end_node_call).unwrap();
        remove_context(context.value).unwrap();
        (reason, delivered_bytes, outstanding, counters)
    };

    let first = run_once();
    let second = run_once();
    assert_eq!(first, second);
    // Independently check against hand-written oracle values.
    assert_eq!(
        first.0,
        "node 'det_producer' output 'out' on topic 'abi/raw/block/det' already committed a frame in this fire; block consumer 'sink.in_vec' allows at most one commit per output per fire"
    );
    assert_eq!(first.1.len(), 64);
    let hdr = WireHeader::read_from_buf(&first.1).unwrap();
    assert_eq!(
        (
            hdr.schema_hash,
            hdr.sequence,
            hdr.timestamp_ns,
            hdr.total_size
        ),
        (Vector3::SCHEMA_HASH, 0, 1_234_567, 64)
    );
    assert_eq!(&first.1[40..48], &1.5f64.to_le_bytes());
    assert_eq!(&first.1[48..56], &(-2.5f64).to_le_bytes());
    assert_eq!(&first.1[56..64], &3.5f64.to_le_bytes());
    assert_eq!(first.2, 1);
    assert_eq!(
        first.3,
        HostLeaseCounters {
            committed: 1,
            discarded: 0,
            released: 0,
            spills: 0,
        }
    );
}
