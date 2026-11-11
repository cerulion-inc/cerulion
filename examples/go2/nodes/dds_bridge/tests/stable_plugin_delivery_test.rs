// SPDX-License-Identifier: AGPL-3.0-only
//! Real DDS writer -> shipped bridge cdylib -> host-owned iceoryx2 slot.
//! Build `cargo build -p dds_bridge` first. The DDS peer is local and needs
//! no robot or ROS installation. Every delivered byte uses a hand oracle.
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{
    AnyPublisher, DylibNodeEntry, ExternalSource, NodeContext, NodeEntry,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_go2_dds::ros2_client::{MessageTypeName, Name};
use cerulion_go2_dds::{best_effort_qos, Go2Participant, ParticipantConfig};
use indexmap::IndexMap;

struct EnvGuard;
impl Drop for EnvGuard {
    fn drop(&mut self) {
        std::env::remove_var(dds_bridge::config::CONFIG_ENV);
    }
}

#[test]
fn stable_bridge_raw_ingress_delivers_exact_frame_with_host_clock() {
    let target = std::env::var_os("CARGO_TARGET_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| {
            std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target")
        });
    let library = if cfg!(target_os = "macos") {
        "libdds_bridge.dylib"
    } else {
        "libdds_bridge.so"
    };
    let mut entry = DylibNodeEntry::load(&target.join("debug").join(library))
        .expect("build the actual dds_bridge cdylib first");
    let clock = Arc::new(VirtualClock::new());
    clock.set(777);
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: "stable_dds_delivery".into(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("isolated host transport");
    let mut outputs = IndexMap::new();
    for name in ["cloud", "odom", "twist", "request_json"] {
        outputs.insert(
            name.to_owned(),
            AnyPublisher::Ipc(
                transport
                    .create_publisher(
                        &format!("/stable_dds/{name}"),
                        MaxSliceLen::const_new(1024 * 1024),
                        0,
                    )
                    .expect("declared output"),
            ),
        );
    }
    let mut context = NodeContext::with_runtime_env(
        outputs,
        IndexMap::new(),
        clock.clone(),
        cerulion_core::ShutdownSignal::new(),
        Arc::new(std::collections::HashMap::new()),
    );
    context.set_transport(transport.clone());
    entry.init(context).expect("real bridge plugin init");
    let topic = "/stable_dds/raw_vector";
    let subscriber = transport
        .create_subscriber(topic)
        .expect("downstream observer");
    let domain = 187;
    let config = tempfile::NamedTempFile::new().expect("config file");
    std::fs::write(config.path(), format!("domain_id: {domain}\nmappings:\n  - dds_topic: /stable_abi_vector\n    ros_type: geometry_msgs/Vector3\n    cerulion_topic: {topic}\n    qos: best_effort\n")).expect("valid raw mapping");
    std::env::set_var(dds_bridge::config::CONFIG_ENV, config.path());
    let _env = EnvGuard;
    let mut source = entry.external_source().expect("stable external descriptor");
    assert!(matches!(source, ExternalSource::Blocking(_) | ExternalSource::Fd(_)), "bridge must expose its real DDS helper");
    if let ExternalSource::Blocking(step) = &mut source { let _ = step(); }
    let participant =
        Go2Participant::new(&ParticipantConfig::new(domain, Vec::new())).expect("local DDS peer");
    let mut node = participant
        .create_node("stable_abi_writer")
        .expect("DDS node");
    let qos = best_effort_qos();
    let dds_topic = node
        .create_topic(
            &Name::new("/", "stable_abi_vector").expect("topic name"),
            MessageTypeName::new("geometry_msgs", "Vector3"),
            &qos,
        )
        .expect("DDS topic");
    let publisher = node
        .create_publisher::<cerulion_go2_dds::messages::Vector3>(&dds_topic, Some(qos))
        .expect("DDS writer");
    let deadline = Instant::now() + Duration::from_secs(15);
    let mut delivered = None;
    while Instant::now() < deadline && delivered.is_none() {
        publisher
            .publish(cerulion_go2_dds::messages::Vector3 {
                x: 1.5,
                y: -2.5,
                z: 3.25,
            })
            .expect("real DDS publish");
        if let ExternalSource::Blocking(step) = &mut source { let _ = step(); }
        entry
            .tick()
            .expect("ordinary ticks remain admitted during raw ingress");
        subscriber
            .try_receive_one(|message| {
                let mut frame = vec![0; WireHeader::SIZE];
                message.header().write_to_buf(&mut frame);
                frame.extend_from_slice(message.payload());
                delivered = Some(frame);
            })
            .expect("host receive");
        if delivered.is_none() {
            std::thread::sleep(Duration::from_millis(25));
        }
    }
    entry.shutdown().expect("bridge teardown");
    let mut expected = vec![0; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::geometry_msgs::Vector3 as ShmMessage>::SCHEMA_HASH,
        total_size: 64,
        offset_table_offset: 64,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns: 777,
        step_tag: cerulion_core::wire::StepTag::UNTAGGED,
    }
    .write_to_buf(&mut expected);
    for value in [1.5_f64, -2.5, 3.25] {
        expected.extend_from_slice(&value.to_le_bytes());
    }
    assert_eq!(delivered, Some(expected), "DDS payload, host clock, sequence and complete wire header must reach downstream unchanged");
}
