// SPDX-License-Identifier: AGPL-3.0-only
//! Two real graph halves, authenticated wire/2 and canonical SHM delivery.

use super::*;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputConsume, InputMeta, MacroPolicy, NodeInfo,
    OutputMeta,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::StepTag;
use cerulion_core::{EgressQos, EgressReliability, LanAccess, SchemaServing};
use native_ros2_messages::geometry_msgs::Vector3;
use std::sync::Mutex;

const TOPIC: &str = "/robot/source/vector";
const PRIVATE: &str = "/robot/source/private";

pub(super) fn frame(sequence: u32) -> Vec<u8> {
    let x = [1.25f64, 2.5, 5.0, 10.0][sequence as usize];
    let mut bytes = vec![0; 64];
    WireHeader {
        schema_hash: Vector3::SCHEMA_HASH,
        total_size: 64,
        offset_table_offset: 64,
        offset_table_count: 0,
        sequence,
        timestamp_ns: 100 + u64::from(sequence),
        step_tag: StepTag::UNTAGGED,
    }
    .write_to_buf(&mut bytes[..40]);
    bytes[40..48].copy_from_slice(&x.to_le_bytes());
    bytes[48..56].copy_from_slice(&(-2.0f64).to_le_bytes());
    bytes[56..64].copy_from_slice(&(3.0f64).to_le_bytes());
    bytes
}

pub(super) fn source_graph(
    source_manager: &TransportManager,
) -> (GraphRuntime, Arc<Mutex<Option<u32>>>) {
    let source_config: GraphConfig = cerulion_core::graph::parse_graph("name: source\nprefix: robot\nnodes:\n  - id: source\n    type: source\n    outputs:\n      - name: vector\n        schema: geometry_msgs/Vector3\n      - name: private\n        schema: geometry_msgs/Vector3\nnetwork:\n  mode: peer\n  egress: [/robot/source/vector]\n").unwrap();
    let command = Arc::new(Mutex::new(None::<u32>));
    let source_command = command.clone();
    let source_entry = ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![],
            vec![
                OutputMeta::new(
                    "vector".into(),
                    Vector3::SCHEMA_HASH,
                    Some(MaxSliceLen::const_new(256)),
                ),
                OutputMeta::new(
                    "private".into(),
                    Vector3::SCHEMA_HASH,
                    Some(MaxSliceLen::const_new(256)),
                ),
            ],
        )
        .with_policy(MacroPolicy::Period { period_ms: 1 }),
        move |ctx| {
            if let Some(sequence) = source_command.lock().unwrap().take() {
                ctx.publisher_mut("vector")
                    .unwrap()
                    .publish_raw(&frame(sequence))?;
            }
            Ok(())
        },
    );
    let source = GraphRuntime::build(
        source_config,
        [(
            "source".into(),
            Box::new(source_entry) as Box<dyn cerulion_core::NodeEntry>,
        )]
        .into_iter()
        .collect(),
        source_manager,
        Arc::new(VirtualClock::new()),
    )
    .unwrap();
    (source, command)
}

fn graph_halves_deliver() -> Vec<Vec<u8>> {
    // The platform rule is independent of environment so this pins the actual
    // default without mutating process environment or selecting Open.
    assert_eq!(LanAccess::platform_default(), LanAccess::Paired);
    let source_manager = test_manager("graph_source");
    let consumer_manager = test_manager("graph_consumer");
    let consumer_config: GraphConfig = cerulion_core::graph::parse_graph("name: consumer\nprefix: desk\nnodes:\n  - id: consumer\n    type: consumer\n    inputs:\n      - name: inp\n        source: /robot/source/vector\nnetwork:\n  mode: peer\n  ingress: [/robot/source/vector]\n").unwrap();
    // This is the same producer scope acquired by the CLI before graph ports.
    let _private_scope = source_manager.exclude_remote_topic(PRIVATE).unwrap();
    let observed = Arc::new(Mutex::new(Vec::new()));
    let consumer_observed = observed.clone();
    let consumer_entry = ClosureNodeEntry::new(
        NodeInfo::with_meta(
            vec![InputMeta {
                name: "inp".into(),
                schema_hash: Vector3::SCHEMA_HASH,
                trigger: true,
                depth: 48,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: None,
                consume: InputConsume::Fifo,
            }],
            vec![],
        )
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "inp".into(),
        }),
        move |ctx| {
            if let Ok(Some(value)) = ctx
                .subscriber_mut("inp")
                .unwrap()
                .try_view::<Vector3, _>(|v| v.x.to_bits())
            {
                consumer_observed.lock().unwrap().push(value);
            }
            Ok(())
        },
    );
    let (mut source, command) = source_graph(&source_manager);
    let mut consumer = GraphRuntime::build(
        consumer_config,
        [(
            "consumer".into(),
            Box::new(consumer_entry) as Box<dyn cerulion_core::NodeEntry>,
        )]
        .into_iter()
        .collect(),
        &consumer_manager,
        Arc::new(VirtualClock::new()),
    )
    .unwrap();
    assert!(consumer.owned_topic_configs()[TOPIC].subscriber_max_buffer_size >= 48);
    let desk_seed = unique_secret();
    let auth = Arc::new(claimed_paired(public_of(&desk_seed)));
    let robot_seed = unique_secret();
    let serving = SchemaServing {
        served_topics: vec![TOPIC.into()],
        egress_qos: vec![EgressQos {
            topic: TOPIC.into(),
            reliability: EgressReliability::Reliable,
            send_queue: std::num::NonZeroUsize::new(2),
        }],
        ..Default::default()
    };
    let robot = RobotFixture::start_graph(
        "graph",
        auth.clone(),
        source_manager.clone(),
        serving.clone(),
        robot_seed,
    );
    let registry = Arc::new(WanRegistry::new(
        HashMap::from([(
            "source".into(),
            WanRobot {
                eid: robot.eid,
                direct_addrs: robot.addrs.clone(),
            },
        )]),
        desk_seed,
        RelayConfig::Disabled,
    ));
    let plane = Arc::new(IrohMirrorPlane::new(consumer_manager.clone(), registry.clone()).unwrap());
    // Catalog hash peeking observes frames produced by the source graph. No
    // demand exists yet, so this warm-up cannot reach the consumer.
    let querying = plane.clone();
    let catalog =
        std::thread::spawn(move || querying.query_catalog("source", Duration::from_secs(5)));
    let deadline = Instant::now() + Duration::from_secs(6);
    while !catalog.is_finished() && Instant::now() < deadline {
        *command.lock().unwrap() = Some(0);
        source.step(Duration::from_millis(1));
        std::thread::sleep(Duration::from_millis(2));
    }
    let catalog = catalog.join().unwrap().unwrap();
    let entry = catalog
        .entries
        .iter()
        .find(|entry| entry.topic == TOPIC)
        .unwrap();
    assert_eq!(entry.schema_hash, Some(Vector3::SCHEMA_HASH));
    assert_eq!(entry.producer_count, Some(1));
    assert_eq!(entry.egress_qos, serving.egress_qos.first().cloned());
    assert!(!catalog.entries.iter().any(|entry| entry.topic == PRIVATE));
    assert!(plane
        .ensure_mirror(&TopicKey::new("source", PRIVATE), Vector3::SCHEMA_HASH)
        .is_err());
    let key = TopicKey::new("source", TOPIC);
    plane.ensure_mirror(&key, Vector3::SCHEMA_HASH).unwrap();
    assert!(consumer_manager
        .create_publisher(TOPIC, MaxSliceLen::const_new(256), 0)
        .is_err());
    let mut tap = consumer_manager.create_data_only_subscriber(TOPIC).unwrap();
    let mut delivered = Vec::new();
    for sequence in 0..3 {
        *command.lock().unwrap() = Some(sequence);
        source.step(Duration::from_millis(1));
        receive_one(&mut consumer, &mut tap, &observed, sequence, &mut delivered);
    }
    assert_eq!(plane.release_mirror(&key), MirrorRelease::Retired);
    // Rebind the same authenticated producer endpoint and rediscover its direct
    // address. The graph and canonical service stay alive through reconnect.
    robot.kill();
    drop(robot);
    let restarted = RobotFixture::start_graph(
        "graph_restarted",
        auth,
        source_manager.clone(),
        serving,
        robot_seed,
    );
    plane
        .update_membership(HashMap::from([(
            "source".into(),
            WanRobot {
                eid: restarted.eid,
                direct_addrs: restarted.addrs.clone(),
            },
        )]))
        .unwrap();
    plane.ensure_mirror(&key, Vector3::SCHEMA_HASH).unwrap();
    *command.lock().unwrap() = Some(3);
    source.step(Duration::from_millis(1));
    receive_one(&mut consumer, &mut tap, &observed, 3, &mut delivered);
    assert_eq!(plane.release_mirror(&key), MirrorRelease::Retired);
    consumer_manager
        .create_remote_ingress_injector(TOPIC, Vector3::SCHEMA_HASH, MaxSliceLen::const_new(256))
        .expect("release frees the canonical writer");
    assert_eq!(
        *observed.lock().unwrap(),
        vec![
            1.25f64.to_bits(),
            2.5f64.to_bits(),
            5.0f64.to_bits(),
            10.0f64.to_bits()
        ]
    );
    assert_eq!(delivered, (0..4).map(frame).collect::<Vec<_>>());
    restarted.kill();
    delivered
}

fn receive_one(
    runtime: &mut GraphRuntime,
    tap: &mut cerulion_core::transport::subscriber::DataOnlySubscriber,
    observed: &Mutex<Vec<u64>>,
    sequence: u32,
    delivered: &mut Vec<Vec<u8>>,
) {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut samples = Vec::new();
    while Instant::now() < deadline {
        runtime.step(Duration::from_millis(1));
        if tap.drain_owned(1, &mut samples).unwrap() == 1 {
            delivered.push(samples[0].payload().to_vec());
            samples.clear();
        }
        if delivered.len() > sequence as usize && observed.lock().unwrap().len() > sequence as usize
        {
            return;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("graph consumer did not receive oracle frame {sequence}");
}

#[test]
fn authenticated_graph_halves_deliver_identical_oracle_bytes_reconnect_and_release() {
    let first = graph_halves_deliver();
    let second = graph_halves_deliver();
    assert_eq!(first, second); // Both runs were independently pinned to frame().
}
