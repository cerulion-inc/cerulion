// SPDX-License-Identifier: AGPL-3.0-only
//! Absolute `source:` references (leading `/`) and the
//! `PublisherProvisioning::External` arm, end-to-end over real iceoryx2.
//!
//! An absolute source with no in-graph producer is an EXTERNAL topic: the
//! graph builds (validation exempts it), provisions NO port requirements
//! (buffer-ceiling only), and out-of-graph publishers
//! attach freely (no single-writer cap, unlike graph-owned topics).
//!
//! Each test uses an isolated per-test iceoryx2 SHM root (parallel-safe).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Data-trigger consumer of an absolute external topic.
#[cerulion_node]
#[derive(Default)]
struct ExtConsumer {
    #[input(trigger)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
    sum: f64,
}

#[cerulion_node_impl]
impl ExtConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.sum += self.inp.x;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn ext_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext".to_string(),
        prefix: "extg".to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "consumer".to_string(),
            node_type: "ext_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/ext/cam".to_string(),
                ..Default::default()
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "consumer".to_string(),
        Box::new(ExtConsumerEntry::with_state(ExtConsumer {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

#[test]
fn absolute_source_builds_and_external_publishers_attach() {
    // The graph builds with an absolute source that matches NO in-graph
    // output (the validation exemption), takes the External provisioning
    // arm, and the topic admits MULTIPLE out-of-graph publishers — the
    // iceoryx2 create-default (2) applies, not the single-writer 1 a
    // graph-owned topic would carry. Data published by the external
    // writer flows into the node body (data-trigger fires).
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build external graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    // TWO external publishers attach — no single-writer cap on a topic
    // the graph doesn't own. (Mutation oracle: External → SingleWriter in
    // for_topology provisions Some(1) and the second create fails.)
    let mut pubr1 = mgr
        .create_publisher("/ext/cam", MaxSliceLen::const_new(256), 0)
        .expect("external publisher 1 must attach");
    let _pubr2 = mgr
        .create_publisher("/ext/cam", MaxSliceLen::const_new(256), 0)
        .expect("external publisher 2 must attach (no single-writer cap)");

    // External data flows into the graph: publish, then step — the
    // data-trigger consumer must fire and see the payload.
    for i in 1..=5u32 {
        let mut proxy = pubr1.loan_proxy::<Vector3>().expect("loan");
        proxy.x = f64::from(i);
        drop(proxy); // publish
        runtime.step(Duration::from_millis(1));
    }
    assert!(
        fires.load(Ordering::Relaxed) >= 5,
        "the data-trigger consumer must fire for external publishes (got {})",
        fires.load(Ordering::Relaxed)
    );
}

#[test]
fn external_topic_open_carries_no_port_requirements() {
    // Create-order independence: the EXTERNAL writer creates the service
    // FIRST with deliberately SMALL provisioning (max_subscribers 3 —
    // below the would-be in-graph + headroom requirement of 6, above the
    // graph's 2 real consumer ports); the graph builds SECOND and must
    // attach, because External provisioning imposes only the buffer
    // ceiling (its genuine consumer requirement). This is the exact
    // foreign-small-provisioning scenario motivating buffer-ceiling-only.
    // (Mutation oracle — iceoryx2 open requirements are MINIMUMS, so the
    // numbers matter: External → (Some(n+4) = 6 subscribers, _) fails
    // this open against the foreign 3; External → (_, Some(1))
    // publishers PASSES the open (1 ≤ 2) but is killed by the
    // two-publisher probe in the test above.) The transport buffer is 16
    // so the graph's ceiling requirement (max(16, depth 10) = 16)
    // matches the foreign creator's — the ceiling itself is pinned
    // separately below.
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = cerulion_core::transport::TransportConfig {
        node_name: "ext_order_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 16,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(transport_config, ix_config)
            .expect("init");
    let mut foreign_cfg = mgr.default_topic_config();
    foreign_cfg.max_subscribers = Some(3);
    let _external_first = mgr
        .create_publisher_with_topic_config("/ext/cam", MaxSliceLen::const_new(256), 0, foreign_cfg)
        .expect("external writer creates the service first (3 subscriber slots)");

    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(fires);
    let clock = Arc::new(VirtualClock::new());
    let _runtime = GraphRuntime::build(config, factories, &mgr, clock).expect(
        "graph must open the pre-existing small-provisioned external service \
         (buffer-ceiling-only: no subscriber/publisher requirements)",
    );
}

#[test]
fn external_topic_buffer_ceiling_requirement_is_still_real() {
    // "Buffer-ceiling only" means the ceiling requirement REMAINS: a
    // foreign service whose ceiling (8, the transport default here) is
    // below the graph's consumer needs (default depth 10 → ceiling
    // max(8, 10) = 10) must fail the build LOUDLY with the
    // ordering-edge hint — the one requirement an External topic keeps is
    // the graph's genuine consumer requirement. (Mutation oracle:
    // dropping the ceiling from External provisioning would let this
    // build succeed with a silently-undersized queue.)
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let transport_config = cerulion_core::transport::TransportConfig {
        node_name: "ext_ceiling_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 8,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(transport_config, ix_config)
            .expect("init");
    let _external_first = mgr
        .create_publisher("/ext/cam", MaxSliceLen::const_new(256), 0)
        .expect("external writer creates the service at ceiling 8");

    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(fires);
    let clock = Arc::new(VirtualClock::new());
    let msg = match GraphRuntime::build(config, factories, &mgr, clock) {
        Ok(_) => panic!("a ceiling-10 consumer must NOT attach to a ceiling-8 service"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("DoesNotSupportRequestedMinBufferSize")
            && msg.contains("smaller buffer ceiling"),
        "the failure must be the buffer-ceiling requirement with the \
         ordering-edge hint: {msg}"
    );
}

/// Period producer publishing to an absolute `topic:` override.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct TfBroadcaster {
    #[output]
    tf: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl TfBroadcaster {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.tf.x = f64::from(self.n);
        Ok(())
    }
}

#[test]
fn topic_override_publishes_under_absolute_name_with_single_writer() {
    // A `topic: /tf` output publishes under the
    // ABSOLUTE name end-to-end; the topic is graph-OWNED, so
    // single-writer provisioning holds under the override (a rogue
    // external publisher is rejected at port creation) — the contrast
    // with External topics, which admit out-of-graph writers freely.
    let fires = Arc::new(AtomicU64::new(0));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "tf".to_string(),
        prefix: "tfg".to_string(),
        nodes: vec![
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "broadcaster".to_string(),
                node_type: "tf_broadcaster".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "tf".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                isolate: None,
                params: Default::default(),
                ros2: None,
                id: "localizer".to_string(),
                node_type: "ext_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "/tf".to_string(),
                    ..Default::default()
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "broadcaster".to_string(),
        Box::new(TfBroadcasterEntry::new()),
    );
    factories.insert(
        "localizer".to_string(),
        Box::new(ExtConsumerEntry::with_state(ExtConsumer {
            fires: Arc::clone(&fires),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build /tf graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    // Single-writer holds under the override (mutation oracle: routing
    // the publisher through the derived-name formula instead of the
    // override-aware resolver would create the service under the DERIVED name — this
    // rogue create on /tf would then SUCCEED and the arm fails).
    let msg = match mgr.create_publisher("/tf", MaxSliceLen::const_new(256), 0) {
        Ok(_) => panic!("/tf is graph-owned — a second publisher must NOT attach"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("single-writer"),
        "the rejection must name the single-writer contract: {msg}"
    );

    // Data flows under the absolute name end-to-end (the data-trigger
    // consumer fires on the broadcaster's period publishes).
    for _ in 0..5 {
        runtime.step(Duration::from_millis(1));
    }
    assert!(
        fires.load(Ordering::Relaxed) >= 1,
        "the consumer must fire on /tf data (got {})",
        fires.load(Ordering::Relaxed)
    );
}

#[test]
#[tracing_test::traced_test]
fn external_topic_silence_warns_once_after_grace() {
    // The runtime follow-through to the graph-load in-prefix warn — an
    // external
    // topic NO publisher ever attaches to warns exactly once at the
    // grace deadline (clock-gated: deterministic under VirtualClock),
    // then never again. Before the deadline: silence. (Mutation oracle:
    // deleting the check or the warn → 0 lines; removing the drain/
    // one-shot → repeated warns on later steps → >1.)
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(fires);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build external graph");
    // One tick short of the grace deadline: no warn yet.
    runtime.step(Duration::from_millis(
        cerulion_core::graph::EXTERNAL_TOPIC_SILENCE_GRACE_MS - 1,
    ));
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("NO publisher attach"))
            .count();
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "warn must not fire before the grace deadline ({n})"
            ))
        }
    });
    // Cross the deadline: exactly one warn; further steps stay quiet.
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(50));
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("NO publisher attach"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 silence warn, got {n}"))
        }
    });
}

#[test]
#[tracing_test::traced_test]
fn external_topic_with_attached_publisher_never_warns() {
    // The negative arm: a publisher attached before the deadline (even a
    // SILENT one — attach is the signal, not data) suppresses the warn.
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ext_graph(fires);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build external graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let _pubr = mgr
        .create_publisher("/ext/cam", MaxSliceLen::const_new(256), 0)
        .expect("external publisher attaches (publishes nothing)");
    runtime.step(Duration::from_millis(
        cerulion_core::graph::EXTERNAL_TOPIC_SILENCE_GRACE_MS + 10,
    ));
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("NO publisher attach"))
            .count();
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "an attached publisher must suppress the warn ({n})"
            ))
        }
    });
}

#[test]
fn declared_ingress_precreates_full_consumer_capacity_and_delivers_oracle_bytes() {
    use cerulion_core::graph::config::{NetworkBlock, NetworkMode};
    use cerulion_core::graph::node::{
        BackpressurePolicy, ClosureNodeEntry, InputConsume, InputMeta, MacroPolicy, NodeInfo,
    };
    use cerulion_core::transport::network::ReinjectOutcome;
    use cerulion_core::transport::PublisherProvisioning;
    use cerulion_core::wire::{StepTag, WireHeader};
    use std::sync::Mutex;

    for _ in 0..2 {
        let (mut config, mut factories) = ext_graph(Arc::new(AtomicU64::new(0)));
        config.network = Some(NetworkBlock {
            mode: NetworkMode::Peer,
            ingress: vec!["/ext/cam".into()],
            ..Default::default()
        });
        let seen = Arc::new(Mutex::new(Vec::new()));
        let output = seen.clone();
        let entry = ClosureNodeEntry::new(
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
                if let Some(input) = ctx.subscriber_mut("inp") {
                    if let Ok(Some(value)) = input.try_view::<Vector3, _>(|v| v.x.to_bits()) {
                        output.lock().unwrap().push(value);
                    }
                }
                Ok(())
            },
        );
        factories.insert("consumer".into(), Box::new(entry));
        let mut runtime =
            GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
                .unwrap();
        let cfg = &runtime.owned_topic_configs()["/ext/cam"];
        assert_eq!(
            cfg.publisher_provisioning,
            PublisherProvisioning::SingleWriter
        );
        assert!(cfg.subscriber_max_buffer_size >= 48);
        let manager = runtime.test_transport().unwrap().clone();
        let injector = manager
            .create_remote_ingress_injector(
                "/ext/cam",
                Vector3::SCHEMA_HASH,
                MaxSliceLen::const_new(256),
            )
            .unwrap();
        assert!(manager
            .create_publisher("/ext/cam", MaxSliceLen::const_new(256), 0)
            .is_err());
        let mut tap = manager.create_data_only_subscriber("/ext/cam").unwrap();
        let mut oracle = Vec::new();
        for (sequence, x) in [1.25f64, 2.5, 5.0].into_iter().enumerate() {
            let mut frame = vec![0; WireHeader::SIZE + 24];
            WireHeader {
                schema_hash: Vector3::SCHEMA_HASH,
                total_size: 64,
                offset_table_offset: 64,
                offset_table_count: 0,
                sequence: sequence as u32,
                timestamp_ns: 100 + sequence as u64,
                step_tag: StepTag::UNTAGGED,
            }
            .write_to_buf(&mut frame[..WireHeader::SIZE]);
            frame[40..48].copy_from_slice(&x.to_le_bytes());
            frame[48..56].copy_from_slice(&(-2.0f64).to_le_bytes());
            frame[56..64].copy_from_slice(&(3.0f64).to_le_bytes());
            assert!(
                matches!(injector.reinject_raw(&frame), ReinjectOutcome::Injected { recipients } if recipients >= 2)
            );
            oracle.push(frame);
        }
        let mut delivered = Vec::new();
        let mut batch = Vec::new();
        for _ in 0..3 {
            assert_eq!(tap.drain_owned(1, &mut batch).unwrap(), 1);
            delivered.push(batch[0].payload().to_vec());
            batch.clear();
        }
        assert_eq!(delivered, oracle);
        for _ in 0..3 {
            runtime.step(Duration::from_millis(1));
        }
        assert_eq!(
            *seen.lock().unwrap(),
            vec![1.25f64.to_bits(), 2.5f64.to_bits(), 5.0f64.to_bits()]
        );
        drop(delivered);
        drop(injector);
        manager
            .create_remote_ingress_injector(
                "/ext/cam",
                Vector3::SCHEMA_HASH,
                MaxSliceLen::const_new(256),
            )
            .expect("release makes the canonical writer slot reusable");
    }
}
