// SPDX-License-Identifier: AGPL-3.0-only
//! The declared `#[input(depth = N)]` IS the input's real
//! iceoryx2 queue, and the topic's service-level `subscriber_max_buffer_size`
//! ceiling is topology-derived as `max(global default, largest consumer
//! depth)` — never below the default, so default openers (CLI introspection,
//! raw transport users) can always attach to graph-created services.
//!
//! Each test uses an isolated per-test iceoryx2 SHM root (parallel-safe).

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::testing::TestTransport;
use cerulion_core::testing::{
    count_at_exclusively, debug_lines_expected, line_level, lines_at_exclusively,
};
use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, INTROSPECTION_SUBSCRIBER_HEADROOM,
};
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Period producer publishing one Vector3 per tick.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DeepProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl DeepProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Stalled block consumer declaring depth 32 — DOUBLE the global default
/// (16), so the topology must raise the topic's service ceiling to 32.
#[cerulion_node(external)]
#[derive(Default)]
struct Depth32Consumer {
    #[input(backpressure = block, depth = 32)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl Depth32Consumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn depth32_graph() -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "depth32".to_string(),
        prefix: "tbs".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "deep_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "depth32_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(DeepProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(Depth32ConsumerEntry::new()),
    );
    (config, factories)
}

#[test]
fn default_opener_attaches_to_raised_ceiling_topic() {
    // A depth-32 input raises the topic ceiling to max(16, 32) = 32. Both
    // halves of the compatibility contract must hold on the LIVE service:
    // (a) a DEFAULT opener (requires the global 16 ≤ 32) attaches — this is
    //     `cerulion topic echo` on a running graph's topic;
    // (b) an opener requiring the full raised ceiling (32) attaches —
    //     proving the service really was provisioned at 32, not the global
    //     default (iceoryx2 rejects openers that require more than the
    //     creator provisioned).
    let (config, factories) = depth32_graph();
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build depth32 graph");
    let mgr = Arc::clone(runtime.test_transport().expect("test transport parked"));
    let topic = "/tbs/producer/out";

    if let Err(e) = mgr.create_subscriber(topic) {
        panic!(
            "a default opener (global buffer 16) must attach to the \
             raised-ceiling (32) topic: {e}"
        );
    }
    if let Err(e) = mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            32,
            1,
            PublisherProvisioning::SingleWriter,
            // `history_size` (native iceoryx2 history); 0 here.
            0,
            // A raw opener carries no standalone listeners.
            0,
        ),
        32,
    ) {
        panic!(
            "an opener requiring the full raised ceiling (32) must attach — \
             the service must have been provisioned at the topology ceiling: {e}"
        );
    }

    // The inverse direction must be LOUD: an opener requiring MORE than the
    // provisioned ceiling (33 > 32) is rejected by iceoryx2 itself — this is
    // the only in-suite observation of the native open-requirement check
    // (without it, the full-ceiling attach above would be vacuous). Require
    // 33 specifically: success-at-32 + failure-at-33 pins the service was
    // provisioned at EXACTLY the topology ceiling. Buffer 33 ≤ 33 passes
    // Cerulion's pre-check, so the failure genuinely comes from the
    // iceoryx2 open.
    match mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            33,
            1,
            PublisherProvisioning::SingleWriter,
            // `history_size` (native iceoryx2 history); 0 here.
            0,
            // A raw opener carries no standalone listeners.
            0,
        ),
        33,
    ) {
        Ok(_) => {
            panic!("an opener requiring ceiling 33 must NOT attach to the 32-provisioned service")
        }
        Err(e) => {
            let msg = e.to_string();
            assert!(
                msg.contains("33") && msg.contains("smaller buffer ceiling"),
                "the open error must carry the requested ceiling AND the \
                 ordering hint (actionable context appended in \
                 open_topic_services): {msg}"
            );
        }
    }

    // Subscriber-slot provisioning is EXACT. The graph has one
    // in-graph subscriber (the body sub; no trigger drains), so the topic is
    // provisioned at 1 + INTROSPECTION_SUBSCRIBER_HEADROOM slots: an opener
    // requiring exactly that attaches; one requiring MORE is rejected by
    // iceoryx2 itself (with the subscriber-slot ordering hint).
    // for_topology(.., in_graph_subs = n) → Some(n + HEADROOM). Written
    // against the constant (it was raised 4 → 5 for the standing liveness
    // observer, and hard-coded literals here silently mis-stated the contract).
    if let Err(e) = mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            32,
            1,
            PublisherProvisioning::SingleWriter,
            // `history_size` (native iceoryx2 history); 0 here.
            0,
            // A raw opener carries no standalone listeners.
            0,
        ),
        1,
    ) {
        panic!(
            "an opener requiring the provisioned {} subscriber slots must attach: {e}",
            1 + INTROSPECTION_SUBSCRIBER_HEADROOM
        );
    }
    match mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            32,
            2,
            PublisherProvisioning::SingleWriter,
            // `history_size` (native iceoryx2 history); 0 here.
            0,
            // A raw opener carries no standalone listeners.
            0,
        ),
        1,
    ) {
        Ok(_) => panic!(
            "an opener requiring {} subscriber slots must NOT attach (provisioned {})",
            2 + INTROSPECTION_SUBSCRIBER_HEADROOM,
            1 + INTROSPECTION_SUBSCRIBER_HEADROOM
        ),
        Err(e) => {
            let msg = e.to_string();
            assert!(
                msg.contains(&format!(
                    "max_subscribers {}",
                    2 + INTROSPECTION_SUBSCRIBER_HEADROOM
                )) && msg.contains("fewer subscriber slots"),
                "the open error must carry the requested slot count AND the \
                 subscriber-slot ordering hint: {msg}"
            );
        }
    }

    // And the declared depth is the REAL queue: the producer plateaus at 32
    // fires against the stalled consumer (depth honored end-to-end).
    for _ in 0..40 {
        runtime.step(Duration::from_millis(10));
    }
    assert_eq!(
        runtime.node_handle("producer").unwrap().fire_count(),
        32,
        "producer must plateau at the declared depth 32 — the input's queue \
         is its declaration, not the global default"
    );
}

#[test]
fn subscriber_buffer_above_ceiling_rejected_loudly() {
    // The defensive pre-check in `create_subscriber_with_buffers`: a buffer
    // request above the topic's ceiling is a wiring bug (the runtime
    // guarantees depth ≤ ceiling by construction) — it must surface a clear
    // error naming both numbers, not a bare iceoryx2 builder failure.
    let tt = TestTransport::with_buffer_size(16);
    let _pubr = tt.publisher("tbs_reject/out", MaxSliceLen::const_new(256), 0);
    let msg = match tt.subscriber_with_buffers("tbs_reject/out", tt.default_topic_config(), 32) {
        Ok(_) => panic!("buffer 32 > ceiling 16 must be rejected"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("32") && msg.contains("16") && msg.contains("ceiling"),
        "the rejection must name the requested buffer, the ceiling, and the \
         word 'ceiling': {msg}"
    );
    // The create-site zero check (this pub entry point bypasses
    // init validation) + its boundary — 0 rejected, 1 (the legal floor)
    // accepted. Kills `== 0` → never / `<= 1` at THIS site (the init-site
    // boundary is pinned separately).
    let msg = match tt.subscriber_with_buffers("tbs_reject/out", tt.default_topic_config(), 0) {
        Ok(_) => panic!("buffer 0 must be rejected at create_subscriber_with_buffers"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains(">= 1"),
        "the rejection must state the bound: {msg}"
    );
    tt.subscriber_with_buffers("tbs_reject/out", tt.default_topic_config(), 1)
        .expect("buffer 1 is the legal floor at the create entry point");
}

#[test]
fn raw_subscriber_buffer_is_behaviorally_real() {
    // The counter-blindness fix: the plateau assertions above observe
    // the producer-side counter, not the queue. This observes the queue:
    // on a global-4 transport, a 16-buffer subscriber must RETAIN all 16
    // published frames (a silently-4 queue would evict 12), and a 2-buffer
    // subscriber must retain exactly the newest 2.
    let tt = TestTransport::with_buffer_size(4);
    let cfg = tt.default_topic_config();
    // ORDER MATTERS (the one-directional open semantics this test
    // documents): the raised-ceiling opener must CREATE the service —
    // a default `tt.publisher` first would create it at ceiling 4 and
    // lock the 16-requiring subscriber out (the native rejection it
    // would hit is pinned by the over-demand arm of the raised-ceiling
    // test; the default-creates-first ordering itself is the pre-create
    // pass's edge).
    let deep = tt
        .subscriber_with_buffers(
            "tbs_real/out",
            cerulion_core::transport::TopicServiceConfig::for_topology(
                cfg,
                16,
                2,
                PublisherProvisioning::SingleWriter,
                // `history_size` (native iceoryx2 history); 0 here.
                0,
                // A raw opener carries no standalone listeners.
                0,
            ),
            16,
        )
        .expect("16-buffer subscriber on a raised-ceiling topic");
    let mut pubr = tt.publisher("tbs_real/out", MaxSliceLen::const_new(256), 0);
    let shallow = tt
        .subscriber_with_buffers(
            "tbs_real/out",
            cerulion_core::transport::TopicServiceConfig::for_topology(
                cfg,
                16,
                2,
                PublisherProvisioning::SingleWriter,
                // `history_size` (native iceoryx2 history); 0 here.
                0,
                // A raw opener carries no standalone listeners.
                0,
            ),
            2,
        )
        .expect("2-buffer subscriber on the same topic");
    for i in 0..16u32 {
        let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
        proxy.x = f64::from(i + 1);
    }
    let mut deep_seen = Vec::new();
    let n = deep
        .try_receive(|msg| {
            let bytes = msg.payload();
            deep_seen.push(f64::from_le_bytes(bytes[0..8].try_into().unwrap()));
        })
        .expect("drain deep");
    assert_eq!(
        n, 16,
        "the 16-buffer queue must retain ALL 16 frames — the requested \
         buffer is the real queue (a silent global-4 queue keeps only 4)"
    );
    assert_eq!(deep_seen.first().copied(), Some(1.0));
    assert_eq!(deep_seen.last().copied(), Some(16.0));
    let mut shallow_seen = Vec::new();
    let n = shallow
        .try_receive(|msg| {
            let bytes = msg.payload();
            shallow_seen.push(f64::from_le_bytes(bytes[0..8].try_into().unwrap()));
        })
        .expect("drain shallow");
    assert_eq!(
        n, 2,
        "the 2-buffer queue must retain exactly the newest 2 of 16 frames"
    );
    assert_eq!(shallow_seen, vec![15.0, 16.0]);
}

/// Stalled depth-2 block consumer for the low-ceiling compatibility pin.
#[cerulion_node(external)]
#[derive(Default)]
struct Depth2Consumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl Depth2Consumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn default_opener_attaches_to_low_depth_topic() {
    // The other half of the ceiling formula (mutation M2): when every
    // declared depth is below the global default, the ceiling must still be
    // floored at the default — dropping the floor (ceiling = max depth = 2)
    // would lock default openers (`cerulion topic echo` requires the global
    // 16) out of the topic.
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "low_depth".to_string(),
        prefix: "tbsl".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "deep_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "depth2_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(DeepProducerEntry::new()));
    factories.insert("consumer".to_string(), Box::new(Depth2ConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build low-depth graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    if let Err(e) = mgr.create_subscriber("/tbsl/producer/out") {
        panic!(
            "a default opener (global 16) must attach to a topic whose max \
             declared depth (2) is below the default — the ceiling is \
             floored at the default: {e}"
        );
    }
}

#[test]
fn zero_subscriber_buffer_rejected_at_init() {
    // iceoryx2 silently clamps a 0 buffer to 1
    // under a log level Cerulion suppresses — the only zero source is
    // transport init, which must reject it loudly instead.
    let config = cerulion_core::transport::TransportConfig {
        node_name: "zero_buf_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 0,
        network: None,
    };
    let err = match cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    ) {
        Ok(_) => panic!("subscriber_buffer_size = 0 must be rejected at init"),
        Err(e) => e.to_string(),
    };
    assert!(
        err.contains(">= 1"),
        "the rejection must state the bound: {err}"
    );
    // Boundary (mutation kill: `== 0` tightened to `<= 1` would
    // reject a legal minimal config): exactly 1 must be accepted.
    let config = cerulion_core::transport::TransportConfig {
        node_name: "one_buf_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 1,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("subscriber_buffer_size = 1 is legal and must be accepted");
    // A hand-mutated zero-ceiling config must be rejected at the
    // open choke point. Only the publisher path can reach it (the
    // subscriber entry point's zero-buffer + buffer>ceiling pre-checks
    // both fire first on that path); iceoryx2 would otherwise silently
    // clamp the ceiling 0 → 1 under a suppressed warn.
    let mut zero_ceiling = mgr.default_topic_config();
    zero_ceiling.subscriber_max_buffer_size = 0;
    let msg = match mgr.create_publisher_with_topic_config(
        "tbs_zero_ceiling/out",
        MaxSliceLen::const_new(256),
        0,
        zero_ceiling,
    ) {
        Ok(_) => panic!("a zero-ceiling config must be rejected at the open choke point"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains(">= 1"),
        "the rejection must state the bound: {msg}"
    );
    // The max_subscribers twins of the same guard class —
    // Some(0) (suppressed clamp) and a garbage huge value (giant
    // allocation as a bare error) are both rejected at the choke point.
    let mut zero_subs = mgr.default_topic_config();
    zero_subs.max_subscribers = Some(0);
    let msg = match mgr.create_publisher_with_topic_config(
        "tbs_zero_subs/out",
        MaxSliceLen::const_new(256),
        0,
        zero_subs,
    ) {
        Ok(_) => panic!("max_subscribers Some(0) must be rejected at the open choke point"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains(">= 1 when set"),
        "the rejection must state the bound: {msg}"
    );
    let mut huge_subs = mgr.default_topic_config();
    huge_subs.max_subscribers = Some(4097);
    let msg = match mgr.create_publisher_with_topic_config(
        "tbs_huge_subs/out",
        MaxSliceLen::const_new(256),
        0,
        huge_subs,
    ) {
        Ok(_) => panic!("max_subscribers above the sanity bound must be rejected"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("<= 4096"),
        "the rejection must state the bound: {msg}"
    );
    mgr.create_publisher("tbs_ceiling_one/out", MaxSliceLen::const_new(256), 0)
        .expect("ceiling 1 (this mgr's default config) is the legal floor at the open choke point");
}

/// Slow drop_oldest consumer with depth 2 for the history-warn pin.
#[cerulion_node(period_ms = 60)]
#[derive(Default)]
struct ShallowHistoryConsumer {
    #[input(backpressure = drop_oldest, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ShallowHistoryConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Sibling consumer whose depth EQUALS the topic's history_size (8) —
/// exactly-fits, zero eviction, must NOT warn. Its presence makes the
/// exactly-one warn count a real oracle (see the test doc).
#[cerulion_node(period_ms = 60)]
#[derive(Default)]
struct ExactFitHistoryConsumer {
    #[input(backpressure = drop_oldest, depth = 8)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ExactFitHistoryConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

#[tracing_test::traced_test]
#[test]
fn history_above_input_depth_warns_at_build() {
    // An input depth below the topic's history_size means a
    // late joiner's real queue silently evicts the oldest replayed frames
    // before its first drain (and SentHistory still fires). The wiring
    // must warn per shallow edge — loud inference at the boundary.
    //
    // Mutation oracle (corrected in review — the graph carries both a
    // depth-2 shallow edge and a depth-8 exactly-fits edge on the same
    // history-8 topic, so the exactly-one count distinguishes all three
    // regressions): deleting the warn → 0; warning on every edge
    // unconditionally → 2; `<` loosened to `<=` (false positive at
    // exactly-fits, where the queue holds ALL replayed frames) → 2.
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "shallow_history".to_string(),
        prefix: "tbsh".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "deep_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 8,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "shallow_history_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "exact_fit".to_string(),
                node_type: "exact_fit_history_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(DeepProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(ShallowHistoryConsumerEntry::new()),
    );
    factories.insert(
        "exact_fit".to_string(),
        Box::new(ExactFitHistoryConsumerEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    let _runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build shallow-history graph");
    logs_assert(|lines: &[&str]| {
        // The warn names what the consumer receives. The structured-logging
        // rule moved the offending VALUES out of the message
        // and into structured fields (`topic=`, `node=`, `input=`, `depth=`,
        // `history_size=`), so the stable half of the line is the prose
        // "input depth is below the topic's history_size" plus the
        // `history_size=` field the operator greps by.
        // count_at_exclusively: the WARN token AND the level-free total of the
        // same conjunction — a warn demoted to INFO/ERROR is not this warn, and
        // a second copy of it at another level is not one either.
        let n = count_at_exclusively(
            lines,
            "WARN",
            &["input depth is below the topic's", "history_size"],
        )?;
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 shallow-history warn, got {n}"))
        }
    });
}

#[test]
fn subscriber_slots_exhaust_at_provisioned_headroom() {
    // The cap is REAL at port creation, and the headroom value is
    // exactly INTROSPECTION_SUBSCRIBER_HEADROOM. The depth32 graph provisions
    // 1 in-graph + HEADROOM slots; its body subscriber holds one, so exactly
    // HEADROOM more default openers attach — the next one fails at iceoryx2
    // port creation (slot exhaustion, a loud SubscriberCreation error).
    // (Mutation oracle: HEADROOM-1 fails the last attach; HEADROOM+1 lets the
    // extra one attach and fails the rejection arm. Written against the
    // constant so the bump 4 → 5 moves the probe with it rather than
    // silently re-describing a different contract.)
    let (config, factories) = depth32_graph();
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build depth32 graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let topic = "/tbs/producer/out";
    let mut held = Vec::new();
    for i in 0..INTROSPECTION_SUBSCRIBER_HEADROOM {
        match mgr.create_subscriber(topic) {
            Ok(sub) => held.push(sub),
            Err(e) => panic!(
                "headroom opener {i} of {INTROSPECTION_SUBSCRIBER_HEADROOM} must attach: {e}"
            ),
        }
    }
    match mgr.create_subscriber(topic) {
        Ok(_) => panic!(
            "extra subscriber {} must NOT attach — the topic is provisioned at \
             exactly in-graph (1) + headroom ({INTROSPECTION_SUBSCRIBER_HEADROOM}) slots",
            INTROSPECTION_SUBSCRIBER_HEADROOM + 1
        ),
        Err(e) => {
            let msg = e.to_string();
            assert!(
                msg.contains("ExceedsMaxSupportedSubscribers")
                    && msg.contains(&format!(
                        "all {} of the topic's subscriber slots",
                        1 + INTROSPECTION_SUBSCRIBER_HEADROOM
                    ))
                    && msg.contains("topic echo"),
                "slot exhaustion must name the cause, the provisioned count, \
                 and the remedy (the bare variant + generic \
                 check-your-YAML advice pointed operators wrong): {msg}"
            );
        }
    }
}

/// The `data_service_missing` DISCRIMINATOR that
/// `topic echo`/`hz`/`info`'s stale-mirror fall-through keys on must distinguish a
/// topic that is GONE (fall through to the remote rung) from one that EXISTS but
/// whose subscriber OPEN failed for an ACTIONABLE reason (slot exhaustion,
/// type-skew — surface it directly). It opens only the data-service FACTORY
/// (never a subscriber PORT), so a slot-EXHAUSTED live topic still reads as
/// PRESENT (`missing == false` ⇒ surface the crafted slot-exhaustion hint), while
/// a never-created topic reads as ABSENT (`missing == true` ⇒ fall through).
/// Pre-fix, `ensure_topic_available` swallowed EVERY open failure into the remote
/// not-found. `data_service_missing` returning `true` for the
/// exhausted live topic would mask the actionable hint.
#[test]
fn data_service_missing_distinguishes_stale_mirror_from_slot_exhaustion() {
    let (config, factories) = depth32_graph();
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build depth32 graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let topic = "/tbs/producer/out";

    // Exhaust the subscriber slots (1 in-graph body sub + HEADROOM openers).
    let mut held = Vec::new();
    for i in 0..INTROSPECTION_SUBSCRIBER_HEADROOM {
        held.push(
            mgr.create_subscriber(topic)
                .unwrap_or_else(|e| panic!("headroom opener {i}: {e}")),
        );
    }

    // (a) The topic EXISTS but a further open-only subscriber fails with the
    //     ACTIONABLE slot-exhaustion hint — the error ensure_topic_available must
    //     SURFACE, never mask with the remote not-found.
    let open_err = match mgr.create_subscriber_open_only(topic) {
        Ok(_) => panic!("slots exhausted — open-only must fail"),
        Err(e) => e.to_string(),
    };
    assert!(
        open_err.contains("ExceedsMaxSupportedSubscribers"),
        "the surfaced error names slot exhaustion: {open_err}"
    );
    // (b) data_service_missing is FALSE for the EXISTING (slot-exhausted) topic —
    //     opening the service FACTORY needs no subscriber slot — so
    //     ensure_topic_available SURFACES (a), never falls through.
    assert!(
        !mgr.data_service_missing(topic),
        "a slot-exhausted LIVE topic is PRESENT (its data service opens) — surface the \
         actionable error, do NOT fall through to remote"
    );
    // (c) data_service_missing is TRUE for a never-created topic — the stale-
    //     mirror / topic-gone class that DOES fall through to the remote rung.
    assert!(
        mgr.data_service_missing("/tbs/producer/never_created"),
        "a genuinely-absent topic reads as missing (falls through to the remote rung)"
    );
    // Keep the slot-holders alive across the assertions above.
    let _ = &held;
}

/// Producer for the data-trigger counting pin.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct TriggerProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl TriggerProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Data-triggered consumer — a MACRO `drop_oldest` `#[input(trigger)]` node,
/// so post-unification it attaches ONE body subscriber plus
/// a STANDALONE listener-only `Listener` — NOT a second subscriber.
#[cerulion_node]
#[derive(Default)]
struct TriggeredConsumer {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl TriggeredConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Plain second consumer on the same topic — pins the PER-CONSUMER term
/// of the provisioning formula (a `consumers.len() → 1` regression is
/// invisible on single-consumer graphs).
#[cerulion_node(period_ms = 50)]
#[derive(Default)]
struct PlainConsumer {
    #[input(backpressure = drop_oldest)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl PlainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Data-triggered consumer whose trigger input is `block` — INELIGIBLE to
/// unify (`block` keeps the dual subscriber because of its
/// producer-pacing subtleties). It is a MACRO node, so `block`, not its
/// macro-ness, is what disqualifies it — broadening eligibility-predicate
/// coverage beyond the `sample(N)` arm the existing WaitSet tests use. It
/// therefore attaches TWO subscribers (body + timestamp drain) and NO
/// standalone listener.
#[cerulion_node]
#[derive(Default)]
struct BlockTriggeredConsumer {
    #[input(trigger, backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl BlockTriggeredConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

#[test]
fn trigger_drain_counts_toward_subscriber_provisioning() {
    // `triggered_consumer` is a MACRO `drop_oldest`
    // data-trigger node, so it is ELIGIBLE to UNIFY onto its body subscriber
    // — it contributes ONE subscriber (body only), NOT the
    // legacy TWO (body + timestamp drain). Its WaitSet wake source is a
    // STANDALONE listener-only `Listener`, provisioned via the
    // topic's `extra_event_listeners` event-port budget, NOT a second
    // subscriber slot. With the plain second consumer's body subscriber the
    // topic is provisioned at 1 (triggered body) + 1 (plain body) +
    // INTROSPECTION_SUBSCRIBER_HEADROOM slots: an opener requiring exactly
    // that attaches, one requiring one MORE is rejected. (Every count here is
    // written against the constant, which was raised 4 → 5 for the standing
    // liveness observer.)
    //
    // (With TWO subscribers per triggered consumer the
    // topic would provision 2 + 1 + HEADROOM and both probes would sit one higher.
    // Right-sizing the unified count to +1 dropped both numbers by one — the
    // SHM win.)
    //
    // (Mutation oracle, still pins the per-consumer term: dropping the
    // trigger consumer's body subscriber count provisions 5 — require-6
    // fails; `consumers.len() → 1` provisions 5 — require-6 fails;
    // double-counting consumers provisions 8 — reject-7 fails.)
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trigger_count".to_string(),
        prefix: "tbst".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "trigger_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "triggered_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "plain".to_string(),
                node_type: "plain_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(TriggerProducerEntry::new()),
    );
    factories.insert(
        "consumer".to_string(),
        Box::new(TriggeredConsumerEntry::new()),
    );
    factories.insert("plain".to_string(), Box::new(PlainConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build trigger graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let topic = "/tbst/producer/out";
    if let Err(e) = mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            2,
            PublisherProvisioning::SingleWriter,
            // `history_size` (native iceoryx2 history); 0 here.
            0,
            // A raw opener carries no standalone listeners.
            0,
        ),
        1,
    ) {
        panic!(
            "an opener requiring the provisioned {} slots (2 bodies + \
             {INTROSPECTION_SUBSCRIBER_HEADROOM} headroom; the unified trigger \
             contributes only its body) must attach: {e}",
            2 + INTROSPECTION_SUBSCRIBER_HEADROOM
        );
    }
    match mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            3,
            PublisherProvisioning::SingleWriter,
            // `history_size` (native iceoryx2 history); 0 here.
            0,
            // A raw opener carries no standalone listeners.
            0,
        ),
        1,
    ) {
        Ok(_) => panic!(
            "an opener requiring {} slots must NOT attach (provisioned {})",
            3 + INTROSPECTION_SUBSCRIBER_HEADROOM,
            2 + INTROSPECTION_SUBSCRIBER_HEADROOM
        ),
        Err(e) => assert!(
            e.to_string().contains(&format!(
                "max_subscribers {}",
                3 + INTROSPECTION_SUBSCRIBER_HEADROOM
            )),
            "the rejection must carry the requested slot count: {e}"
        ),
    }

    // The EVENT-service listener cap must cover the
    // standalone listener-only `Listener` the unified data-trigger input
    // created — pins the `extra_event_listeners` budget. The
    // topic provisions (2 + HEADROOM) data subscribers + 1 publisher
    // (SingleWriter) + 1 extra listener (the one unified input) event listener
    // slots. A raw opener requiring that many must attach (under the old
    // budget, which omitted the extra term, the live event service carried one
    // fewer listener and this opener would be rejected — the mutation oracle
    // for the budget).
    match mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            2,
            PublisherProvisioning::SingleWriter,
            0,
            // Require the extra standalone-listener slot the graph provisioned.
            1,
        ),
        1,
    ) {
        Ok(_) => {}
        Err(e) => panic!(
            "an opener requiring the standalone-listener event slot \
             (extra_event_listeners = 1) must attach — the graph provisioned \
             it for the unified data-trigger input: {e}"
        ),
    }
}

#[test]
fn mixed_eligibility_topic_provisions_both_subscriber_and_listener_budgets() {
    // A single trigger topic with a
    // mixed population of data-trigger consumers — one unified (eligible) and
    // one ineligible — must provision both the right number of data subscriber
    // slots and the standalone listener (`extra_event_listeners`) slots, and
    // build successfully at the tightened count.
    //
    // The unit-level `for_topology` tests prove the two budgets thread through
    // the constructor; this is the END-TO-END proof that a real
    // `build_for_test` graph threads them correctly when BOTH eligibility
    // branches of the `NodeInfo::unifies_data_trigger` predicate fire on the
    // SAME topic.
    //
    // - `unified`   : `#[input(trigger)]` (default `drop_oldest`) MACRO node →
    //                 ELIGIBLE → +1 BODY subscriber, +1 `extra_event_listeners`
    //                 (its WaitSet wake source is a standalone listener-only
    //                 `Listener`, NOT a second subscriber). No drain subscriber.
    // - `blocked`   : `#[input(trigger, backpressure = block)]` MACRO node →
    //                 INELIGIBLE (`block` keeps the dual subscriber) → +2
    //                 subscribers (body + timestamp drain), NO standalone
    //                 listener. (Mixed topic ⇒ the `block` degrades to
    //                 `drop_oldest` at runtime with a warn; the build still
    //                 succeeds and the eligibility predicate keys off the
    //                 DECLARED `Block`, so the dual-subscriber count holds.)
    //
    // Topic `/mxt/producer/out` therefore provisions:
    //   data subscribers  = 2 consumer edges + 1 block drain
    //                       + INTROSPECTION_SUBSCRIBER_HEADROOM
    //   event listeners   = those subscribers + 1 publisher (SingleWriter)
    //                       + 1 extra (the unified input)
    //
    // Oracle, not `build.is_ok()`: we BUILD (a smoke test that the unified
    // input's standalone `Listener` attaches and the mixed-eligibility wiring
    // holds — note the introspection headroom alone would absorb the lone
    // standalone listener, so the BUILD does NOT by itself pin the budget),
    // THEN probe the EXACT provisioning — which IS the budget oracle:
    // require-(3 + HEADROOM) data subscribers attaches / require one MORE
    // rejected; require the extra-listener slot (extra_event_listeners = 1)
    // attaches / require ONE MORE (= 2) rejected. The require-one-more arm is
    // what pins the listener budget is EXACT, not merely "enough".
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "mixed_eligibility".to_string(),
        prefix: "mxt".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "trigger_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "unified".to_string(),
                node_type: "triggered_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "blocked".to_string(),
                node_type: "block_triggered_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(TriggerProducerEntry::new()),
    );
    factories.insert(
        "unified".to_string(),
        Box::new(TriggeredConsumerEntry::new()),
    );
    factories.insert(
        "blocked".to_string(),
        Box::new(BlockTriggeredConsumerEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("mixed-eligibility graph must build — the unified input's standalone listener attaches and both eligibility branches provision (the exact budget is pinned by the probes below, not by this build)");
    let mgr = runtime.test_transport().expect("test transport parked");
    let topic = "/mxt/producer/out";

    // Data-subscriber budget = in-graph 3 (2 bodies + 1 block drain) +
    // INTROSPECTION_SUBSCRIBER_HEADROOM (added by the `for_topology`
    // constructor). An opener requiring the same in-graph 3 attaches; one
    // requiring in-graph 4 is rejected. (Mutation oracle: if the unified
    // consumer were mis-counted as a drain too, in-graph would be 4 and the
    // require-one-more arm would WRONGLY attach.)
    if let Err(e) = mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            3,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        ),
        1,
    ) {
        panic!(
            "an opener requiring the provisioned {} data subscriber slots \
             (unified body + block body + block drain + \
             {INTROSPECTION_SUBSCRIBER_HEADROOM} headroom) must attach: {e}",
            3 + INTROSPECTION_SUBSCRIBER_HEADROOM
        );
    }
    match mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            4,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        ),
        1,
    ) {
        Ok(_) => panic!(
            "an opener requiring {} data subscriber slots must NOT attach (provisioned {})",
            4 + INTROSPECTION_SUBSCRIBER_HEADROOM,
            3 + INTROSPECTION_SUBSCRIBER_HEADROOM
        ),
        Err(e) => assert!(
            e.to_string().contains(&format!(
                "max_subscribers {}",
                4 + INTROSPECTION_SUBSCRIBER_HEADROOM
            )),
            "the rejection must carry the requested slot count: {e}"
        ),
    }

    // Event-listener budget = 9 (7 subscribers + 1 publisher + 1 extra). An
    // opener requiring the ONE standalone-listener slot the unified input
    // provisioned attaches; requiring ONE MORE than that is rejected — pins the
    // listener budget is EXACT, not just "enough". (Mutation oracle: dropping
    // the `extra_event_listeners` thread provisions only 8 event ports, so the
    // require-1 opener's 9th listener is rejected.)
    if let Err(e) = mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            2,
            PublisherProvisioning::SingleWriter,
            0,
            // Require the one standalone-listener slot the unified input added.
            1,
        ),
        1,
    ) {
        panic!(
            "an opener requiring the standalone-listener event slot \
             (extra_event_listeners = 1 → 9 event ports) must attach — the \
             graph provisioned it for the UNIFIED data-trigger input on this \
             mixed topic: {e}"
        );
    }
    match mgr.create_subscriber_with_buffers(
        topic,
        cerulion_core::transport::TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            16,
            2,
            PublisherProvisioning::SingleWriter,
            0,
            // ONE MORE listener than provisioned (→ one event port over budget).
            2,
        ),
        1,
    ) {
        Ok(_) => panic!(
            "an opener requiring TWO extra event listeners ({} ports) must NOT \
             attach — the mixed topic provisioned exactly ONE ({} ports), for \
             the single unified input",
            6 + INTROSPECTION_SUBSCRIBER_HEADROOM,
            5 + INTROSPECTION_SUBSCRIBER_HEADROOM
        ),
        Err(e) => assert!(
            e.to_string().contains(&format!(
                "max_listeners/max_notifiers {}",
                6 + INTROSPECTION_SUBSCRIBER_HEADROOM
            )),
            "the rejection must carry the over-budget armed event requirement \
             (provisioned {}, this opener needed {}): {e}",
            5 + INTROSPECTION_SUBSCRIBER_HEADROOM,
            6 + INTROSPECTION_SUBSCRIBER_HEADROOM
        ),
    }
}

#[test]
fn event_service_caps_track_subscriber_provisioning() {
    // Pins the event-cap behavior: the event service's listener/notifier caps
    // must track the per-topic max_subscribers + the publisher term
    // (the live service's publisher count — here the
    // create-default 2, since the creator left it unset) — not 16/16
    // defaults. Provision 20 data slots directly (decoupled from in-graph
    // counts so data slots can't exhaust first), then attach 16 default
    // openers: 16 subscribers + 1 publisher = 17 listeners, which the
    // 16/16 defaults would reject at the 16th subscriber's listener — the
    // provisioned 22 (20 + 2) admits them all.
    // (Mutation oracle: deleting the event-cap block fails subscriber 15
    // with ExceedsMaxSupportedListeners; verified empirically.)
    let config = cerulion_core::transport::TransportConfig {
        node_name: "event_caps_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(20);
    let _pubr = mgr
        .create_publisher_with_topic_config(
            "tbs_event_caps/out",
            MaxSliceLen::const_new(256),
            0,
            cfg,
        )
        .expect("publisher on a 20-slot topic");
    let mut held = Vec::new();
    for i in 0..16 {
        match mgr.create_subscriber("tbs_event_caps/out") {
            Ok(sub) => held.push(sub),
            Err(e) => panic!(
                "subscriber {i} of 16 must attach (20 data slots, 22 event \
                 ports provisioned; iceoryx2's 16/16 event defaults would \
                 reject the 17th listener here): {e}"
            ),
        }
    }
}

#[test]
fn max_subscribers_boundary_4096_accepted_by_choke_point() {
    // Confirms the choke-point boundary uses `>` rather than `>=`, without
    // provisioning 4096 slots: against a pre-existing default service
    // (8 slots), an opener requiring Some(4096) must pass the choke-point
    // check (the bound is inclusive — "<= 4096") and fail at iceoryx2's
    // OPEN verification (4096 > provisioned 8) with the slot count + hint.
    // With a `>=` boundary the choke point would reject first with the
    // self-contradictory message "must be <= 4096" while rejecting 4096.
    let tt = TestTransport::with_buffer_size(8);
    let _pubr = tt.publisher("tbs_4096/out", MaxSliceLen::const_new(256), 0);
    let mut cfg = tt.default_topic_config();
    cfg.max_subscribers = Some(4096);
    let msg = match tt.subscriber_with_buffers("tbs_4096/out", cfg, 1) {
        Ok(_) => panic!("requiring 4096 slots on an 8-slot service must fail at OPEN"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_subscribers 4096") && msg.contains("fewer subscriber slots"),
        "the failure must come from iceoryx2's open verification (carrying \
         the requested count + the slot hint), not the choke-point bound: {msg}"
    );
}

#[test]
fn single_writer_enforced_at_service_level() {
    // A graph topic is provisioned max_publishers = 1 — the
    // graph's own producer holds the only slot, so a rogue second
    // publisher (default path — no open requirement, but port creation
    // hits the cap) fails LOUDLY with the single-writer explanation.
    // Topology already rejects double-producers at build; this pins that
    // iceoryx2 itself enforces it against out-of-graph writers.
    // (Mutation oracle: dropping the Some(1) provisioning leaves the
    // iceoryx2 default of 2 — the rogue publisher attaches and the
    // rejection arm fails.)
    let (config, factories) = depth32_graph();
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build depth32 graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let msg = match mgr.create_publisher("/tbs/producer/out", MaxSliceLen::const_new(256), 0) {
        Ok(_) => panic!("a second publisher must NOT attach to a single-writer graph topic"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("ExceedsMaxSupportedPublishers")
            && msg.contains("a publisher already holds the topic's only slot")
            && msg.contains("single-writer")
            && msg.contains("multi_publisher_topics"),
        "the rejection must name the cause, the held slot, and the \
         single-writer contract: {msg}"
    );

    // Exact provisioning, both directions: an opener requiring the
    // provisioned 1 attaches; one requiring 2 is rejected by iceoryx2's
    // open verification with the publisher-slot hint.
    let mut req_one = mgr.default_topic_config();
    req_one.max_publishers = Some(1);
    if let Err(e) = mgr.create_subscriber_with_buffers("/tbs/producer/out", req_one, 1) {
        panic!("an opener requiring the provisioned 1 publisher slot must attach: {e}");
    }
    let mut req_two = mgr.default_topic_config();
    req_two.max_publishers = Some(2);
    let msg = match mgr.create_subscriber_with_buffers("/tbs/producer/out", req_two, 1) {
        Ok(_) => panic!("an opener requiring 2 publisher slots must NOT attach (provisioned 1)"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_publishers 2")
            && msg.contains("fewer publisher slots than this opener requires")
            && msg.contains("single-writer graph topic"),
        "the rejection must carry the requested count and the corrected \
         direction narrative (the failing opener requires MORE than the \
         single-writer service provides): {msg}"
    );
}

#[test]
fn max_publishers_guards_at_choke_point() {
    // The max_publishers twins of the zero/huge guard class.
    let tt = TestTransport::with_buffer_size(8);
    let mut zero_pubs = tt.default_topic_config();
    zero_pubs.max_publishers = Some(0);
    let msg = match tt.subscriber_with_buffers("tbs_zero_pubs/out", zero_pubs, 1) {
        Ok(_) => panic!("max_publishers Some(0) must be rejected at the choke point"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_publishers must be >= 1 when set"),
        "the rejection must state the bound: {msg}"
    );
    let mut huge_pubs = tt.default_topic_config();
    huge_pubs.max_publishers = Some(4097);
    let msg = match tt.subscriber_with_buffers("tbs_huge_pubs/out", huge_pubs, 1) {
        Ok(_) => panic!("max_publishers above the sanity bound must be rejected"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_publishers must be <= 4096"),
        "the rejection must state the bound: {msg}"
    );
    // Publisher-path symmetry (the subscriber-path
    // precedent: the publisher entry point has no subscriber pre-checks
    // in front of the choke point, so exercise the guard through it too).
    let config = cerulion_core::transport::TransportConfig {
        node_name: "pub_choke_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init");
    let mut zero_pubs = mgr.default_topic_config();
    zero_pubs.max_publishers = Some(0);
    let msg = match mgr.create_publisher_with_topic_config(
        "tbs_zero_pubs_pubpath/out",
        MaxSliceLen::const_new(256),
        0,
        zero_pubs,
    ) {
        Ok(_) => panic!("max_publishers Some(0) must be rejected on the publisher path too"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_publishers must be >= 1 when set"),
        "the rejection must state the bound: {msg}"
    );
}

#[test]
fn publisher_exhaustion_generic_arm_on_multi_slot_topic() {
    // The port-create exhaustion hint branches on
    // the live provisioned count — the single-writer story is only told
    // for single-writer services. On a default topic (iceoryx2
    // create-default: 2 publisher slots, no graph provisioning) the third
    // publisher must get the generic all-N-slots message and NOT the
    // single-writer claim. (Pins the count→literal-1 comparison and the
    // branch condition on the hint.)
    let config = cerulion_core::transport::TransportConfig {
        node_name: "pub_generic_arm_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init");
    let _pubr1 = mgr
        .create_publisher("tbs_generic/out", MaxSliceLen::const_new(256), 0)
        .expect("first publisher must attach (2 default slots)");
    let _pubr2 = mgr
        .create_publisher("tbs_generic/out", MaxSliceLen::const_new(256), 0)
        .expect("second publisher must attach (2 default slots)");
    let msg = match mgr.create_publisher("tbs_generic/out", MaxSliceLen::const_new(256), 0) {
        Ok(_) => panic!("a third publisher must NOT attach to a 2-slot default topic"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("ExceedsMaxSupportedPublishers")
            && msg.contains("all 2 of the topic's publisher slots are attached"),
        "the generic arm must carry the live slot count: {msg}"
    );
    assert!(
        !msg.contains("single-writer"),
        "the single-writer story must NOT be told for a multi-slot topic: {msg}"
    );
}

#[test]
fn event_caps_track_publisher_term_on_producer_less_provisioning() {
    // The event-service listener/notifier caps must track the
    // topic's publisher term too — the live data service's publisher
    // count (here the create-default 2, since the creator leaves the
    // setter uncalled) — not a hardcoded +1 for "the single producer".
    // The data service admits 2 publishers, each attaching a listener +
    // a notifier, so the caps must budget subscribers + 2. (Mutation oracle: reverting the
    // publisher term to the old +1 makes the SECOND publisher below die
    // at event-port creation — verified empirically.)
    let config = cerulion_core::transport::TransportConfig {
        node_name: "event_pub_term_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(2);
    // max_publishers stays None: data service gets iceoryx2's
    // create-default 2 publisher slots; event caps must be 2 + 2.
    let _pubr1 = mgr
        .create_publisher_with_topic_config(
            "tbs_event_pub_term/out",
            MaxSliceLen::const_new(256),
            0,
            cfg,
        )
        .expect("first publisher creates the service (2 data slots, 4 event ports)");
    // Fill the subscriber side completely: 2 data slots = 2 listeners.
    let _sub1 = mgr
        .create_subscriber("tbs_event_pub_term/out")
        .expect("subscriber 1 of 2");
    let _sub2 = mgr
        .create_subscriber("tbs_event_pub_term/out")
        .expect("subscriber 2 of 2");
    // The SECOND publisher needs event port #4 — exactly the slot the
    // hardcoded +1 formula would not have provisioned (caps 3).
    let _pubr2 = mgr
        .create_publisher("tbs_event_pub_term/out", MaxSliceLen::const_new(256), 0)
        .expect(
            "second publisher must attach: the event caps budget the \
             publisher term (2 subscribers + 2 publishers = 4 ports)",
        );
}

#[test]
fn max_publishers_boundary_4096_accepted_by_choke_point() {
    // Confirms the boundary uses `>` rather than `>=` on the
    // publisher guard (the twin of the subscriber test): against
    // a pre-existing default service (2 publisher slots), an opener
    // requiring Some(4096) must pass the choke point (the bound is
    // inclusive — "<= 4096") and fail at iceoryx2's OPEN verification
    // (4096 > provisioned 2) with the requested count + the
    // publisher-slot hint. With a `>=` boundary the choke point would reject
    // first with the self-contradictory "must be <= 4096" while rejecting
    // 4096.
    let tt = TestTransport::with_buffer_size(8);
    let _pubr = tt.publisher("tbs_pub4096/out", MaxSliceLen::const_new(256), 0);
    let mut cfg = tt.default_topic_config();
    cfg.max_publishers = Some(4096);
    let msg = match tt.subscriber_with_buffers("tbs_pub4096/out", cfg, 1) {
        Ok(_) => panic!("requiring 4096 publisher slots on a 2-slot service must fail at OPEN"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_publishers 4096")
            && msg.contains("fewer publisher slots than this opener requires"),
        "the failure must come from iceoryx2's open verification (carrying \
         the requested count + the publisher-slot hint), not the choke-point \
         bound: {msg}"
    );
}

#[tracing_test::traced_test]
#[test]
fn degraded_single_writer_open_warns() {
    // Loud-over-silent: iceoryx2's open
    // verification is at-least, so a graph opener requiring Some(1)
    // silently attaches to a pre-existing default-created service (2
    // slots) — the single-writer contract is void there (a rogue can take
    // the second slot) and only this warn says so. The require-2 arm pins
    // the boundary: live == required must NOT warn. (Mutation oracle:
    // deleting the warn → 0 lines; `>` loosened to `>=` → the require-2
    // open warns too → 2 lines. Both die on the exactly-1 count.)
    let tt = TestTransport::with_buffer_size(8);
    let _pubr = tt.publisher("tbs_degraded/out", MaxSliceLen::const_new(256), 0);
    let mut cfg = tt.default_topic_config();
    cfg.max_publishers = Some(1);
    tt.subscriber_with_buffers("tbs_degraded/out", cfg, 1)
        .expect("require-1 opener attaches to the 2-slot service (at-least semantics)");
    // Dedup pin: a second degraded open of the same topic must
    // not warn again — open_topic_services runs once per producer/body/
    // drain open of a graph topic and the warn is once per (topic, kind)
    // per manager. Deleting the dedup turns the exactly-1 count into 2.
    let mut cfg_again = tt.default_topic_config();
    cfg_again.max_publishers = Some(1);
    tt.subscriber_with_buffers("tbs_degraded/out", cfg_again, 1)
        .expect("second require-1 opener attaches");
    let mut cfg_eq = tt.default_topic_config();
    cfg_eq.max_publishers = Some(2);
    tt.subscriber_with_buffers("tbs_degraded/out", cfg_eq, 1)
        .expect("require-2 opener attaches exactly (live == required)");
    // The subscriber twin — the default-created service has 8
    // subscriber slots; a require-5 opener attaches silently but the
    // exactly-(in-graph + headroom) bound is void there. Same
    // exactly-once dedup contract.
    let mut cfg_subs = tt.default_topic_config();
    cfg_subs.max_subscribers = Some(5);
    tt.subscriber_with_buffers("tbs_degraded/out", cfg_subs, 1)
        .expect("require-5 subscriber opener attaches to the 8-slot service");
    logs_assert(|lines: &[&str]| {
        // Each degraded warn matched WITH its level token AND against the
        // level-free total of that marker: a warn demoted to INFO/ERROR is not
        // the one warn of its kind, and neither is a second copy of it emitted
        // at another level.
        let pubs = count_at_exclusively(lines, "WARN", &["publisher provisioning degraded"])?;
        let subs = count_at_exclusively(lines, "WARN", &["subscriber provisioning degraded"])?;
        if pubs == 1 && subs == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 degraded warn per kind, got pubs={pubs} subs={subs}"
            ))
        }
    });
}

#[test]
fn event_caps_cover_every_data_admitted_attacher_on_preexisting_service() {
    // Pins the live-subs term, not the requested term, in the armed event
    // formula: a foreign data service pre-exists with 8 default
    // subscriber slots and no event service. An armed opener requiring
    // only Some(5) creates the event service — its caps must come from
    // the live terms (8 + 2 = 10), not the requested (5 + 2 = 7), or
    // data-admitted subscribers #8.. die at event-port create blamed on
    // a rogue that doesn't exist.
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "tbs_live_subs/out/data".try_into().expect("service name");
    let _foreign = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(4)
        .create()
        .expect("foreign data service at iceoryx2 defaults (8 subscriber slots)");
    let config = cerulion_core::transport::TransportConfig {
        node_name: "live_subs_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(config, ix_config).expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(5);
    let _armed = mgr
        .create_subscriber_with_buffers("tbs_live_subs/out", cfg, 1)
        .expect("armed opener attaches and creates the event service");
    let mut subs = Vec::new();
    for i in 2..=8 {
        subs.push(
            mgr.create_subscriber("tbs_live_subs/out")
                .unwrap_or_else(|e| {
                    panic!(
                        "subscriber {i} of 8 must attach (8 subs + 0 pubs = 8 <= 10 \
                     event ports from LIVE terms): {e}"
                    )
                }),
        );
    }
}

#[test]
fn requirement_only_opener_passes_event_caps_on_single_writer_topic() {
    // Regression pin for the live-truth event
    // formula: an opener asserting the topic's full subscriber capacity
    // (Some(5)) with no publisher requirement must attach to a
    // single-writer graph topic. Under the superseded requested-intent
    // formula its event requirement was 5 + ITS OWN create-default 2 = 7
    // > the provisioned 5 + 1 = 6 — a lockout for tooling asserting
    // subscriber capacity. With live terms (5 + live 1 = 6 ≤ 6) an armed
    // requirement can never exceed caps provisioned by the same formula.
    let (config, factories) = depth32_graph();
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build depth32 graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut cfg = mgr.default_topic_config();
    // The provisioned value (1 in-graph + the introspection headroom).
    cfg.max_subscribers = Some(1 + INTROSPECTION_SUBSCRIBER_HEADROOM);
    if let Err(e) = mgr.create_subscriber_with_buffers("/tbs/producer/out", cfg, 1) {
        panic!(
            "a subscriber-capacity-asserting opener must attach (the event \
             requirement must use the LIVE publisher term, not this opener's \
             own default): {e}"
        );
    }
}

#[test]
fn event_listener_exhaustion_names_cause_count_remedy() {
    // Pins event_port_exhaustion_hint end-to-end
    // (all four arms): a topic provisioned
    // 1 subscriber + 1 publisher carries 2 event listener slots; the
    // Cerulion publisher holds #1 and a RAW iceoryx2 listener (the hint's
    // own narrated rogue) takes #2, so the subscriber's listener create
    // dies — with the cap, the attach math, and the raw-attacher remedy
    // named. (Also pins the cap-source and kind-string values in the
    // helper, and a multiplication-shaped cap regression: 1×1 = 1 would
    // reject the raw listener at #2.)
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let config = cerulion_core::transport::TransportConfig {
        node_name: "raw_listener_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(config, ix_config.clone())
        .expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(1);
    cfg.max_publishers = Some(1);
    let _pubr = mgr
        .create_publisher_with_topic_config(
            "tbs_raw_listener/out",
            MaxSliceLen::const_new(256),
            0,
            cfg,
        )
        .expect("publisher creates the 1+1 topic (2 event ports)");
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let event_name: iceoryx2::service::service_name::ServiceName = "tbs_raw_listener/out/event"
        .try_into()
        .expect("service name");
    let event_svc = raw_node
        .service_builder(&event_name)
        .event()
        .open()
        .expect("raw open of the graph-created event service");
    let _raw_listener = event_svc
        .listener_builder()
        .create()
        .expect("raw listener takes event port 2 of 2");
    let msg = match mgr.create_subscriber("tbs_raw_listener/out") {
        Ok(_) => panic!("the subscriber's listener must NOT fit (2/2 event ports held)"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("all 2 of the topic's event listener slots")
            && msg.contains("raw event-service user"),
        "the rejection must name the cap, the attach math, and the \
         raw-attacher remedy: {msg}"
    );
}

#[test]
fn foreign_typed_service_rejected_with_type_skew_hint() {
    // Pins the IncompatibleTypes environment arm —
    // the one deterministically reachable arm — and the formatting
    // split (environment failures carry no requirements parenthetical): a
    // raw iceoryx2 service squatting on the topic's data name with a
    // different payload type must fail the open with the type-skew story.
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "tbs_type_skew/out/data".try_into().expect("service name");
    let _foreign = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<u64>()
        .create()
        .expect("foreign-typed data service");
    let config = cerulion_core::transport::TransportConfig {
        node_name: "type_skew_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(config, ix_config).expect("init");
    let msg = match mgr.create_subscriber("tbs_type_skew/out") {
        Ok(_) => panic!("opening a foreign-typed service must fail"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("different payload type"),
        "the type-skew arm must fire: {msg}"
    );
    assert!(
        !msg.contains("this opener requires"),
        "environment failures must NOT carry the requirements parenthetical: {msg}"
    );
}

#[test]
fn event_caps_armed_by_publisher_only_provisioning() {
    // A hand-mutated publisher-only config
    // (max_subscribers None + max_publishers Some(20)) must arm the event
    // caps too — under a subscribers-only gate the event service would
    // keep iceoryx2's 16/16 defaults while the data service admits 20
    // publishers, and publisher #17's event ports would fail far from the
    // mutation site. live terms: caps = live subs (default 8) + live pubs
    // (20) = 28. (Mutation oracle: reverting the arming gate to
    // max_subscribers-only → publisher #17 dies at event-port create.)
    let config = cerulion_core::transport::TransportConfig {
        node_name: "pub_only_caps_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(
        config,
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_publishers = Some(20);
    let mut held = Vec::new();
    held.push(
        mgr.create_publisher_with_topic_config(
            "tbs_pub_only/out",
            MaxSliceLen::const_new(256),
            0,
            cfg,
        )
        .expect("publisher 1 of 17 creates the topic (28 event ports)"),
    );
    for i in 2..=17 {
        held.push(
            mgr.create_publisher("tbs_pub_only/out", MaxSliceLen::const_new(256), 0)
                .unwrap_or_else(|e| {
                    panic!(
                        "publisher {i} of 17 must attach (20 data slots, 28 event \
                         ports — the 16/16 defaults would reject #17): {e}"
                    )
                }),
        );
    }
    // Pins the live-subs term rather than a hardcoded literal: the data
    // service admits 8 default subscribers; with caps 28 all 6 attach
    // (17 pubs + 6 subs = 23 ≤ 28); under a literal-2 subs term (caps
    // 22) subscriber #6 needs event port 23 and dies.
    let mut subs = Vec::new();
    for i in 1..=6 {
        subs.push(
            mgr.create_subscriber("tbs_pub_only/out")
                .unwrap_or_else(|e| {
                    panic!("subscriber {i} of 6 must attach (23 <= 28 event ports): {e}")
                }),
        );
    }
}

#[test]
fn foreign_tight_event_service_rejected_with_caps_and_ordering_hint() {
    // Pins the event-open error mapper (previously zero
    // coverage on the caps parenthetical and the ordering hint): a raw
    // event service created with 1/1 caps before any Cerulion open; an
    // armed Cerulion opener then creates the data service fresh (live: 1
    // sub + 2 pubs) and its event open requires 3 > 1 — rejected with
    // the requirement and the raw-creator story.
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let event_name: iceoryx2::service::service_name::ServiceName = "tbs_tight_event/out/event"
        .try_into()
        .expect("service name");
    let _foreign = raw_node
        .service_builder(&event_name)
        .event()
        .max_listeners(1)
        .max_notifiers(1)
        .create()
        .expect("tight foreign event service");
    let config = cerulion_core::transport::TransportConfig {
        node_name: "tight_event_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(config, ix_config).expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(1);
    let msg = match mgr.create_subscriber_with_buffers("tbs_tight_event/out", cfg, 1) {
        Ok(_) => panic!("an armed event open requiring 3 ports on a 1/1 service must fail"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("max_listeners/max_notifiers 3"),
        "the failure must carry the armed event requirement: {msg}"
    );
    assert!(
        msg.contains("fewer event") && msg.contains("raw event-service user"),
        "the failure must carry the ordering hint naming the raw creator: {msg}"
    );
}

#[test]
fn event_port_exhaustion_hints_cover_all_attach_sites() {
    // event_listener_exhaustion_names_cause_count_remedy pins the
    // shared helper body via the subscriber-path listener site; these
    // three arms pin the remaining call sites (each site's matches!
    // condition + kind/cap arguments are pinned independently) —
    // subscriber-path notifier, publisher-path notifier, publisher-path
    // listener.
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let config = cerulion_core::transport::TransportConfig {
        node_name: "event_arms_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(config, ix_config.clone())
        .expect("init");
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let one_one = |mgr: &cerulion_core::transport::TransportManager| {
        let mut cfg = mgr.default_topic_config();
        cfg.max_subscribers = Some(1);
        cfg.max_publishers = Some(1);
        cfg
    };
    let raw_event = |name: &str| {
        let event_name: iceoryx2::service::service_name::ServiceName =
            name.try_into().expect("service name");
        raw_node
            .service_builder(&event_name)
            .event()
            .open()
            .expect("raw open of the Cerulion-created event service")
    };

    // (a) subscriber-path NOTIFIER: the Cerulion publisher holds notifier
    // #1 (and listener #1) on a 1+1 topic (caps 2/2); a raw notifier
    // takes #2; the subscriber's listener (#2) fits but its notifier (#3)
    // dies at the sub-notifier arm.
    let _pub_a = mgr
        .create_publisher_with_topic_config(
            "tbs_arm_a/out",
            MaxSliceLen::const_new(256),
            0,
            one_one(&mgr),
        )
        .expect("publisher creates the 1+1 topic");
    let ev_a = raw_event("tbs_arm_a/out/event");
    let _raw_notifier_a = ev_a
        .notifier_builder()
        .create()
        .expect("raw notifier takes event notifier 2 of 2");
    let msg = match mgr.create_subscriber("tbs_arm_a/out") {
        Ok(_) => panic!("the subscriber's notifier must NOT fit (2/2 held)"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("all 2 of the topic's event notifier slots"),
        "the subscriber-path notifier arm must fire: {msg}"
    );

    // (b) publisher-path NOTIFIER: a Cerulion SUBSCRIBER creates the 1+1
    // topic (listener #1 + notifier #1); raw notifier #2; the publisher's
    // notifier (#3, created before its listener) dies at the
    // pub-notifier arm.
    let _sub_b = mgr
        .create_subscriber_with_buffers("tbs_arm_b/out", one_one(&mgr), 1)
        .expect("subscriber creates the 1+1 topic");
    let ev_b = raw_event("tbs_arm_b/out/event");
    let _raw_notifier_b = ev_b
        .notifier_builder()
        .create()
        .expect("raw notifier takes event notifier 2 of 2");
    let msg = match mgr.create_publisher("tbs_arm_b/out", MaxSliceLen::const_new(256), 0) {
        Ok(_) => panic!("the publisher's notifier must NOT fit (2/2 held)"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("all 2 of the topic's event notifier slots"),
        "the publisher-path notifier arm must fire: {msg}"
    );

    // (c) publisher-path LISTENER: subscriber creates the 1+1 topic
    // (listener #1); raw LISTENER #2; the publisher's notifier (#2) fits
    // but its listener (#3) dies at the pub-listener arm.
    let _sub_c = mgr
        .create_subscriber_with_buffers("tbs_arm_c/out", one_one(&mgr), 1)
        .expect("subscriber creates the 1+1 topic");
    let ev_c = raw_event("tbs_arm_c/out/event");
    let _raw_listener_c = ev_c
        .listener_builder()
        .create()
        .expect("raw listener takes event listener 2 of 2");
    let msg = match mgr.create_publisher("tbs_arm_c/out", MaxSliceLen::const_new(256), 0) {
        Ok(_) => panic!("the publisher's listener must NOT fit (2/2 held)"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("all 2 of the topic's event listener slots"),
        "the publisher-path listener arm must fire: {msg}"
    );
}

#[test]
fn trigger_listener_exhaustion_names_cause_count_remedy() {
    // Pins `create_trigger_listener`'s OWN
    // `matches!(e, ExceedsMaxSupportedListeners)` exhaustion-hint arm
    // (transport/mod.rs ~1415). This is an INDEPENDENT call site from the
    // shared `event_port_exhaustion_hint` BODY (pinned by
    // `event_listener_exhaustion_names_cause_count_remedy`) and the three
    // subscriber/publisher attach sites (pinned by
    // `event_port_exhaustion_hints_cover_all_attach_sites`) — the
    // standalone listener-only wake source (the unified data-trigger's
    // WaitSet listener) is the 4th, previously-unpinned `matches!` site.
    //
    // A topic provisioned 1 subscriber + 1 publisher carries 2 event
    // listener slots (1 + 1 + 0 extra). The Cerulion publisher holds
    // listener #1 and a RAW iceoryx2 listener takes #2, so the standalone
    // trigger listener (#3) dies — with the cap, the attach math, and the
    // raw-attacher remedy named. (Mutation oracle: changing the arm's
    // `ExceedsMaxSupportedListeners` to another variant — or `false` —
    // empties the hint and fails the substring asserts.)
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let config = cerulion_core::transport::TransportConfig {
        node_name: "trigger_listener_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr = cerulion_core::transport::TransportManager::init_for_test(config, ix_config.clone())
        .expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(1);
    cfg.max_publishers = Some(1);
    // The Cerulion publisher creates the owned 1+1 topic (2 event listener
    // slots) and holds listener #1 (and notifier #1).
    let _pubr = mgr
        .create_publisher_with_topic_config(
            "tbs_trigger_listener/out",
            MaxSliceLen::const_new(256),
            0,
            cfg,
        )
        .expect("publisher creates the 1+1 topic (2 event listener slots)");
    // A raw iceoryx2 listener takes listener slot #2 of 2.
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let event_name: iceoryx2::service::service_name::ServiceName = "tbs_trigger_listener/out/event"
        .try_into()
        .expect("service name");
    let event_svc = raw_node
        .service_builder(&event_name)
        .event()
        .open()
        .expect("raw open of the graph-created event service");
    let _raw_listener = event_svc
        .listener_builder()
        .create()
        .expect("raw listener takes event listener 2 of 2");
    // The standalone trigger listener needs listener #3 — it must NOT fit.
    let msg = match mgr.create_trigger_listener_for_test("tbs_trigger_listener/out", cfg) {
        Ok(_) => panic!("the standalone trigger listener must NOT fit (2/2 listener slots held)"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("all 2 of the topic's event listener slots")
            && msg.contains("raw event-service user"),
        "create_trigger_listener's exhaustion arm must name the cap, the \
         attach math, and the raw-attacher remedy: {msg}"
    );
}

#[tracing_test::traced_test]
#[test]
fn live_derived_event_budget_clamped_at_sanity_bound() {
    // A foreign data service with an unreasonable port count
    // (4200 subscribers) must not size Cerulion's event caps from garbage
    // — the live-derived sum is clamped to MAX_REASONABLE_PORTS with a
    // warn. The choke-point guards only cover THIS opener's requested
    // config; live-derived terms have no other bound (iceoryx2 applies no
    // upper clamp at create).
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix_config)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "tbs_huge_live/out/data".try_into().expect("service name");
    let _foreign = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(4)
        .max_subscribers(4200)
        .create()
        .expect("foreign data service with an unreasonable subscriber count");
    let config = cerulion_core::transport::TransportConfig {
        node_name: "huge_live_test".into(),
        clock: Arc::new(VirtualClock::new()),
        subscriber_buffer_size: 4,
        network: None,
    };
    let mgr =
        cerulion_core::transport::TransportManager::init_for_test(config, ix_config).expect("init");
    let mut cfg = mgr.default_topic_config();
    cfg.max_subscribers = Some(1);
    mgr.create_subscriber_with_buffers("tbs_huge_live/out", cfg, 1)
        .expect("the armed opener attaches (event caps clamped, not garbage-sized)");
    logs_assert(|lines: &[&str]| {
        // Matched WITH the level token AND against the level-free total: a
        // clamp warn demoted to INFO/ERROR would otherwise still count as the
        // one warn, and so would a second copy of it at another level.
        let n = count_at_exclusively(lines, "WARN", &["exceeds the sanity bound"])?;
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 clamp WARN, got {n}"))
        }
    });
}

#[test]
#[tracing_test::traced_test]
fn subscriber_success_log_fires_only_after_ports_exist() {
    // Review regression pin: the "subscriber created" debug
    // log lives at the end of `finish_subscriber` — a subscriber whose
    // port creation fails (slot exhaustion) must not emit it. Provision
    // max_subscribers Some(1): the first attach succeeds (exactly 1 log),
    // the second dies at port creation (0 further logs). (Mutation
    // oracle: moving the log back into the callers, before the fallible
    // tail, makes the count 2.)
    let tt = TestTransport::with_buffer_size(8);
    let mut cfg = tt.default_topic_config();
    cfg.max_subscribers = Some(1);
    let _s1 = tt
        .subscriber_with_buffers("tbs_log_pin/out", cfg, 1)
        .expect("first subscriber takes the only slot");
    let mut cfg2 = tt.default_topic_config();
    cfg2.max_subscribers = Some(1);
    if tt
        .subscriber_with_buffers("tbs_log_pin/out", cfg2, 1)
        .is_ok()
    {
        panic!("the second subscriber must die at port creation (1 slot)");
    }
    // Release-visible half of the ORDER claim: the
    // breadcrumb below is `debug!` and compiles out under
    // `release_max_level_info`, so in that profile the count proves
    // nothing about WHEN it fires. What every profile can see is the STATE
    // the breadcrumb reports: after one successful attach and one refused
    // one, exactly one subscriber port exists on the topic — a "created"
    // claim for the refused attach would be a lie against this count.
    // Limit: a log moved BEFORE the fallible tail is only
    // observable where `debug!` renders; the DEBUG-profile arm below is
    // that pin, this arm is the release floor.
    assert_eq!(
        tt.manager().topic_subscriber_count("tbs_log_pin/out"),
        1,
        "exactly one subscriber port must exist after one attach and one refusal"
    );
    logs_assert(|lines: &[&str]| {
        // Level-free twin: the subscriber-created breadcrumb must never be LOUD — the half of the
        // contract that survives `release_max_level_info`, where the gated
        // DEBUG count reads 0.
        for level in ["WARN", "INFO", "ERROR"] {
            let loud = lines
                .iter()
                .filter(|l| line_level(l) == Some(level) && (l.contains("subscriber created")))
                .count();
            if loud != 0 {
                return Err(format!(
                    "the subscriber-created breadcrumb was emitted at {level} ({loud} line(s))"
                ));
            }
        }
        // Counted AT DEBUG, not by text: the breadcrumb re-emitted at `trace!`
        // is still one line, and the loud sweep above permits TRACE. The same
        // call also pairs the level-free total, so a duplicate at another level
        // cannot read as the one breadcrumb.
        let n = count_at_exclusively(lines, "DEBUG", &["subscriber created", "tbs_log_pin/out"])?;
        // The success log is `debug!`: it exists only where `debug!` is compiled in.
        if n == debug_lines_expected(1) {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 success log for the topic, got {n}"
            ))
        }
    });
}

/// The create-leg borrow FLOOR degrade warn (the loaned-take provisioning
/// twin of `degraded_single_writer_open_warns`). An opener that REQUESTS a
/// create floor (`create_borrow_floor = Some(4)` — the rmw loanable-type
/// create, which is `External`-provisioned) and attaches to a PRE-EXISTING
/// service born below it must warn EXACTLY once per topic, naming the topic,
/// the requested floor and the live value — including on the `External`
/// arm, which the owned-topic borrow warn deliberately excludes. Controls:
/// a pre-existing service AT the floor does not warn, and a floored opener
/// that CREATES the service itself does not warn (nothing to degrade from).
///
/// Mutation oracle: wrapping the warn in `publisher_provisioning !=
/// External` (the owned-topic warn's shape) silences the External opener ⇒
/// the exactly-1 count fails at 0; deleting the dedup ⇒ 2.
#[tracing_test::traced_test]
#[test]
fn create_borrow_floor_degraded_open_warns_once_including_external() {
    let tt = TestTransport::with_buffer_size(8);

    // Pre-existing service born at borrow 3 (the owned-topic HOLD floor —
    // exactly the value the owned-topic warn is blind to on External): a
    // creator opener carrying a genuine borrow-3 requirement creates it.
    let mut creator = tt.default_topic_config();
    creator.subscriber_max_borrowed_samples = Some(3);
    let _creator_sub = tt
        .subscriber_with_buffers("tbs_floor_degraded/out", creator, 1)
        .expect("borrow-3 creator");

    // The loanable rmw shape: External + create floor 4, attaching to the
    // pre-existing borrow-3 service (open leg tolerant) ⇒ ONE warn.
    let mut floored = tt.default_topic_config();
    floored.create_borrow_floor = Some(4);
    assert_eq!(
        floored.publisher_provisioning,
        cerulion_core::transport::PublisherProvisioning::External,
        "the rmw opener shape is External-provisioned"
    );
    tt.subscriber_with_buffers("tbs_floor_degraded/out", floored, 1)
        .expect("floored External opener attaches to the smaller service");
    // Dedup: a second floored open of the same topic must NOT warn again.
    tt.subscriber_with_buffers("tbs_floor_degraded/out", floored, 1)
        .expect("second floored opener attaches");

    // Control 1: a pre-existing service AT the floor — no degradation.
    let mut creator4 = tt.default_topic_config();
    creator4.subscriber_max_borrowed_samples = Some(4);
    let _creator4_sub = tt
        .subscriber_with_buffers("tbs_floor_satisfied/out", creator4, 1)
        .expect("borrow-4 creator");
    tt.subscriber_with_buffers("tbs_floor_satisfied/out", floored, 1)
        .expect("floored opener attaches to a service at the floor");

    // Control 2: the floored opener CREATES the service (fresh topic) — it
    // is born at the floor, nothing to degrade from.
    tt.subscriber_with_buffers("tbs_floor_creator/out", floored, 1)
        .expect("floored opener creates its own service");

    // Control 3: a floor AT the iceoryx2 default (`Some(2)`, documented as
    // None-equivalent — it imposes nothing on the create leg) attaching to
    // a pre-existing borrow-1 service is a VALID configuration, not a
    // degradation: no warn. (Mutation oracle: reading the raw field instead
    // of the normalized floor fires here with requested=2 live=1.)
    let mut creator1 = tt.default_topic_config();
    creator1.subscriber_max_borrowed_samples = Some(1);
    let _creator1_sub = tt
        .subscriber_with_buffers("tbs_floor_noop/out", creator1, 1)
        .expect("borrow-1 creator");
    let mut noop_floor = tt.default_topic_config();
    noop_floor.create_borrow_floor = Some(2);
    tt.subscriber_with_buffers("tbs_floor_noop/out", noop_floor, 1)
        .expect("a no-op floor attaches to a borrow-1 service");

    logs_assert(|lines: &[&str]| {
        // Matched WITH the level token AND against the level-free total of the
        // same marker: a degraded warn demoted to INFO/ERROR would otherwise
        // still count as the one warn, and so would a second copy of it at
        // another level. (The control-topic absence check below stays
        // level-free — an absence holds at every level.)
        let floor_lines = lines_at_exclusively(lines, "WARN", &["loan borrow floor degraded"])?;
        if floor_lines.len() != 1 {
            return Err(format!(
                "expected exactly 1 borrow-floor degraded warn, got {}: {floor_lines:?}",
                floor_lines.len()
            ));
        }
        let line = floor_lines[0];
        for needle in ["tbs_floor_degraded/out", "requested=4", "live=3"] {
            if !line.contains(needle) {
                return Err(format!("degraded warn must carry `{needle}`: {line}"));
            }
        }
        // Neither control topic may appear on a floor-degraded line.
        if lines
            .iter()
            .any(|l| l.contains("loan borrow floor degraded") && !l.contains("tbs_floor_degraded"))
        {
            return Err("a control topic produced a borrow-floor degraded warn".to_string());
        }
        Ok(())
    });
}
