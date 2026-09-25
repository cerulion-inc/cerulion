// SPDX-License-Identifier: AGPL-3.0-only
//! Startup-race pre-create of OWNED topic services.
//!
//! The graph pre-creates every owned topic's iceoryx2 service with its final
//! topology-derived config at build start, BEFORE any port wires, so it
//! claims the service ahead of a foreign/default opener that would otherwise
//! win the create race at the default ceiling. The node loop then opens owned
//! topics OPEN-ONLY (`.open()`), making the pre-create **load-bearing**:
//!
//!   - If the pre-create pass were removed (or its owned/External filter
//!     inverted), the first owned open-only `.open()` would hit `DoesNotExist`
//!     and `build` would fail. So an owned-topic graph BUILDING SUCCESSFULLY
//!     is the load-bearing pin — it passes here and fails under either of
//!     those changes.
//!   - If pre-create used the wrong (default) config instead of the
//!     topology-derived one, a raised-ceiling owned topic's deep consumer
//!     port could not attach and `build` would fail — so the deep-consumer
//!     build succeeding pins config-correctness.
//!
//! Determinism note: the pre-create benefit (claiming services EARLY) is a
//! concurrency-window shrink that is not itself deterministically observable
//! in a single-process test — the contracts pinned here (owned services
//! exist at the graph's caps; the load-bearing open-only path) are what the
//! deterministic suite can verify, and they fail loudly under the regressions
//! above. The External path (NOT pre-created — create-or-opened by the
//! consumer) is regression-guarded by `absolute_source_external_iox2_test`.
//!
//! Per-test SHM root via `build_for_test` — parallel-safe.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// Period producer: one Vector3 per tick into a graph-OWNED topic.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PreProducer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl PreProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Data-trigger consumer (default depth) of an owned topic.
#[cerulion_node]
#[derive(Default)]
struct PreConsumer {
    #[input(trigger)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl PreConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// Data-trigger consumer with depth 16 — raises the owned topic's ceiling
/// above the global default (8), exercising the config-correctness pin.
#[cerulion_node]
#[derive(Default)]
struct PreDeepConsumer {
    #[input(trigger, depth = 16)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl PreDeepConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// Data-trigger consumer with depth 32 — raises the owned topic's ceiling
/// to 32 (well above iceoryx2's subscriber-buffer floor), so a graph built
/// on top of a ceiling-16 incumbent service is rejected at pre-create.
#[cerulion_node]
#[derive(Default)]
struct PreVeryDeepConsumer {
    #[input(trigger, depth = 32)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl PreVeryDeepConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// producer → consumer over the OWNED topic `{prefix}/producer/out`.
fn owned_graph(
    prefix: &str,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "precreate_owned".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "pre_producer".to_string(),
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
                node_type: "pre_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(PreProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(PreConsumerEntry::with_state(PreConsumer {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// producer → DEEP consumer (depth 16) over the OWNED topic — ceiling 16.
fn deep_owned_graph(
    prefix: &str,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "precreate_deep".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "pre_producer".to_string(),
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
                node_type: "pre_deep_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(PreProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(PreDeepConsumerEntry::with_state(PreDeepConsumer {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// producer → VERY-DEEP consumer (depth 32) over the OWNED topic — ceiling 32.
fn very_deep_owned_graph(
    prefix: &str,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "precreate_very_deep".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "pre_producer".to_string(),
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
                node_type: "pre_very_deep_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(PreProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(PreVeryDeepConsumerEntry::with_state(PreVeryDeepConsumer {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

#[test]
fn owned_topic_graph_builds_and_data_flows_through_precreated_service() {
    // The producer's topic is graph-OWNED, so the build pre-creates its
    // service before any port wires, and the publisher + the consumer's
    // subscriber open it OPEN-ONLY. If the pre-create pass were removed (or
    // its owned/External filter inverted), the first owned open-only `.open()`
    // would hit DoesNotExist and this build would FAIL — so the build
    // succeeding is the load-bearing pin. Data flowing end-to-end (the
    // data-trigger consumer fires) proves the pre-created service is the real
    // one the ports attached to.
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = owned_graph("preca", Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("owned-topic graph must build — the pre-create pass created the service");
    for _ in 0..6 {
        runtime.step(Duration::from_millis(10));
    }
    assert!(
        runtime.node_handle("producer").unwrap().fire_count() > 0,
        "period producer must fire"
    );
    assert!(
        fires.load(Ordering::Relaxed) > 0,
        "data-trigger consumer must fire — data flowed through the pre-created owned service"
    );
}

#[test]
fn owned_service_created_at_graph_caps_default_opener_attaches() {
    // A deep consumer (depth 16 > default ceiling 8) raises the owned topic's
    // ceiling to 16. The build SUCCEEDING with a depth-16 consumer proves the
    // pre-created service is at ceiling 16 (a default-config pre-create would
    // leave the depth-16 consumer port unable to attach, failing the build —
    // the config-correctness pin). A default opener (ceiling 8 ≤ 16) then
    // attaches to the graph-created service, proving it exists at the graph's
    // caps.
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = deep_owned_graph("precb", Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("deep-consumer graph builds — pre-create made the owned service at ceiling 16");
    let mgr = runtime.test_transport().expect("test transport parked");
    // A default opener (ceiling 8 ≤ the graph's 16) attaches to the
    // graph-owned, pre-created service. Held to scope end so the subscriber
    // (and the service reference it holds) stays alive through the asserts.
    let _default_opener = mgr
        .create_subscriber("precb/producer/out")
        .expect("default opener attaches to the graph-created owned service (ceiling 8 ≤ 16)");
    for _ in 0..4 {
        runtime.step(Duration::from_millis(10));
    }
    assert!(
        runtime.node_handle("producer").unwrap().fire_count() > 0,
        "producer still runs after the introspection opener attached"
    );
    assert!(
        fires.load(Ordering::Relaxed) > 0,
        "deep consumer fires — data flows through the raised-ceiling owned service"
    );
}

#[test]
fn precreate_build_is_deterministic_across_runs() {
    // Two fresh builds of the same owned-topic graph (separate per-test SHM
    // roots) produce identical producer/consumer fire counts — the
    // pre-create pass introduces no nondeterminism (Principle #7).
    let run = |prefix: &str| -> (u64, u64) {
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = owned_graph(prefix, Arc::clone(&fires));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("owned-topic graph builds");
        for _ in 0..8 {
            runtime.step(Duration::from_millis(10));
        }
        (
            runtime.node_handle("producer").unwrap().fire_count(),
            fires.load(Ordering::Relaxed),
        )
    };
    // Distinct prefixes only to avoid any same-name interaction; the
    // build/step logic is identical, so the counts must match.
    let a = run("precd1");
    let b = run("precd2");
    assert_eq!(
        a, b,
        "pre-create + build + step must be bit-identical across runs"
    );
    assert!(a.0 > 0 && a.1 > 0, "both nodes fired");
}

#[test]
fn precreate_rejects_topic_owned_by_a_running_graph_at_an_incompatible_ceiling() {
    // The core ordering-edge the pre-create pass closes, modeled realistically: graph A
    // (running) provisioned the OWNED topic's service at ceiling 16 — its
    // PUBLISHER commits the buffer, the proven floor-independent enforcement
    // `topic_buffer_sizing_test` pins. Graph B, sharing the prefix, declares a
    // depth-32 consumer on the SAME topic → it requires ceiling 32 and must be
    // REJECTED at the PRE-CREATE pass, loudly, with the ordering-edge remedy,
    // instead of dying order-dependently at a later open site. (A
    // subscriber-only foreign service can't be used here: iceoryx2 floors a
    // small `subscriber_max_buffer_size` to its internal minimum, so it never
    // ends up below the graph's requirement — hence a publisher-backed
    // incumbent built at a known ceiling.)
    let clock_a = Arc::new(VirtualClock::new());
    let (config_a, factories_a) = deep_owned_graph("precx", Arc::new(AtomicU64::new(0)));
    let runtime_a = GraphRuntime::build_for_test(config_a, factories_a, clock_a, 8)
        .expect("graph A builds, provisioning the owned service at ceiling 16");
    let mgr = runtime_a.test_transport().expect("test transport parked");
    // Graph B (same prefix → same topic) requires ceiling 32 on graph A's
    // ceiling-16 service, built over the SAME transport so the incumbent
    // service is already present. `runtime_a` is held to scope end, keeping
    // graph A's publisher (and the ceiling-16 service) alive through the build.
    let clock_b = Arc::new(VirtualClock::new());
    let (config_b, factories_b) = very_deep_owned_graph("precx", Arc::new(AtomicU64::new(0)));
    let err = match GraphRuntime::build(config_b, factories_b, mgr, clock_b) {
        Ok(_) => panic!(
            "graph B build must FAIL — it requires ceiling 32 on a topic graph A \
             already provisioned at ceiling 16"
        ),
        Err(e) => format!("{e:?}"),
    };
    assert!(
        err.contains("smaller buffer ceiling") || err.contains("start the graph first"),
        "the pre-create rejection must name the ordering-edge remedy, got: {err}"
    );
}
