// SPDX-License-Identifier: AGPL-3.0-only
//! Resolved ports and trigger policies of a built graph runtime.

/// Version of the serialized run topology.
pub const RUN_TOPOLOGY_VERSION: u32 = 1;

/// A runtime's nodes in graph declaration order.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunTopology {
    pub version: u32,
    pub nodes: Vec<TopologyNode>,
}

/// The ports and scheduler trigger resolved for one node instance.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopologyNode {
    pub id: String,
    #[serde(rename = "type")]
    pub node_type: String,
    pub trigger: TopologyTrigger,
    /// Inputs in YAML declaration order, including non-trigger inputs.
    pub inputs: Vec<TopologyPort>,
    /// Outputs in YAML declaration order, which defines the write-mask bit order.
    pub outputs: Vec<TopologyPort>,
    /// Whether this entry can report which outputs it wrote during a tick.
    #[serde(default)]
    pub reports_writes: bool,
}

/// A port's local name and resolved topic.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TopologyPort {
    pub name: String,
    pub topic: String,
}

/// The scheduler's resolved trigger, with topics identifying its inputs.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum TopologyTrigger {
    Period {
        interval_ns: u64,
    },
    Data {
        /// The declared trigger input's topic. Empty for the default Data
        /// policy, which fires only when the host signals the node.
        topics: Vec<String>,
        /// True when the default Data policy has more than one input, so a
        /// host-signaled fire cannot name the input it served.
        ambiguous: bool,
    },
    Sync {
        topics: Vec<String>,
        /// None denotes unbounded synchronization.
        window_ns: Option<u64>,
    },
    #[serde(deserialize_with = "deserialize_external_trigger")]
    External,
}

// Serde's internally tagged unit variants otherwise ignore unknown fields.
fn deserialize_external_trigger<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<(), D::Error> {
    #[derive(serde::Deserialize)]
    #[serde(deny_unknown_fields)]
    struct ExternalFields {}

    <ExternalFields as serde::Deserialize>::deserialize(deserializer).map(|_| ())
}

impl TopologyTrigger {
    /// Stable fire-record cause byte; 0 is unknown and 2 is reserved.
    pub fn cause(&self) -> u8 {
        match self {
            Self::Period { .. } => 1,
            Self::Data { .. } => 3,
            Self::Sync { .. } => 4,
            Self::External => 5,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::VirtualClock;
    use crate::graph::node::{BackpressurePolicy, InputMeta, MacroPolicy};
    use crate::graph::{parse_graph, ClosureNodeEntry, GraphRuntime, NodeEntry, NodeInfo};
    use crate::wire::WireHeader;
    use indexmap::IndexMap;
    use serde_json::json;
    use std::sync::Arc;
    use std::time::Duration;

    const GRAPH: &str = r#"
prefix: robot
nodes:
  - id: source
    type: clock_source
    outputs:
      - {name: count, schema: std_msgs/UInt64, topic: /clock/count}
  - id: relay
    type: ordered_relay
    inputs:
      - {name: context, source: /app/context}
      - {name: event, source: /clock/count}
    outputs:
      - {name: z, schema: std_msgs/UInt64}
      - {name: a, schema: std_msgs/UInt64, topic: /relay/absolute}
  - id: sink
    type: aligned_sink
    inputs:
      - {name: context, source: /app/context}
      - {name: z, source: relay/z}
      - {name: a, source: /relay/absolute}
"#;

    fn meta(name: &str, trigger: bool) -> InputMeta {
        InputMeta {
            name: name.into(),
            schema_hash: 0,
            trigger,
            depth: 4,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
            consume: Default::default(),
        }
    }

    fn build_graph(sync: MacroPolicy) -> GraphRuntime {
        let infos = [
            (
                "source",
                NodeInfo::from_names(vec![], vec!["count".into()])
                    .with_policy(MacroPolicy::Period { period_ms: 1_000 }),
            ),
            (
                "relay",
                NodeInfo::from_names(
                    vec!["context".into(), "event".into()],
                    vec!["z".into(), "a".into()],
                )
                .with_policy(MacroPolicy::DataTrigger {
                    input_name: "event".into(),
                }),
            ),
            (
                "sink",
                NodeInfo::with_meta(
                    vec![meta("context", false), meta("z", true), meta("a", true)],
                    vec![],
                )
                .with_policy(sync),
            ),
        ];
        let factories: IndexMap<String, Box<dyn NodeEntry>> = infos
            .into_iter()
            .rev()
            .map(|(id, info)| {
                (
                    id.into(),
                    Box::new(ClosureNodeEntry::new(info, |_| Ok(()))) as Box<dyn NodeEntry>,
                )
            })
            .collect();
        GraphRuntime::build_for_test(
            parse_graph(GRAPH).unwrap(),
            factories,
            Arc::new(VirtualClock::new()),
            4,
        )
        .unwrap()
    }

    #[test]
    fn run_topology_describes_built_nodes_topics_and_trigger_policies_in_order() {
        let runtime = build_graph(MacroPolicy::Sync { window_ms: 12 });
        let topology = runtime.run_topology();
        assert_eq!(topology.version, RUN_TOPOLOGY_VERSION);
        assert_eq!(RUN_TOPOLOGY_VERSION, 1);
        assert_eq!(
            serde_json::to_value(&topology).unwrap(),
            json!({
                "version": 1,
                "nodes": [
                    {
                        "id": "source", "type": "clock_source",
                        "inputs": [],
                        "outputs": [{"name": "count", "topic": "/clock/count"}],
                        "trigger": {"kind": "period", "interval_ns": 1_000_000_000_u64},
                        "reports_writes": false
                    },
                    {
                        "id": "relay", "type": "ordered_relay",
                        "inputs": [
                            {"name": "context", "topic": "/app/context"},
                            {"name": "event", "topic": "/clock/count"}
                        ],
                        "outputs": [
                            {"name": "z", "topic": "/robot/relay/z"},
                            {"name": "a", "topic": "/relay/absolute"}
                        ],
                        "trigger": {"kind": "data", "topics": ["/clock/count"], "ambiguous": false},
                        "reports_writes": false
                    },
                    {
                        "id": "sink", "type": "aligned_sink",
                        "inputs": [
                            {"name": "context", "topic": "/app/context"},
                            {"name": "z", "topic": "/robot/relay/z"},
                            {"name": "a", "topic": "/relay/absolute"}
                        ],
                        "outputs": [],
                        "trigger": {"kind": "sync", "topics": ["/robot/relay/z", "/relay/absolute"], "window_ns": 12_000_000},
                        "reports_writes": false
                    }
                ]
            })
        );
        let decoded: RunTopology =
            serde_json::from_slice(&serde_json::to_vec(&topology).unwrap()).unwrap();
        assert_eq!(decoded, topology);
    }

    #[test]
    fn run_topology_preserves_unbounded_sync_and_excludes_context_inputs() {
        let runtime = build_graph(MacroPolicy::UnboundedSync);
        assert_eq!(
            runtime.run_topology().nodes[2].trigger,
            TopologyTrigger::Sync {
                topics: vec!["/robot/relay/z".into(), "/relay/absolute".into()],
                window_ns: None,
            }
        );
    }

    #[test]
    fn run_topology_marks_only_multi_input_default_data_as_ambiguous() {
        for (inputs, ambiguous) in [
            ("[]", false),
            ("[{name: z, source: /external/z}]", false),
            (
                "[{name: z, source: /external/z}, {name: a, source: /external/a}]",
                true,
            ),
        ] {
            let yaml = format!(
                "prefix: robot\nnodes:\n  - id: sink\n    type: default_data\n    inputs: {inputs}\n"
            );
            let config = parse_graph(&yaml).unwrap();
            let info = NodeInfo::from_names(
                config.nodes[0]
                    .inputs
                    .iter()
                    .map(|i| i.name.clone())
                    .collect(),
                vec![],
            );
            let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
            factories.insert(
                "sink".into(),
                Box::new(ClosureNodeEntry::new(info, |_| Ok(()))),
            );
            let runtime =
                GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 4)
                    .unwrap();
            assert_eq!(
                runtime.run_topology().nodes[0].trigger,
                TopologyTrigger::Data {
                    topics: vec![],
                    ambiguous,
                }
            );
        }
    }

    #[test]
    fn run_topology_lists_only_input_topics_whose_arrival_fires_the_node() {
        const FANOUT: &str = r#"
prefix: robot
nodes:
  - id: camera
    type: frame_source
    outputs:
      - {name: frame, schema: std_msgs/UInt64, topic: /camera/frame}
  - id: declared
    type: declared_trigger
    inputs:
      - {name: event, source: /camera/frame}
  - id: fallback
    type: default_data
    inputs:
      - {name: event, source: /camera/frame}
"#;
        let camera = ClosureNodeEntry::new(
            NodeInfo::from_names(vec![], vec!["frame".into()])
                .with_policy(MacroPolicy::Period { period_ms: 1 }),
            |ctx| {
                let mut frame = [0_u8; WireHeader::SIZE + 8];
                let mut header = WireHeader::new(0, 0, 0);
                header.total_size = frame.len() as u32;
                header.write_to_buf(&mut frame[..WireHeader::SIZE]);
                ctx.publisher_mut("frame")
                    .expect("frame is wired")
                    .publish_raw(&frame)
                    .map(drop)
            },
        );
        let declared = ClosureNodeEntry::new(
            NodeInfo::with_meta(vec![meta("event", true)], vec![]).with_policy(
                MacroPolicy::DataTrigger {
                    input_name: "event".into(),
                },
            ),
            |_| Ok(()),
        );
        let fallback =
            ClosureNodeEntry::new(NodeInfo::from_names(vec!["event".into()], vec![]), |_| {
                Ok(())
            });
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("camera".into(), Box::new(camera));
        factories.insert("declared".into(), Box::new(declared));
        factories.insert("fallback".into(), Box::new(fallback));
        let mut runtime = GraphRuntime::build_for_test(
            parse_graph(FANOUT).unwrap(),
            factories,
            Arc::new(VirtualClock::new()),
            8,
        )
        .unwrap();
        let fires = |runtime: &GraphRuntime, id: &str| {
            runtime.node_handle(id).expect("node is built").fire_count()
        };

        for _ in 0..3 {
            runtime.step(Duration::from_millis(1));
        }
        assert_eq!(fires(&runtime, "camera"), 3);
        assert!(fires(&runtime, "declared") > 0);
        assert_eq!(fires(&runtime, "fallback"), 0);
        runtime.signal_data("fallback").unwrap();
        runtime.step(Duration::from_millis(1));
        assert_eq!(fires(&runtime, "fallback"), 1);

        let topology = runtime.run_topology();
        assert_eq!(
            topology.nodes[1].trigger,
            TopologyTrigger::Data {
                topics: vec!["/camera/frame".into()],
                ambiguous: false,
            }
        );
        assert_eq!(
            topology.nodes[2].trigger,
            TopologyTrigger::Data {
                topics: vec![],
                ambiguous: false,
            }
        );
        assert_eq!(
            topology.nodes[2].inputs,
            vec![TopologyPort {
                name: "event".into(),
                topic: "/camera/frame".into(),
            }]
        );
    }

    #[test]
    fn run_topology_preserves_external_policy() {
        let config = parse_graph("nodes: [{id: source, type: external_source}]").unwrap();
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert(
            "source".into(),
            Box::new(ClosureNodeEntry::new(
                NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::External),
                |_| Ok(()),
            )),
        );
        let runtime =
            GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 4)
                .unwrap();
        assert_eq!(
            runtime.run_topology().nodes[0].trigger,
            TopologyTrigger::External
        );
    }

    #[test]
    fn run_topology_snapshot_is_owned_and_stable_across_steps() {
        let mut runtime = build_graph(MacroPolicy::Sync { window_ms: 12 });
        let expected = runtime.run_topology();
        let mut snapshot = runtime.run_topology();
        snapshot.nodes[0].id.clear();
        snapshot.nodes[1].outputs.reverse();
        snapshot.nodes[2].reports_writes = true;
        runtime.step(Duration::from_millis(1_000));
        assert_eq!(runtime.run_topology(), expected);
    }

    #[test]
    fn run_topology_trigger_causes_match_the_fire_wire_contract() {
        for (trigger, cause) in [
            (TopologyTrigger::Period { interval_ns: 1 }, 1),
            (
                TopologyTrigger::Data {
                    topics: vec![],
                    ambiguous: false,
                },
                3,
            ),
            (
                TopologyTrigger::Data {
                    topics: vec!["/a".into(), "/b".into()],
                    ambiguous: true,
                },
                3,
            ),
            (
                TopologyTrigger::Sync {
                    topics: vec![],
                    window_ns: Some(1),
                },
                4,
            ),
            (
                TopologyTrigger::Sync {
                    topics: vec![],
                    window_ns: None,
                },
                4,
            ),
            (TopologyTrigger::External, 5),
        ] {
            assert_eq!(trigger.cause(), cause);
        }
    }

    #[test]
    fn run_topology_node_defaults_missing_write_capability_to_false() {
        let node: TopologyNode = serde_json::from_value(json!({
            "id": "external", "type": "source", "trigger": {"kind": "external"},
            "inputs": [], "outputs": []
        }))
        .unwrap();
        assert!(!node.reports_writes);
    }

    #[test]
    fn run_topology_rejects_unknown_fields_at_every_document_boundary() {
        let topology = build_graph(MacroPolicy::Sync { window_ms: 12 }).run_topology();
        let value = serde_json::to_value(topology).unwrap();
        let mut unknown_topology = value.clone();
        unknown_topology["versoin"] = json!(1);
        assert!(serde_json::from_value::<RunTopology>(unknown_topology).is_err());

        let mut unknown_node = value["nodes"][1].clone();
        unknown_node["reports_write"] = json!(true);
        assert!(serde_json::from_value::<TopologyNode>(unknown_node).is_err());

        let mut unknown_port = value["nodes"][1]["outputs"][0].clone();
        unknown_port["topci"] = json!("/wrong");
        assert!(serde_json::from_value::<TopologyPort>(unknown_port).is_err());

        for node in value["nodes"].as_array().unwrap() {
            let mut unknown_trigger = node["trigger"].clone();
            unknown_trigger["interval_ms"] = json!(10);
            assert!(serde_json::from_value::<TopologyTrigger>(unknown_trigger).is_err());
        }
        let unknown_external = json!({"kind": "external", "topic": "/wrong"});
        assert!(serde_json::from_value::<TopologyTrigger>(unknown_external).is_err());
    }

    #[test]
    fn run_topology_does_not_enable_yaml_policy_overrides() {
        let error = parse_graph("nodes: [{id: source, type: source, policy: {trigger: input}}]")
            .unwrap_err();
        assert!(error.to_string().contains("policy"), "{error}");
    }
}
