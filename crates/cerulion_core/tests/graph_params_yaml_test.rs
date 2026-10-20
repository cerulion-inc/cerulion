// SPDX-License-Identifier: AGPL-3.0-only
//! The `params:` key on a graph YAML node entry: the round-trip and rejection
//! oracle arms a new config field owes the deny-unknown-fields contract, and
//! the graph-level resolution the runtime runs before any transport resource
//! exists.
//!
//! ```bash
//! cargo test -p cerulion_core --test graph_params_yaml_test
//! ```

use cerulion_core::graph::params::{resolve_graph_params, ParamSpec, ParamType, ParamValue};
use cerulion_core::graph::{parse_graph_raw, NodeInfo};
use indexmap::IndexMap;

const TWO_INSTANCES: &str = "\
prefix: demo
nodes:
  - id: left
    type: cam
    params:
      gain: 2.5
      frame_id: left_cam
    outputs:
      - name: cmd
        schema: geometry_msgs/Vector3
  - id: right
    type: cam
    params:
      gain: 0.5
      enabled: false
    outputs:
      - name: cmd
        schema: geometry_msgs/Vector3
  - id: plain
    type: cam
    outputs:
      - name: cmd
        schema: geometry_msgs/Vector3
";

/// What the `cam` type declares (the shape of the params fixture).
fn cam_info() -> NodeInfo {
    NodeInfo::from_names(vec![], vec!["cmd".into()]).with_params(vec![
        ParamSpec {
            name: "gain".into(),
            ty: ParamType::F64,
            default: ParamValue::Float(1.0),
            min: Some(ParamValue::Float(0.0)),
            max: Some(ParamValue::Float(10.0)),
            description: None,
        },
        ParamSpec {
            name: "frame_id".into(),
            ty: ParamType::String,
            default: ParamValue::Str("base_link".into()),
            min: None,
            max: None,
            description: None,
        },
        ParamSpec {
            name: "enabled".into(),
            ty: ParamType::Bool,
            default: ParamValue::Bool(true),
            min: None,
            max: None,
            description: None,
        },
    ])
}

fn infos_for(config: &cerulion_core::graph::GraphConfig) -> IndexMap<String, NodeInfo> {
    config
        .nodes
        .iter()
        .map(|n| (n.id.clone(), cam_info()))
        .collect()
}

fn pairs(values: &IndexMap<String, ParamValue>) -> Vec<(&str, ParamValue)> {
    values
        .iter()
        .map(|(k, v)| (k.as_str(), v.clone()))
        .collect()
}

#[test]
fn params_parse_into_a_flat_map_per_node_and_absent_means_empty() {
    let config = parse_graph_raw(TWO_INSTANCES).expect("parse");
    assert_eq!(
        pairs(&config.nodes[0].params),
        vec![
            ("gain", ParamValue::Float(2.5)),
            ("frame_id", ParamValue::Str("left_cam".into())),
        ]
    );
    assert_eq!(
        pairs(&config.nodes[1].params),
        vec![
            ("gain", ParamValue::Float(0.5)),
            ("enabled", ParamValue::Bool(false)),
        ]
    );
    assert!(config.nodes[2].params.is_empty());
}

#[test]
fn the_key_round_trips_and_is_omitted_when_empty() {
    let config = parse_graph_raw(TWO_INSTANCES).expect("parse");
    let yaml = serde_yaml::to_string(&config).expect("serialize");
    assert!(yaml.contains("params:"), "{yaml}");
    assert!(yaml.contains("gain: 2.5"), "{yaml}");
    assert!(yaml.contains("frame_id: left_cam"), "{yaml}");
    // The node without params gets no `params:` line of its own.
    assert_eq!(yaml.matches("params:").count(), 2, "{yaml}");
    let again = parse_graph_raw(&yaml).expect("reparse");
    assert_eq!(again.nodes[0].params, config.nodes[0].params);
    assert_eq!(again.nodes[1].params, config.nodes[1].params);
    assert!(again.nodes[2].params.is_empty());
}

#[test]
fn a_list_or_a_nested_map_under_params_is_refused_at_parse() {
    let list = "prefix: demo\nnodes:\n  - id: a\n    type: cam\n    params:\n      - gain\n";
    let err = parse_graph_raw(list).expect_err("a list is not a params map");
    assert!(err.to_string().contains("params"), "{err}");
    let nested = "prefix: demo\nnodes:\n  - id: a\n    type: cam\n    params:\n      motor:\n        gain: 1.0\n";
    let err = parse_graph_raw(nested).expect_err("a nested map is not a scalar value");
    assert!(err.to_string().contains("params"), "{err}");
}

#[test]
fn a_misspelled_params_key_is_refused_naming_it() {
    let yaml = "prefix: demo\nnodes:\n  - id: a\n    type: cam\n    parms:\n      gain: 1.0\n";
    let err = parse_graph_raw(yaml).expect_err("a misspelled key is never dropped");
    assert!(err.to_string().contains("parms"), "{err}");
}

#[test]
fn graph_resolution_gives_every_node_its_full_map_with_defaults_filled() {
    let config = parse_graph_raw(TWO_INSTANCES).expect("parse");
    let resolved = resolve_graph_params(&config, &infos_for(&config)).expect("resolve");
    assert_eq!(
        pairs(&resolved["left"]),
        vec![
            ("gain", ParamValue::Float(2.5)),
            ("frame_id", ParamValue::Str("left_cam".into())),
            ("enabled", ParamValue::Bool(true)),
        ]
    );
    assert_eq!(
        pairs(&resolved["right"]),
        vec![
            ("gain", ParamValue::Float(0.5)),
            ("frame_id", ParamValue::Str("base_link".into())),
            ("enabled", ParamValue::Bool(false)),
        ]
    );
    assert_eq!(
        pairs(&resolved["plain"]),
        vec![
            ("gain", ParamValue::Float(1.0)),
            ("frame_id", ParamValue::Str("base_link".into())),
            ("enabled", ParamValue::Bool(true)),
        ]
    );
}

#[test]
fn graph_resolution_refuses_each_class_naming_node_and_param() {
    let cases: [(&str, &str); 3] = [
        (
            "params:\n      gian: 2.5\n",
            "node 'a': unknown param 'gian' (did you mean 'gain'?); declared params: gain, frame_id, enabled",
        ),
        (
            "params:\n      gain: 2\n",
            "node 'a': param 'gain' expects f64, got integer 2; a float param takes a float value, for example 2.0",
        ),
        (
            "params:\n      gain: 11.0\n",
            "node 'a': param 'gain' = 11.0 is outside [0.0, 10.0]",
        ),
    ];
    for (params_block, expected) in cases {
        let yaml = format!(
            "prefix: demo\nnodes:\n  - id: a\n    type: cam\n    {params_block}    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n"
        );
        let config = parse_graph_raw(&yaml).expect("the YAML itself is well formed");
        let err = resolve_graph_params(&config, &infos_for(&config)).expect_err(expected);
        assert_eq!(
            err.to_string(),
            format!("Graph validation error: {expected}")
        );
    }
}

#[test]
fn params_on_a_node_type_that_declares_none_is_refused() {
    let yaml = "prefix: demo\nnodes:\n  - id: a\n    type: ticker\n    params:\n      gain: 1.0\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n";
    let config = parse_graph_raw(yaml).expect("parse");
    let mut infos = IndexMap::new();
    infos.insert(
        "a".to_string(),
        NodeInfo::from_names(vec![], vec!["cmd".into()]),
    );
    let err = resolve_graph_params(&config, &infos).expect_err("no params declared");
    assert_eq!(
        err.to_string(),
        "Graph validation error: node 'a': unknown param 'gain'; this node type declares no params, so `params:` is not accepted on it"
    );
}

#[test]
fn params_on_a_ros2_entry_is_refused_pointing_at_the_ros2_block() {
    let yaml = "prefix: demo\nnodes:\n  - id: talker\n    ros2:\n      package: demo_nodes_cpp\n      executable: talker\n    params:\n      rate: 2\n";
    let config = parse_graph_raw(yaml).expect("parse");
    let err =
        resolve_graph_params(&config, &IndexMap::new()).expect_err("ros2 entries take ros2.params");
    assert!(
        err.to_string().contains("ros2.params") && err.to_string().contains("'talker'"),
        "{err}"
    );
}

#[test]
fn a_string_param_given_off_stays_the_string_off_through_the_graph_file() {
    let yaml = "prefix: demo\nnodes:\n  - id: a\n    type: cam\n    params:\n      frame_id: off\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n";
    let config = parse_graph_raw(yaml).expect("parse");
    let resolved = resolve_graph_params(&config, &infos_for(&config)).expect("resolve");
    assert_eq!(resolved["a"]["frame_id"], ParamValue::Str("off".into()));
}

#[test]
fn resolution_is_deterministic_across_two_runs() {
    let config = parse_graph_raw(TWO_INSTANCES).expect("parse");
    let first = resolve_graph_params(&config, &infos_for(&config)).expect("resolve");
    let second = resolve_graph_params(&config, &infos_for(&config)).expect("resolve");
    assert_eq!(first, second);
    assert_eq!(
        first.keys().cloned().collect::<Vec<_>>(),
        vec!["left", "right", "plain"],
        "node order follows the graph file"
    );
}
