// SPDX-License-Identifier: AGPL-3.0-only
//! The sample app's graph asset is the graph its runtime library links.
//!
//! The emulator test reads the topics and values this graph produces
//! (`/app/timer/count` counting up by 2, `/app/relay/echo` copying it), so
//! the oracle below is written by hand from that contract, not read back
//! from the file.

use std::path::Path;

use cerulion_core::graph::params::ParamValue;
use cerulion_core::graph::parse_graph;

fn asset_text() -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("android/app/src/main/assets/graph.yaml");
    std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("read the sample graph asset {}: {error}", path.display()))
}

#[test]
fn sample_graph_asset_parses_with_prefix_app_and_three_native_nodes() {
    let graph = parse_graph(&asset_text()).expect("the sample graph asset parses");

    // An explicit prefix: without it the parser takes the host name.
    assert_eq!(graph.prefix, "app");
    assert_eq!(graph.native_nodes().count(), 3);
    assert_eq!(graph.nodes.len(), 3);

    let nodes: Vec<(&str, &str)> = graph
        .nodes
        .iter()
        .map(|node| (node.id.as_str(), node.node_type.as_str()))
        .collect();
    assert_eq!(
        nodes,
        [
            ("timer", "timer"),
            ("relay", "relay"),
            ("printer", "printer")
        ]
    );

    let timer = &graph.nodes[0];
    assert_eq!(timer.params.get("step"), Some(&ParamValue::Int(2)));
    assert_eq!(timer.outputs.len(), 1);
    assert_eq!(timer.outputs[0].name, "count");

    let relay = &graph.nodes[1];
    assert_eq!(relay.inputs.len(), 1);
    assert_eq!(relay.inputs[0].name, "count");
    assert_eq!(relay.inputs[0].source, "timer/count");
    assert_eq!(relay.outputs.len(), 1);
    assert_eq!(relay.outputs[0].name, "echo");

    let printer = &graph.nodes[2];
    assert_eq!(printer.inputs.len(), 1);
    assert_eq!(printer.inputs[0].name, "count");
    assert_eq!(printer.inputs[0].source, "relay/echo");
    assert!(printer.outputs.is_empty());
}

#[test]
fn sample_graph_asset_has_no_launch_args_or_process_groups() {
    // The embedded runtime does not resolve launch arguments, and an app
    // cannot run process groups, so the asset must use neither.
    let text = asset_text();
    assert!(
        !text.contains("${"),
        "the asset must not use launch arguments"
    );
    assert!(!text.contains("args:"), "the asset must not declare args");
    assert!(
        !text.contains("process_groups"),
        "the asset must not declare process groups"
    );
}
