// SPDX-License-Identifier: AGPL-3.0-only
//! `graph_edit`: wiring, unwiring and unstaging edit a hand-authored graph in
//! place.
//!
//! The oracle is the DOCUMENT: each arm asserts the exact text of the file
//! after the edit, so a writer that kept the comments but moved the `network:`
//! block, or re-spelled a quoted scalar, fails. A refused edit must leave the
//! file byte-identical and write no backup.
//!
//! Parallel-safe: a scratch workspace per test, no transport, no process spawn.

use std::path::Path;

use cerulion_cli_engine::graph_edit::{
    graph_unstage, graph_unwire, graph_wire, GraphEditError, PortRef,
};
use cerulion_cli_engine::node_cmd;
use cerulion_cli_engine::workspace::{workspace_create, CerulionWorkspace};
use tempfile::TempDir;

fn port(node: &str, port: &str) -> PortRef {
    PortRef {
        node: node.to_string(),
        port: port.to_string(),
    }
}

/// A workspace with three node types: `camera` (output `image`), `detector`
/// (input `image`, output `boxes`) and `logger` (input `data` of a schema the
/// camera does not produce).
fn workspace(tmp: &Path) -> CerulionWorkspace {
    let ws = workspace_create(tmp, "ws").expect("workspace");
    for (node, ports) in [
        ("camera", vec![("image", "sensor_msgs/Image", true)]),
        (
            "detector",
            vec![
                ("image", "sensor_msgs/Image", false),
                ("boxes", "geometry_msgs/Vector3", true),
            ],
        ),
        ("logger", vec![("data", "geometry_msgs/Vector3", false)]),
    ] {
        node_cmd::node_create(&ws.nodes_dir, &ws.root.join("Cargo.toml"), node, None)
            .expect("node create");
        for (name, schema, is_output) in ports {
            node_cmd::node_modify_add_port(
                &ws.nodes_dir,
                node,
                name,
                Some(schema),
                is_output,
                false,
            )
            .expect("declare port");
        }
    }
    ws
}

/// Hand-authored: header comment, a per-node comment, two-space list
/// indentation, a trailing comment, and a `network:` block after `nodes:`.
const HAND_AUTHORED: &str = "\
# Perception graph, hand-authored.
prefix: percep
nodes:
  # The camera is deliberately first.
  - id: cam
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
  - id: det # the detector
    type: detector
    outputs:
      - name: boxes
        schema: geometry_msgs/Vector3
  - id: log
    type: logger

# Only the boxes leave this machine.
network:
  mode: disabled
";

fn write_graph(ws: &CerulionWorkspace, body: &str) {
    std::fs::write(ws.graphs_dir.join("percep.yaml"), body).unwrap();
}

fn read_graph(ws: &CerulionWorkspace) -> String {
    std::fs::read_to_string(ws.graphs_dir.join("percep.yaml")).unwrap()
}

fn no_backup(ws: &CerulionWorkspace) -> bool {
    !ws.graphs_dir.join("percep.yaml.bak").exists()
}

#[test]
fn wiring_into_a_hand_authored_graph_preserves_every_byte_it_did_not_add() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(&ws, HAND_AUTHORED);

    let out = graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    let expected = HAND_AUTHORED.replace(
        "  - id: det # the detector\n    type: detector\n",
        "  - id: det # the detector\n    type: detector\n    inputs:\n      - name: image\n        source: cam/image\n",
    );
    assert_eq!(out.raw, expected);
    assert_eq!(read_graph(&ws), expected);
    assert_eq!(
        std::fs::read_to_string(ws.graphs_dir.join("percep.yaml.bak")).unwrap(),
        HAND_AUTHORED,
        "the prior bytes are backed up, as `node stage` does"
    );
}

#[test]
fn unwiring_restores_the_document_byte_for_byte() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(&ws, HAND_AUTHORED);
    graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    let out = graph_unwire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert_eq!(out.raw, HAND_AUTHORED);
    assert_eq!(read_graph(&ws), HAND_AUTHORED);
}

#[test]
fn unstaging_a_node_removes_only_its_entry() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(&ws, HAND_AUTHORED);
    let out = graph_unstage(&ws, "percep", "log", false).unwrap();
    assert_eq!(
        out.raw,
        HAND_AUTHORED.replace("  - id: log\n    type: logger\n", "")
    );
    assert!(out.removed_wires.is_empty());
}

#[test]
fn a_wire_to_a_mismatched_schema_is_refused_and_writes_nothing() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(&ws, HAND_AUTHORED);
    match graph_wire(&ws, "percep", &port("cam", "image"), &port("log", "data")) {
        Err(GraphEditError::SchemaMismatch {
            expected, found, ..
        }) => {
            assert_eq!(expected, "geometry_msgs/Vector3");
            assert_eq!(found, "sensor_msgs/Image");
        }
        other => panic!("expected a schema mismatch, got {other:?}"),
    }
    assert_eq!(read_graph(&ws), HAND_AUTHORED);
    assert!(no_backup(&ws));
}

#[test]
fn an_agreeing_bare_spelling_is_not_a_mismatch() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    // `Image` and `sensor_msgs/Image` name one schema; the graph may use
    // either spelling and a wire is still compatible.
    write_graph(
        &ws,
        &HAND_AUTHORED.replace("schema: sensor_msgs/Image", "schema: Image"),
    );
    let out = graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert!(
        out.raw.contains("        source: cam/image\n"),
        "{}",
        out.raw
    );
    assert_eq!(read_graph(&ws), out.raw);
}

#[test]
fn unstaging_a_node_that_feeds_others_lists_the_wires_unless_forced() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(&ws, HAND_AUTHORED);
    graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    let wired = read_graph(&ws);
    std::fs::remove_file(ws.graphs_dir.join("percep.yaml.bak")).unwrap();

    match graph_unstage(&ws, "percep", "cam", false) {
        Err(GraphEditError::WouldBreak { wires }) => {
            assert_eq!(wires.len(), 1);
            assert_eq!(wires[0].from, port("cam", "image"));
            assert_eq!(wires[0].to, port("det", "image"));
        }
        other => panic!("expected would_break, got {other:?}"),
    }
    assert_eq!(read_graph(&ws), wired, "a refusal writes nothing");
    assert!(no_backup(&ws));

    let out = graph_unstage(&ws, "percep", "cam", true).unwrap();
    assert_eq!(out.removed_wires.len(), 1);
    assert_eq!(
        out.raw,
        HAND_AUTHORED.replace(
            "  # The camera is deliberately first.\n  - id: cam\n    type: camera\n    outputs:\n      - name: image\n        schema: sensor_msgs/Image\n",
            "  # The camera is deliberately first.\n"
        ),
        "the node and the wire into the detector are gone, the comment stays"
    );
}

#[test]
fn engine_refusals_leave_the_file_untouched() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(&ws, HAND_AUTHORED);
    graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    let wired = read_graph(&ws);
    std::fs::remove_file(ws.graphs_dir.join("percep.yaml.bak")).unwrap();

    let refusals: Vec<(&str, Result<_, GraphEditError>)> = vec![
        (
            "already wired",
            graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")),
        ),
        (
            "unknown producer node",
            graph_wire(&ws, "percep", &port("nope", "image"), &port("det", "image")),
        ),
        (
            "unknown producer port",
            graph_wire(&ws, "percep", &port("cam", "nope"), &port("det", "image")),
        ),
        (
            "unknown consumer port",
            graph_wire(&ws, "percep", &port("cam", "image"), &port("log", "nope")),
        ),
        (
            "no such wire",
            graph_unwire(&ws, "percep", &port("cam", "image"), &port("log", "data")),
        ),
        ("unknown node", graph_unstage(&ws, "percep", "nope", false)),
        (
            "unknown graph",
            graph_wire(&ws, "missing", &port("cam", "image"), &port("det", "image")),
        ),
    ];
    for (what, result) in refusals {
        assert!(
            matches!(result, Err(GraphEditError::Cli(_))),
            "{what}: {result:?}"
        );
    }
    assert_eq!(read_graph(&ws), wired);
    assert!(no_backup(&ws));
}

#[test]
fn a_layout_the_splice_cannot_edit_is_refused_not_rewritten() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    // The consumer's `inputs:` is a flow value: not editable in place.
    let doc = HAND_AUTHORED.replace(
        "  - id: log\n    type: logger\n",
        "  - id: log\n    type: logger\n    inputs: [{name: data, source: det/boxes}]\n",
    );
    write_graph(&ws, &doc);
    let result = graph_unwire(&ws, "percep", &port("det", "boxes"), &port("log", "data"));
    assert!(matches!(result, Err(GraphEditError::Cli(_))), "{result:?}");
    assert_eq!(read_graph(&ws), doc);
    assert!(no_backup(&ws));
}

#[test]
fn removing_the_only_node_is_refused() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(
        &ws,
        "prefix: percep\nnodes:\n  - id: cam\n    type: camera\n",
    );
    let before = read_graph(&ws);
    let result = graph_unstage(&ws, "percep", "cam", false);
    assert!(matches!(result, Err(GraphEditError::Cli(_))), "{result:?}");
    assert_eq!(read_graph(&ws), before);
    assert!(no_backup(&ws));
}

#[test]
fn a_graph_without_a_prefix_line_can_be_wired_and_unstaged() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    let doc = HAND_AUTHORED.replace("prefix: percep\n", "");
    write_graph(&ws, &doc);
    let out = graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert!(!out.raw.contains("prefix:"), "no prefix line is invented");
    assert!(out.raw.contains("source: cam/image"), "{}", out.raw);
    let out = graph_unstage(&ws, "percep", "log", false).unwrap();
    assert!(!out.raw.contains("id: log"), "{}", out.raw);
}

#[test]
fn a_topic_another_node_still_publishes_keeps_its_readers_on_unstage() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    let doc = "\
prefix: percep
multi_publisher_topics: [/shared/image]
nodes:
  - id: cam_a
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
        topic: /shared/image
  - id: cam_b
    type: camera
    outputs:
      - name: image
        schema: sensor_msgs/Image
        topic: /shared/image
  - id: det
    type: detector
    inputs:
      - name: image
        source: /shared/image
";
    write_graph(&ws, doc);
    let out = graph_unstage(&ws, "percep", "cam_a", false).unwrap();
    assert!(out.removed_wires.is_empty());
    assert!(out.raw.contains("source: /shared/image"), "{}", out.raw);
    assert!(!out.raw.contains("cam_a"), "{}", out.raw);
    // The last publisher is a real dependency again.
    let result = graph_unstage(&ws, "percep", "cam_b", false);
    assert!(
        matches!(result, Err(GraphEditError::WouldBreak { ref wires }) if wires.len() == 1),
        "{result:?}"
    );
}

#[test]
fn removing_the_only_input_keeps_a_comment_on_the_inputs_key() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    let doc = HAND_AUTHORED.replace(
        "  - id: det # the detector\n    type: detector\n",
        "  - id: det # the detector\n    type: detector\n    inputs: # camera feed\n      - name: image\n        source: cam/image\n",
    );
    write_graph(&ws, &doc);
    let out = graph_unwire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    let expected = HAND_AUTHORED.replace(
        "  - id: det # the detector\n    type: detector\n",
        "  - id: det # the detector\n    type: detector\n    inputs: [] # camera feed\n",
    );
    assert_eq!(out.raw, expected);
    // And it wires again.
    let again = graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert!(again.raw.contains("source: cam/image"), "{}", again.raw);
}

#[test]
fn a_graph_without_a_prefix_line_resolves_an_absolute_host_topic_for_unstage() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    let host = cerulion_core::graph::default_prefix("");
    let doc = HAND_AUTHORED.replace("prefix: percep\n", "").replace(
        "  - id: log\n    type: logger\n",
        &format!(
            "  - id: log\n    type: logger\n    inputs:\n      - name: frames\n        source: /{host}/cam/image\n"
        ),
    );
    write_graph(&ws, &doc);
    let err = graph_unstage(&ws, "percep", "cam", false).unwrap_err();
    assert!(
        matches!(err, GraphEditError::WouldBreak { .. }),
        "the consumer of the host-prefixed topic blocks the unstage: {err:?}"
    );
    assert_eq!(read_graph(&ws), doc, "a refusal writes nothing");
}

#[test]
fn a_topic_override_is_wired_by_its_absolute_topic() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    write_graph(
        &ws,
        &HAND_AUTHORED.replace(
            "        schema: sensor_msgs/Image\n  - id: det",
            "        schema: sensor_msgs/Image\n        topic: /shared/image\n  - id: det",
        ),
    );
    let out = graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert!(
        out.raw.contains("        source: /shared/image\n"),
        "{}",
        out.raw
    );
    let back = graph_unwire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert!(!back.raw.contains("source:"), "{}", back.raw);
}

#[test]
fn removing_the_only_input_takes_its_own_line_comment_with_it() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    let doc = HAND_AUTHORED.replace(
        "  - id: det # the detector\n    type: detector\n",
        "  - id: det # the detector\n    type: detector\n    inputs:\n      # camera feed\n      - name: image\n        source: cam/image\n",
    );
    write_graph(&ws, &doc);
    let out = graph_unwire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert_eq!(out.raw, HAND_AUTHORED);
}

#[test]
fn wiring_a_node_keeps_the_comment_above_its_outputs_attached() {
    let tmp = TempDir::new().unwrap();
    let ws = workspace(tmp.path());
    let doc = HAND_AUTHORED.replace(
        "    type: detector\n    outputs:\n",
        "    type: detector\n    # what the detector emits\n    outputs:\n",
    );
    write_graph(&ws, &doc);
    let out = graph_wire(&ws, "percep", &port("cam", "image"), &port("det", "image")).unwrap();
    assert!(
        out.raw.contains(
            "    inputs:\n      - name: image\n        source: cam/image\n    # what the detector emits\n    outputs:\n"
        ),
        "{}",
        out.raw
    );
}
