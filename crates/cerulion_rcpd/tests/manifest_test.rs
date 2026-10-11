// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp.yaml` manifest: parse, resolve the graph, refuse bad input.

use cerulion_rcpd::manifest::{DeclaredNode, RcpManifest, RobotIdentity};
use cerulion_rcpd::RcpdError;

#[test]
fn an_empty_manifest_is_the_default() {
    assert_eq!(
        RcpManifest::parse("").expect("parse"),
        RcpManifest::default()
    );
}

#[test]
fn robot_and_graph_parse() {
    let m = RcpManifest::parse("robot: go2\ngraph: graphs/go2.yaml\n").expect("parse");
    assert_eq!(m.robot.as_deref(), Some("go2"));
    assert_eq!(
        m.graph.as_deref(),
        Some(std::path::Path::new("graphs/go2.yaml"))
    );
}

#[test]
fn an_unknown_key_is_refused() {
    let err = RcpManifest::parse("robot: go2\ntools: []\n").expect_err("unknown key");
    assert!(err.contains("tools"), "{err}");
}

#[test]
fn the_graph_resolves_relative_to_the_manifest() {
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::create_dir(dir.path().join("graphs")).expect("mkdir");
    std::fs::write(
        dir.path().join("graphs/g.yaml"),
        "name: g\nprefix: from_graph\nnodes:\n  - id: lidar\n    type: lidar_node\n",
    )
    .expect("write graph");
    let manifest = dir.path().join("rcp.yaml");
    std::fs::write(&manifest, "graph: graphs/g.yaml\n").expect("write manifest");
    assert_eq!(
        RcpManifest::load(&manifest).expect("load"),
        RobotIdentity {
            robot: "from_graph".into(),
            nodes: vec![DeclaredNode {
                node_id: "lidar".into(),
                node_type: "lidar_node".into(),
            }],
        }
    );
}

#[test]
fn the_manifest_robot_wins_over_the_graph_prefix() {
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        dir.path().join("g.yaml"),
        "name: g\nprefix: from_graph\nnodes: []\n",
    )
    .expect("write graph");
    let manifest = dir.path().join("rcp.yaml");
    std::fs::write(&manifest, "robot: named\ngraph: g.yaml\n").expect("write manifest");
    let identity = RcpManifest::load(&manifest).expect("load");
    assert_eq!(identity.robot, "named");
    assert!(identity.nodes.is_empty());
}

#[test]
fn a_missing_manifest_is_a_read_error() {
    let dir = tempfile::tempdir().expect("tempdir");
    let err = RcpManifest::load(&dir.path().join("absent.yaml")).expect_err("missing");
    assert!(matches!(err, RcpdError::ManifestRead { .. }), "{err:?}");
}

#[test]
fn a_malformed_manifest_is_a_parse_error() {
    let dir = tempfile::tempdir().expect("tempdir");
    let manifest = dir.path().join("rcp.yaml");
    std::fs::write(&manifest, "robot: [unclosed\n").expect("write");
    let err = RcpManifest::load(&manifest).expect_err("malformed");
    assert!(matches!(err, RcpdError::ManifestParse { .. }), "{err:?}");
}

#[test]
fn a_missing_graph_is_a_graph_read_error() {
    let dir = tempfile::tempdir().expect("tempdir");
    let manifest = dir.path().join("rcp.yaml");
    std::fs::write(&manifest, "graph: absent.yaml\n").expect("write");
    let err = RcpManifest::load(&manifest).expect_err("missing graph");
    assert!(matches!(err, RcpdError::GraphRead { .. }), "{err:?}");
}

#[test]
fn a_malformed_graph_is_a_graph_parse_error() {
    let dir = tempfile::tempdir().expect("tempdir");
    std::fs::write(dir.path().join("g.yaml"), "nodes: {not: a list}\n").expect("write graph");
    let manifest = dir.path().join("rcp.yaml");
    std::fs::write(&manifest, "graph: g.yaml\n").expect("write");
    let err = RcpManifest::load(&manifest).expect_err("malformed graph");
    assert!(matches!(err, RcpdError::GraphParse { .. }), "{err:?}");
}
