// SPDX-License-Identifier: AGPL-3.0-only
//! The `rcp.yaml` manifest: parse, resolve the graph, refuse bad input.

use cerulion_rcpd::manifest::{
    DeclaredNode, EstopClear, IrreversibleTools, Limits, RcpManifest, RobotIdentity, SafetyClass,
};
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
    let err = RcpManifest::parse("robot: go2\nskills: []\n").expect_err("unknown key");
    assert!(err.contains("skills"), "{err}");
}

const STEP_TOOL: &str = "
tools:
  - name: arm.step
    description: one period
    safety_class: reversible
    input_schema:
      type: object
      properties:
        q: { type: array, items: { type: number }, minItems: 6, maxItems: 6 }
      required: [q]
";

#[test]
fn tools_state_and_journal_parse_with_defaults() {
    let m = RcpManifest::parse(&format!(
        "{STEP_TOOL}state:\n  - name: arm\n    topic: /arm/state\n"
    ))
    .expect("parse");
    assert_eq!(m.tools.len(), 1);
    assert_eq!(m.tools[0].timeout_ms, 2000);
    assert!(m.tools[0].safety_class.is_mutating());
    assert_eq!(m.journal.calls_topic, "/rcp/calls");
    assert_eq!(m.journal.results_topic, "/rcp/results");
    assert_eq!(m.state[0].name, "arm");
}

#[test]
fn a_tool_surface_the_daemon_cannot_serve_is_refused() {
    let cases = [
        (STEP_TOOL.replace("arm.step", "estop"), "built in"),
        (STEP_TOOL.replace("arm.step", "estop_clear"), "built in"),
        (
            format!("{STEP_TOOL}{}", &STEP_TOOL["\ntools:\n".len()..]),
            "declared twice",
        ),
        (
            format!("{STEP_TOOL}limits:\n  estop_clear: nobody\n"),
            "nobody",
        ),
        (
            format!("{STEP_TOOL}limits:\n  estop: agent\n"),
            "unknown field `estop`",
        ),
        (STEP_TOOL.replace("minItems: 6", "pattern: x"), "pattern"),
        (STEP_TOOL.replace("safety_class", "safety"), "safety"),
        (
            format!("{STEP_TOOL}journal:\n  calls_topic: rcp/calls\n"),
            "absolute",
        ),
        (
            format!("{STEP_TOOL}state:\n  - name: a/b\n    topic: /s\n"),
            "state name",
        ),
        (
            format!("{STEP_TOOL}state:\n  - name: s\n    topic: /rcp/calls\n"),
            "journal topic",
        ),
        (
            format!("{STEP_TOOL}journal:\n  calls_topic: /x\n  results_topic: /x\n"),
            "must differ",
        ),
        (STEP_TOOL.replace("arm.step", "arm step"), "tool name"),
    ];
    for (yaml, expected) in cases {
        let err = RcpManifest::parse(&yaml).expect_err(expected);
        assert!(err.contains(expected), "{expected}: {err}");
    }
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

#[test]
fn a_blank_robot_is_refused() {
    for yaml in ["robot: \"\"\n", "robot: \"   \"\n"] {
        let err = cerulion_rcpd::manifest::RcpManifest::parse(yaml).expect_err(yaml);
        assert!(err.contains("robot"), "{err}");
    }
}

#[test]
fn limits_default_to_agent_clearing_and_allowed_irreversible_tools() {
    let defaults = Limits {
        estop_clear: EstopClear::Agent,
        irreversible: IrreversibleTools::Allow,
    };
    assert_eq!(Limits::default(), defaults);
    assert_eq!(
        RcpManifest::parse(STEP_TOOL).expect("parse").limits,
        defaults
    );
    assert_eq!(
        RcpManifest::parse("limits: {}\n").expect("parse").limits,
        defaults
    );
    let m = RcpManifest::parse("limits:\n  estop_clear: operator\n  irreversible: refuse\n")
        .expect("parse");
    assert_eq!(
        m.limits,
        Limits {
            estop_clear: EstopClear::Operator,
            irreversible: IrreversibleTools::Refuse,
        }
    );
}

const CUT_TOOL: &str = "
tools:
  - name: arm.cut
    description: cut the cable
    safety_class: irreversible
    input_schema: { type: object, properties: {} }
";

#[test]
fn an_irreversible_tool_loads_by_default() {
    for extra in ["", "limits:\n  irreversible: allow\n"] {
        let m = RcpManifest::parse(&format!("{CUT_TOOL}{extra}")).expect(extra);
        assert_eq!(m.tools[0].safety_class, SafetyClass::Irreversible);
        assert!(m.tools[0].safety_class.is_mutating());
    }
}

#[test]
fn the_refuse_switch_fails_the_load_naming_the_tool() {
    let dir = tempfile::tempdir().expect("tempdir");
    let manifest = dir.path().join("rcp.yaml");
    std::fs::write(
        &manifest,
        format!(
            "{STEP_TOOL}{}limits:\n  irreversible: refuse\n",
            &CUT_TOOL["\ntools:\n".len()..]
        ),
    )
    .expect("write");
    let err = RcpManifest::load_full(&manifest).expect_err("refused");
    let RcpdError::ManifestParse { reason, .. } = &err else {
        panic!("a parse error: {err:?}");
    };
    assert_eq!(
        reason,
        "tool \"arm.cut\" is irreversible and `limits.irreversible` is `refuse`"
    );
    // The switch refuses only irreversible tools.
    std::fs::write(
        &manifest,
        format!("{STEP_TOOL}limits:\n  irreversible: refuse\n"),
    )
    .expect("write");
    RcpManifest::load_full(&manifest).expect("a reversible tool still loads");
}
