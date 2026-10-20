// SPDX-License-Identifier: AGPL-3.0-only
//! The `params` check of `cerulion graph validate`: every node's `params:`
//! resolved against the specs its node library declares, before any node
//! runs.
//!
//! One check line per node that sets `params:` or whose type declares a
//! `#[param]`, labelled `params '<node id>'`. A passing line lists the values
//! in force (the graph's value or the macro default); a failing line carries
//! the same text the graph build refuses with (an unknown name with the
//! nearest declared name, a value of the wrong type, a value outside the
//! bounds), so `graph validate` and `graph run` disagree about nothing. The
//! graph build re-runs the same resolution, which is why a failing line here
//! blocks the run even under `--no-validate`. A library that resolved but
//! whose node info cannot be read (a library that declares params it has no
//! symbol to receive, a corrupt info document) fails the line of EVERY node
//! of that type: the build refuses the library the same way, `params:` or not.
//!
//! The node info is read through the caller's [`NodeInspector`] (in this
//! process for the one-shot CLI, in a child for a standing daemon), once per
//! node type.
//!
//! [`fill_resolved_params`] is the run's twin of the check: it completes each
//! node's `params:` to the resolved map, so the effective graph a run embeds
//! (its run directory's and its recording's `graph.yaml`) carries the values
//! in force, defaults included.

use std::path::{Path, PathBuf};

use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::params::{resolve_params, ParamValues};
use cerulion_core::graph::NodeInfo;
use indexmap::IndexMap;

use crate::graph_cmd::ValidationCheck;
use crate::node_inspector::NodeInspector;

/// Label prefix of the per-node params check: `params '<node id>'`.
pub const PARAMS_CHECK_PREFIX: &str = "params '";

fn label(node_id: &str) -> String {
    format!("{PARAMS_CHECK_PREFIX}{node_id}'")
}

/// The values in force, as a graph author would write them: `gain=2.5,
/// frame_id="left_cam"`. Empty for a node with no params.
fn render_values(values: &ParamValues) -> String {
    if values.is_empty() {
        return "no params".to_string();
    }
    values
        .iter()
        .map(|(name, value)| format!("{name}={value}"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Build the params checks for `config`. `libraries` maps each native node
/// TYPE that resolved to its cdylib path; a type absent from it did not
/// resolve, and its cdylib check already says so, so a node of that type
/// gets a `not checked` failure only when it sets `params:` (the user must
/// learn those values were never looked at).
pub fn params_checks(
    config: &GraphConfig,
    libraries: &IndexMap<String, PathBuf>,
    inspector: &dyn NodeInspector,
) -> Vec<ValidationCheck> {
    let mut infos: IndexMap<String, Result<NodeInfo, String>> = IndexMap::new();
    let mut checks = Vec::new();
    for node in &config.nodes {
        if node.is_ros2() {
            if !node.params.is_empty() {
                checks.push(ValidationCheck::fail(
                    label(&node.id),
                    "`params:` is not accepted on a `ros2:` entry; put ROS 2 parameters under \
                     `ros2.params` on that entry",
                ));
            }
            continue;
        }
        let Some(lib) = libraries.get(&node.node_type) else {
            if !node.params.is_empty() {
                checks.push(ValidationCheck::fail(
                    label(&node.id),
                    format!(
                        "not checked: the node library for type '{}' did not resolve (see the \
                         cdylib check)",
                        node.node_type
                    ),
                ));
            }
            continue;
        };
        let info = infos
            .entry(node.node_type.clone())
            .or_insert_with(|| inspect(inspector, lib, &node.node_type));
        match info {
            // The library resolved but will not read: the graph build refuses
            // it for every node of the type, so every node gets the line.
            Err(reason) => checks.push(ValidationCheck::fail(
                label(&node.id),
                format!("not checked: the node library could not be read: {reason}"),
            )),
            Ok(info) => {
                if node.params.is_empty() && info.params().is_empty() {
                    continue;
                }
                match resolve_params(&node.id, info.params(), &node.params) {
                    Ok(values) => checks.push(ValidationCheck::pass(
                        label(&node.id),
                        render_values(&values),
                    )),
                    Err(problem) => {
                        checks.push(ValidationCheck::fail(label(&node.id), problem.to_string()))
                    }
                }
            }
        }
    }
    checks
}

fn inspect(inspector: &dyn NodeInspector, lib: &Path, node_type: &str) -> Result<NodeInfo, String> {
    inspector.inspect(lib, node_type).map_err(|e| e.to_string())
}

/// Complete each native node's `params:` to its RESOLVED map: the graph's
/// values over the declared defaults, in declaration order, exactly what the
/// graph build hands the node. The effective graph a run embeds then carries
/// the values in force, so a recording shows every value a node ran with and
/// `bag play --resim` applies them even after a node's `#[param]` default
/// changed in the meantime.
///
/// Best effort and never fatal: a node whose library did not resolve or
/// could not be read, or whose values the resolution refuses, keeps its map
/// as written, and the graph build (or `graph validate`) reports the problem
/// in its own words. Resolving an already resolved map is a no-op, so the
/// build's own resolution of the filled config yields the same values.
pub fn fill_resolved_params(
    config: &mut GraphConfig,
    libraries: &IndexMap<String, PathBuf>,
    inspector: &dyn NodeInspector,
) {
    let mut infos: IndexMap<String, Result<NodeInfo, String>> = IndexMap::new();
    for node in &mut config.nodes {
        if node.is_ros2() {
            continue;
        }
        let Some(lib) = libraries.get(&node.node_type) else {
            continue;
        };
        let Ok(info) = infos
            .entry(node.node_type.clone())
            .or_insert_with(|| inspect(inspector, lib, &node.node_type))
        else {
            continue;
        };
        if let Ok(values) = resolve_params(&node.id, info.params(), &node.params) {
            node.params = values;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::CliResult;
    use cerulion_core::graph::params::{ParamSpec, ParamType, ParamValue};
    use cerulion_core::graph::parse_graph_raw;

    /// A stand-in inspector: answers from a fixed table instead of loading a
    /// library, so the check's own logic is exercised without a cdylib.
    struct Table(IndexMap<String, NodeInfo>);

    impl NodeInspector for Table {
        fn inspect(&self, _cdylib: &Path, label: &str) -> CliResult<NodeInfo> {
            self.0
                .get(label)
                .cloned()
                .ok_or_else(|| crate::error::CliError::Validation(format!("no info for {label}")))
        }
    }

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
        ])
    }

    fn libraries(types: &[&str]) -> IndexMap<String, PathBuf> {
        types
            .iter()
            .map(|t| (t.to_string(), PathBuf::from(format!("lib{t}.so"))))
            .collect()
    }

    fn table() -> Table {
        let mut t = IndexMap::new();
        t.insert("cam".to_string(), cam_info());
        t.insert(
            "ticker".to_string(),
            NodeInfo::from_names(vec![], vec!["cmd".into()]),
        );
        Table(t)
    }

    const GRAPH: &str = "\
prefix: demo
nodes:
  - id: left
    type: cam
    params: {gain: 2.5, frame_id: left_cam}
    outputs: [{name: cmd, schema: geometry_msgs/Vector3}]
  - id: right
    type: cam
    outputs: [{name: cmd, schema: geometry_msgs/Vector3}]
  - id: tick
    type: ticker
    outputs: [{name: cmd, schema: geometry_msgs/Vector3}]
";

    #[test]
    fn a_passing_line_lists_the_values_in_force_and_a_type_without_params_gets_no_line() {
        let config = parse_graph_raw(GRAPH).unwrap();
        let checks = params_checks(&config, &libraries(&["cam", "ticker"]), &table());
        let rendered: Vec<(String, bool, String)> = checks
            .iter()
            .map(|c| (c.label.clone(), c.passed, c.detail.clone()))
            .collect();
        assert_eq!(
            rendered,
            vec![
                (
                    "params 'left'".to_string(),
                    true,
                    "gain=2.5, frame_id=\"left_cam\"".to_string()
                ),
                (
                    "params 'right'".to_string(),
                    true,
                    "gain=1.0, frame_id=\"base_link\"".to_string()
                ),
            ]
        );
    }

    #[test]
    fn each_refusal_class_fails_its_node_line_with_the_graph_build_text() {
        for (block, expected) in [
            (
                "params: {gian: 2.5}",
                "node 'left': unknown param 'gian' (did you mean 'gain'?); declared params: gain, frame_id",
            ),
            (
                "params: {gain: 2}",
                "node 'left': param 'gain' expects f64, got integer 2; a float param takes a float value, for example 2.0",
            ),
            (
                "params: {gain: 11.0}",
                "node 'left': param 'gain' = 11.0 is outside [0.0, 10.0]",
            ),
        ] {
            let yaml = format!(
                "prefix: demo\nnodes:\n  - id: left\n    type: cam\n    {block}\n    outputs: [{{name: cmd, schema: geometry_msgs/Vector3}}]\n"
            );
            let config = parse_graph_raw(&yaml).unwrap();
            let checks = params_checks(&config, &libraries(&["cam"]), &table());
            assert_eq!(checks.len(), 1, "{block}");
            assert!(!checks[0].passed, "{block}");
            assert_eq!(checks[0].label, "params 'left'");
            assert_eq!(checks[0].detail, expected);
        }
    }

    #[test]
    fn params_on_a_type_that_declares_none_fails_by_name() {
        let yaml = "prefix: demo\nnodes:\n  - id: tick\n    type: ticker\n    params: {gain: 1.0}\n    outputs: [{name: cmd, schema: geometry_msgs/Vector3}]\n";
        let config = parse_graph_raw(yaml).unwrap();
        let checks = params_checks(&config, &libraries(&["ticker"]), &table());
        assert_eq!(checks.len(), 1);
        assert!(!checks[0].passed);
        assert_eq!(
            checks[0].detail,
            "node 'tick': unknown param 'gain'; this node type declares no params, so `params:` is not accepted on it"
        );
    }

    #[test]
    fn an_unresolved_library_is_reported_only_when_the_node_sets_params() {
        let config = parse_graph_raw(GRAPH).unwrap();
        let checks = params_checks(&config, &libraries(&[]), &table());
        assert_eq!(checks.len(), 1);
        assert_eq!(checks[0].label, "params 'left'");
        assert!(!checks[0].passed);
        assert!(
            checks[0].detail.starts_with("not checked:"),
            "{}",
            checks[0].detail
        );
    }

    /// A library that resolves but will not read is refused by the graph
    /// build for every node of its type, so every node gets the failing line,
    /// `params:` set or not.
    #[test]
    fn an_unreadable_library_fails_every_node_of_its_type() {
        struct Refuses;
        impl NodeInspector for Refuses {
            fn inspect(&self, _cdylib: &Path, label: &str) -> CliResult<NodeInfo> {
                Err(crate::error::CliError::Validation(format!(
                    "{label}: declares params (gain) but exports no symbol"
                )))
            }
        }
        let config = parse_graph_raw(GRAPH).unwrap();
        let checks = params_checks(&config, &libraries(&["cam", "ticker"]), &Refuses);
        let rendered: Vec<(&str, bool)> = checks
            .iter()
            .map(|c| (c.label.as_str(), c.passed))
            .collect();
        assert_eq!(
            rendered,
            vec![
                ("params 'left'", false),
                ("params 'right'", false),
                ("params 'tick'", false),
            ]
        );
        assert_eq!(
            checks[1].detail,
            "not checked: the node library could not be read: cam: declares params (gain) but \
             exports no symbol"
        );
    }

    #[test]
    fn the_fill_completes_each_node_to_its_resolved_map_and_leaves_the_rest_as_written() {
        let mut config = parse_graph_raw(GRAPH).unwrap();
        fill_resolved_params(&mut config, &libraries(&["cam", "ticker"]), &table());
        let render = |config: &GraphConfig| -> Vec<(String, String)> {
            config
                .nodes
                .iter()
                .map(|n| (n.id.clone(), render_values(&n.params)))
                .collect()
        };
        let rendered = render(&config);
        assert_eq!(
            rendered,
            vec![
                (
                    "left".to_string(),
                    "gain=2.5, frame_id=\"left_cam\"".to_string()
                ),
                (
                    "right".to_string(),
                    "gain=1.0, frame_id=\"base_link\"".to_string()
                ),
                ("tick".to_string(), "no params".to_string()),
            ]
        );
        // Filling a filled config changes nothing (the build resolves it again).
        fill_resolved_params(&mut config, &libraries(&["cam", "ticker"]), &table());
        assert_eq!(render(&config), rendered);
        // A refused value, an unresolved library and a `ros2:` entry stay as
        // written; the build says why in its own words.
        let yaml = "prefix: demo\nnodes:\n  - id: left\n    type: cam\n    params: {gain: 11.0}\n    outputs: [{name: cmd, schema: geometry_msgs/Vector3}]\n  - id: orphan\n    type: cam\n    params: {gain: 2.0}\n    outputs: [{name: cmd, schema: geometry_msgs/Vector3}]\n  - id: talker\n    ros2:\n      package: demo_nodes_cpp\n      executable: talker\n";
        let mut config = parse_graph_raw(yaml).unwrap();
        fill_resolved_params(&mut config, &libraries(&[]), &table());
        assert_eq!(
            render_values(&config.nodes[1].params),
            "gain=2.0",
            "no library resolved: nothing changes"
        );
        fill_resolved_params(&mut config, &libraries(&["cam"]), &table());
        assert_eq!(
            render_values(&config.nodes[0].params),
            "gain=11.0",
            "a refused value stays as written for the build to refuse"
        );
        assert_eq!(
            render_values(&config.nodes[1].params),
            "gain=2.0, frame_id=\"base_link\""
        );
        assert!(config.nodes[2].params.is_empty());
    }

    #[test]
    fn params_on_a_ros2_entry_fails_pointing_at_the_ros2_block() {
        let yaml = "prefix: demo\nnodes:\n  - id: talker\n    ros2:\n      package: demo_nodes_cpp\n      executable: talker\n    params: {rate: 2}\n";
        let config = parse_graph_raw(yaml).unwrap();
        let checks = params_checks(&config, &libraries(&[]), &table());
        assert_eq!(checks.len(), 1);
        assert!(!checks[0].passed);
        assert!(
            checks[0].detail.contains("ros2.params"),
            "{}",
            checks[0].detail
        );
    }
}
