// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph run <NAME> [NAME:=VALUE]...`: the launch-argument
//! overrides an operator passes beside the graph name, and the graph read
//! that applies them.
//!
//! A graph declares its arguments under a top-level `args:` map with a
//! default each; an override replaces one default for this run. The
//! resolution and the `${name}` substitution are
//! [`cerulion_core::graph::launch_args`]'s pure functions; this module is the
//! CLI's thin edge over them: it turns the command-line words into typed
//! overrides and reads the graph file with them applied, so `graph run` sees
//! the same resolved document every other verb sees (with defaults), plus
//! the operator's choices.
//!
//! Every mistake is refused before any node library loads: a malformed
//! `name:=value`, an override for an argument the graph does not declare
//! (with the nearest declared name), and every reference problem the core
//! resolution names. The file's bytes are returned untouched beside the
//! resolved config, so the auto-partition splice keeps writing into the
//! author's document, `args:` and references included.

use std::path::Path;

use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::launch_args::{parse_override, LaunchOverride};

use crate::error::{CliError, CliResult};

/// The clap value parser of the `NAME:=VALUE` positional: a word that is not
/// a well-formed override is a usage error at parse, before any handler is
/// installed or any file read. The word itself is kept; [`parse_overrides`]
/// turns the accepted words into typed overrides. Checking at parse keeps
/// the `--record` contract: a value after a bare `--record` is a positional,
/// and a positional that is no override is refused right there instead of
/// becoming a silent `--record <value>` form.
pub fn check_override_word(word: &str) -> Result<String, String> {
    parse_override(word)
        .map(|_| word.to_string())
        .map_err(|problem| {
            format!(
                "{problem}; a flag that takes a value is written with `=`, for example \
                 `--record=DIR`"
            )
        })
}

/// Parse every `name:=value` word into a typed override. The first malformed
/// word refuses the whole list, naming the word and the expected shape.
pub fn parse_overrides(words: &[String]) -> CliResult<Vec<LaunchOverride>> {
    words
        .iter()
        .map(|word| {
            parse_override(word).map_err(|problem| CliError::Validation(problem.to_string()))
        })
        .collect()
}

/// Read `graphs/<name>.yaml` with its launch arguments resolved under
/// `overrides`, and return the exact bytes read beside the config.
///
/// This is `graph_cmd::graph_read_raw` with overrides: the file-stem
/// identity is adopted the same way, and a graph with no `args:` resolves to
/// itself. An override for a graph that declares no arguments is refused
/// like any other unknown override.
pub fn graph_read_raw_with_launch_args(
    graphs_dir: &Path,
    name: &str,
    overrides: &[LaunchOverride],
) -> CliResult<(GraphConfig, String)> {
    let graph_path = graphs_dir.join(format!("{name}.yaml"));
    if !graph_path.exists() {
        return Err(CliError::GraphNotFound {
            name: name.to_string(),
        });
    }
    let yaml = std::fs::read_to_string(&graph_path)?;
    let mut config = cerulion_core::graph::parse_graph_raw_with_launch_args(&yaml, overrides)?;
    cerulion_core::graph::adopt_file_stem_identity(&mut config, name);
    Ok((config, yaml))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::graph::ParamValue;

    const ROBOT: &str = "args:\n  ns: lab\n  side: left\nprefix: ${ns}\nnodes:\n  - id: cam\n    type: cam\n    params:\n      frame_id: ${side}_cam\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n        topic: /${ns}/${side}/cmd\n";

    fn workspace_with(graph_yaml: &str) -> tempfile::TempDir {
        let dir = tempfile::tempdir().expect("tempdir");
        let graphs = dir.path().join("graphs");
        std::fs::create_dir_all(&graphs).expect("graphs dir");
        std::fs::write(graphs.join("robot.yaml"), graph_yaml).expect("write graph");
        dir
    }

    #[test]
    fn the_read_applies_overrides_adopts_the_stem_and_returns_the_raw_bytes() {
        let ws = workspace_with(ROBOT);
        let overrides = parse_overrides(&["side:=right".to_string()]).expect("parses");
        let (config, raw) =
            graph_read_raw_with_launch_args(&ws.path().join("graphs"), "robot", &overrides)
                .expect("the graph reads");
        assert_eq!(config.identity(), "robot");
        assert_eq!(config.prefix, "lab");
        assert_eq!(
            config.nodes[0].outputs[0].topic.as_deref(),
            Some("/lab/right/cmd")
        );
        assert_eq!(
            config.nodes[0].params["frame_id"],
            ParamValue::Str("right_cam".to_string())
        );
        assert_eq!(
            raw, ROBOT,
            "the bytes returned are the file's, references included"
        );
    }

    #[test]
    fn without_overrides_the_read_matches_the_plain_graph_read() {
        let ws = workspace_with(ROBOT);
        let graphs = ws.path().join("graphs");
        let (with, _) = graph_read_raw_with_launch_args(&graphs, "robot", &[]).expect("reads");
        let (plain, _) = crate::graph_cmd::graph_read_raw(&graphs, "robot").expect("reads");
        assert_eq!(
            serde_yaml::to_string(&with).unwrap(),
            serde_yaml::to_string(&plain).unwrap()
        );
        assert_eq!(
            with.nodes[0].outputs[0].topic.as_deref(),
            Some("/lab/left/cmd")
        );
    }

    #[test]
    fn a_missing_graph_is_the_same_not_found_error() {
        let ws = workspace_with(ROBOT);
        let err = graph_read_raw_with_launch_args(&ws.path().join("graphs"), "nope", &[])
            .expect_err("missing");
        assert!(matches!(err, CliError::GraphNotFound { name } if name == "nope"));
    }

    #[test]
    fn an_unknown_override_is_refused_naming_the_nearest_declared_argument() {
        let ws = workspace_with(ROBOT);
        let overrides = parse_overrides(&["sdie:=right".to_string()]).expect("parses");
        let msg = graph_read_raw_with_launch_args(&ws.path().join("graphs"), "robot", &overrides)
            .expect_err("refused")
            .to_string();
        assert!(
            msg.contains(
                "launch argument 'sdie' is not declared by this graph (did you mean 'side'?); \
                 declared args: ns, side"
            ),
            "{msg}"
        );
    }

    #[test]
    fn a_malformed_word_is_refused_before_any_file_is_read() {
        let msg = parse_overrides(&["side=right".to_string()])
            .expect_err("refused")
            .to_string();
        assert!(
            msg.starts_with("launch argument override 'side=right' is malformed: ")
                && msg.contains("write `name:=value`"),
            "{msg}"
        );
    }
}
