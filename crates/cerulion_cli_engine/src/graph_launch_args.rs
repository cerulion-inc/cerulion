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
//! author's document, `args:` and references included, and so is the
//! parse's record of which argument fed which `params:` value, so a param
//! refusal names the argument.
//!
//! A partition saved into the graph file (`graph run --yes`) must hold for
//! the graph as declared: [`overrides_change_wiring`] tells `graph run`
//! whether this run's overrides moved the prefix, a topic or a source, and
//! the save is refused when they did (the run itself is not).

use std::path::Path;

use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::launch_args::{parse_override, ArgProvenance, LaunchOverride};

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
/// `overrides`, and return the exact bytes read and the argument provenance
/// (which argument fed which `params:` value) beside the config.
///
/// This is `graph_cmd::graph_read_raw` with overrides: the file-stem
/// identity is adopted the same way, and a graph with no `args:` resolves to
/// itself. An override for a graph that declares no arguments is refused
/// like any other unknown override.
pub fn graph_read_raw_with_launch_args(
    graphs_dir: &Path,
    name: &str,
    overrides: &[LaunchOverride],
) -> CliResult<(GraphConfig, String, ArgProvenance)> {
    let graph_path = graphs_dir.join(format!("{name}.yaml"));
    if !graph_path.exists() {
        return Err(CliError::GraphNotFound {
            name: name.to_string(),
        });
    }
    let yaml = std::fs::read_to_string(&graph_path)?;
    let (mut config, provenance) =
        cerulion_core::graph::parse_graph_raw_with_launch_args(&yaml, overrides)?;
    cerulion_core::graph::adopt_file_stem_identity(&mut config, name);
    Ok((config, yaml, provenance))
}

/// Whether `resolved` (the graph `raw` parsed under this run's overrides)
/// differs from the graph's declared defaults in its wiring: the `prefix`,
/// an output's `topic` or an input's `source`. `params:` values do not
/// count. `resolved` must be the parse as read, before any prefix default is
/// filled in, or a prefix-less file would always differ.
pub fn overrides_change_wiring(raw: &str, resolved: &GraphConfig) -> CliResult<bool> {
    let (defaults, _) = cerulion_core::graph::parse_graph_raw_with_launch_args(raw, &[])?;
    Ok(wiring_of(&defaults) != wiring_of(resolved))
}

/// The wiring of a graph as comparable lines: the prefix, then each node's
/// output topics and input sources in declaration order.
fn wiring_of(config: &GraphConfig) -> Vec<String> {
    let mut lines = vec![format!("prefix={}", config.prefix)];
    for node in &config.nodes {
        for output in &node.outputs {
            lines.push(format!(
                "{} out {}={}",
                node.id,
                output.name,
                output.topic.as_deref().unwrap_or("")
            ));
        }
        for input in &node.inputs {
            lines.push(format!("{} in {}={}", node.id, input.name, input.source));
        }
    }
    lines
}

/// The refusal `graph run --yes` gives when this run's overrides change the
/// graph's wiring: the derived partition holds for this run only, so it is
/// not saved into the file. The run is not refused; only the save is.
pub fn refuse_persist_under_wiring_overrides(graph: &str) -> CliError {
    CliError::Validation(format!(
        "graph '{graph}': this run's launch-argument overrides change its wiring (the prefix, \
         an output's topic or an input's source), so the partition derived for this run is \
         not saved into the graph file with --yes; drop --yes to run the derived groups in \
         memory, or save a partition for the declared defaults with `cerulion graph partition \
         {graph}`"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::graph::ParamValue;

    const ROBOT: &str = "args:\n  ns: lab\n  side: left\n  cam_gain: 2.5\nprefix: ${ns}\nnodes:\n  - id: cam\n    type: cam\n    params:\n      gain: ${cam_gain}\n      frame_id: ${side}_cam\n    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n        topic: /${ns}/${side}/cmd\n";

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
        let overrides = parse_overrides(&["side:=right".to_string(), "cam_gain:=0.5".to_string()])
            .expect("parses");
        let (config, raw, provenance) =
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
        assert_eq!(config.nodes[0].params["gain"], ParamValue::Float(0.5));
        assert_eq!(
            raw, ROBOT,
            "the bytes returned are the file's, references included"
        );
        // The provenance names the argument behind the typed param value
        // and the override that set it.
        assert_eq!(
            provenance.describe("cam", "gain").as_deref(),
            Some("from launch argument 'cam_gain', set by override `cam_gain:=0.5`")
        );
        assert_eq!(provenance.describe("cam", "frame_id"), None);
    }

    #[test]
    fn overrides_that_move_the_wiring_are_told_apart_from_ones_that_feed_params() {
        let resolved = |words: &[&str]| {
            let overrides =
                parse_overrides(&words.iter().map(|w| w.to_string()).collect::<Vec<_>>())
                    .expect("parses");
            cerulion_core::graph::parse_graph_raw_with_launch_args(ROBOT, &overrides)
                .expect("resolves")
                .0
        };
        assert!(!overrides_change_wiring(ROBOT, &resolved(&[])).unwrap());
        assert!(!overrides_change_wiring(ROBOT, &resolved(&["cam_gain:=0.5"])).unwrap());
        assert!(overrides_change_wiring(ROBOT, &resolved(&["side:=right"])).unwrap());
        assert!(overrides_change_wiring(ROBOT, &resolved(&["ns:=other"])).unwrap());
        // A file with no `prefix:` line compares equal to itself as read.
        let no_prefix = ROBOT.replace("prefix: ${ns}\n", "");
        let (as_read, _) =
            cerulion_core::graph::parse_graph_raw_with_launch_args(&no_prefix, &[]).unwrap();
        assert!(!overrides_change_wiring(&no_prefix, &as_read).unwrap());
        let msg = refuse_persist_under_wiring_overrides("robot").to_string();
        assert!(
            msg.contains("graph 'robot': this run's launch-argument overrides change its wiring")
                && msg.contains("drop --yes")
                && msg.contains("`cerulion graph partition robot`"),
            "{msg}"
        );
    }

    #[test]
    fn without_overrides_the_read_matches_the_plain_graph_read() {
        let ws = workspace_with(ROBOT);
        let graphs = ws.path().join("graphs");
        let (with, _, provenance) =
            graph_read_raw_with_launch_args(&graphs, "robot", &[]).expect("reads");
        assert_eq!(
            provenance.describe("cam", "gain").as_deref(),
            Some("from launch argument 'cam_gain', its declared default")
        );
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
