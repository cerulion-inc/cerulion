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
        // A `ros2:` entry declares no `#[param]` specs; native `params:` on one
        // is the graph topology check's refusal (`validate_graph`), which every
        // run makes, `--no-validate` included.
        if node.is_ros2() {
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

/// Refuse the run when a native node's `params:` do not resolve against what
/// its library declares, with the graph build's own error (`Graph validation
/// error: node '<id>': ...`). A run calls this BEFORE any step that writes a
/// file (the auto-partition pre-flight's `--yes` write), so a run the build
/// would refuse over its params leaves the graph file untouched, with or
/// without `--no-validate`. A library that resolved but will not read is
/// refused here too (the build refuses it the same way); a type whose library
/// did not resolve is left to the cdylib lookup that reports it.
pub fn refuse_unresolvable_params(
    config: &GraphConfig,
    libraries: &IndexMap<String, PathBuf>,
    inspector: &dyn NodeInspector,
) -> crate::error::CliResult<()> {
    let mut infos: IndexMap<String, Result<NodeInfo, String>> = IndexMap::new();
    for node in &config.nodes {
        if node.is_ros2() {
            continue;
        }
        let Some(lib) = libraries.get(&node.node_type) else {
            continue;
        };
        let info = match infos
            .entry(node.node_type.clone())
            .or_insert_with(|| inspect(inspector, lib, &node.node_type))
        {
            Ok(info) => info,
            // The build refuses a library that will not read (one that
            // declares params with no symbol to receive them, a corrupt info
            // document) for every node of its type: refuse it here too, with
            // the read's own words, so it cannot refuse the run AFTER a write.
            Err(reason) => {
                return Err(crate::error::CliError::Validation(format!(
                    "node '{}': the node library could not be read: {reason}",
                    node.id
                )));
            }
        };
        if let Err(problem) = resolve_params(&node.id, info.params(), &node.params) {
            return Err(crate::error::CliError::Transport(problem.into()));
        }
    }
    Ok(())
}

/// A [`NodeInspector`] that loads each library into THIS process and KEEPS
/// it loaded for as long as this value lives. A run holds one for its whole
/// length, so the node load that follows (the planning build, the runtime's
/// factories) opens the library that is already loaded instead of loading it
/// a second time: the library's static initialization runs once per run, and
/// its teardown once, at the end, exactly as without the params read.
#[derive(Default)]
pub struct KeepLoaded {
    held: std::sync::Mutex<Vec<cerulion_core::graph::node::DylibNodeEntry>>,
}

impl KeepLoaded {
    /// How many libraries this value holds loaded.
    pub fn held(&self) -> usize {
        self.held
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .len()
    }
}

impl NodeInspector for KeepLoaded {
    fn inspect(&self, cdylib: &Path, _label: &str) -> crate::error::CliResult<NodeInfo> {
        use cerulion_core::NodeEntry as _;
        let entry = cerulion_core::graph::node::DylibNodeEntry::load(cdylib)?;
        let info = entry.info();
        // Held whether or not the info read: the run loads it again either way.
        self.held
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .push(entry);
        Ok(info?)
    }
}

/// A [`NodeInspector`] that reads each node library through `inner` at most
/// once and answers every later inspection of the same path from that first
/// read. A run hands one of these to both of its params passes (the
/// validation report and [`fill_resolved_params`]), so a library's static
/// initialization runs once for both rather than once per pass, and both
/// passes see the same node info. An inspection that failed replays its
/// message verbatim.
pub struct OncePerLibrary<'a> {
    inner: &'a dyn NodeInspector,
    read: std::sync::Mutex<IndexMap<PathBuf, Result<NodeInfo, String>>>,
}

impl<'a> OncePerLibrary<'a> {
    pub fn new(inner: &'a dyn NodeInspector) -> Self {
        Self {
            inner,
            read: std::sync::Mutex::new(IndexMap::new()),
        }
    }
}

impl NodeInspector for OncePerLibrary<'_> {
    fn inspect(&self, cdylib: &Path, label: &str) -> crate::error::CliResult<NodeInfo> {
        let mut read = self
            .read
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        read.entry(cdylib.to_path_buf())
            .or_insert_with(|| self.inner.inspect(cdylib, label).map_err(|e| e.to_string()))
            .clone()
            .map_err(crate::error::CliError::Validation)
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

    /// Each type resolved to a stand-in path: `Table` answers by label and
    /// never opens the path.
    fn libraries(types: &[&str]) -> IndexMap<String, PathBuf> {
        types
            .iter()
            .map(|t| (t.to_string(), PathBuf::from(t)))
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

    /// The run's two params passes share one `OncePerLibrary`: each library
    /// is read once for both, a failed read replays its message word for
    /// word, and both passes see the same answers as the uncached inspector.
    #[test]
    fn once_per_library_reads_each_library_once_for_both_passes() {
        struct Counting {
            table: Table,
            reads: std::sync::Mutex<Vec<String>>,
        }
        impl NodeInspector for Counting {
            fn inspect(&self, cdylib: &Path, label: &str) -> CliResult<NodeInfo> {
                self.reads
                    .lock()
                    .unwrap()
                    .push(cdylib.display().to_string());
                if label == "broken" {
                    return Err(crate::error::CliError::Validation(
                        "broken: declares params (gain) but exports no symbol".to_string(),
                    ));
                }
                self.table.inspect(cdylib, label)
            }
        }
        let counting = Counting {
            table: table(),
            reads: std::sync::Mutex::new(Vec::new()),
        };
        let once = OncePerLibrary::new(&counting);
        let mut config = parse_graph_raw(GRAPH).unwrap();
        let libs = libraries(&["cam", "ticker"]);
        let render = |checks: Vec<ValidationCheck>| -> Vec<(String, bool, String)> {
            checks
                .into_iter()
                .map(|c| (c.label, c.passed, c.detail))
                .collect()
        };
        assert_eq!(
            render(params_checks(&config, &libs, &once)),
            render(params_checks(&config, &libs, &table())),
            "the cache must not change what the check reports"
        );
        fill_resolved_params(&mut config, &libs, &once);
        assert_eq!(
            render_values(&config.nodes[1].params),
            "gain=1.0, frame_id=\"base_link\""
        );
        assert_eq!(
            *counting.reads.lock().unwrap(),
            vec!["cam".to_string(), "ticker".to_string()],
            "each library is read once across both passes"
        );

        let first = once
            .inspect(Path::new("broken"), "broken")
            .unwrap_err()
            .to_string();
        let second = once
            .inspect(Path::new("broken"), "broken")
            .unwrap_err()
            .to_string();
        assert_eq!(
            first,
            "broken: declares params (gain) but exports no symbol"
        );
        assert_eq!(second, first);
        assert_eq!(
            counting.reads.lock().unwrap().len(),
            3,
            "a failed read is not retried"
        );
    }

    /// The pre-write refusal carries the graph build's exact text, passes a
    /// graph whose params all resolve, refuses a library that will not read,
    /// and leaves an unresolved library to the cdylib lookup.
    #[test]
    fn the_pre_write_refusal_uses_the_build_text_and_passes_a_resolvable_graph() {
        let config = parse_graph_raw(GRAPH).unwrap();
        refuse_unresolvable_params(&config, &libraries(&["cam", "ticker"]), &table())
            .expect("every params block in GRAPH resolves");

        let out_of_range = GRAPH.replace("gain: 2.5", "gain: 11.0");
        let config = parse_graph_raw(&out_of_range).unwrap();
        let err = refuse_unresolvable_params(&config, &libraries(&["cam", "ticker"]), &table())
            .expect_err("gain above its max");
        let infos: IndexMap<String, NodeInfo> = [
            ("left".to_string(), cam_info()),
            ("right".to_string(), cam_info()),
        ]
        .into_iter()
        .collect();
        let build = cerulion_core::graph::params::resolve_graph_params(&config, &infos)
            .expect_err("the build refuses the same graph");
        assert_eq!(err.to_string(), build.to_string());
        assert!(
            err.to_string()
                .starts_with("Graph validation error: node 'left': "),
            "{err}"
        );

        // Not resolved: the cdylib lookup's refusal.
        refuse_unresolvable_params(&config, &libraries(&[]), &table())
            .expect("an unresolved library is the cdylib check's");
        struct Refuses;
        impl NodeInspector for Refuses {
            fn inspect(&self, _cdylib: &Path, _label: &str) -> CliResult<NodeInfo> {
                Err(crate::error::CliError::Validation("unreadable".into()))
            }
        }
        let err = refuse_unresolvable_params(&config, &libraries(&["cam", "ticker"]), &Refuses)
            .expect_err("an unreadable library is refused before any write");
        assert_eq!(
            err.to_string(),
            "node 'left': the node library could not be read: unreadable"
        );
    }

    /// The run's loader keeps every library it read loaded, so the run's own
    /// load of the same path finds it already in the process (no second
    /// static initialization), and the memo above it reads each path once.
    #[cfg(unix)]
    #[test]
    fn keep_loaded_holds_each_library_it_read_for_the_run_to_reuse() {
        let lib = cerulion_core::testing::find_fixture_cdylib("test_node_macro_param_cdylib");
        let loader = KeepLoaded::default();
        let once = OncePerLibrary::new(&loader);
        let info = once.inspect(&lib, "param").expect("the fixture reads");
        let names: Vec<&str> = info.params().iter().map(|p| p.name.as_str()).collect();
        assert!(!names.is_empty(), "the param fixture declares params");
        once.inspect(&lib, "param").expect("the memo answers again");
        assert_eq!(loader.held(), 1, "one library, read and held once");

        // While held, the image is in the process: `RTLD_NOLOAD` opens only
        // an already loaded library, so a non-null handle proves the run's
        // next load of this path reuses it rather than initializing it again.
        let path = std::ffi::CString::new(lib.as_os_str().as_encoded_bytes()).unwrap();
        // SAFETY: `RTLD_NOLOAD` never loads or initializes anything; the
        // handle it returns adds one reference, released right below.
        let handle = unsafe { libc::dlopen(path.as_ptr(), libc::RTLD_NOLOAD | libc::RTLD_LAZY) };
        assert!(
            !handle.is_null(),
            "the held library is loaded in this process"
        );
        // SAFETY: `handle` came from the `dlopen` above and is closed once.
        unsafe { libc::dlclose(handle) };

        let missing = loader.inspect(Path::new("/nonexistent/libnone.dylib"), "none");
        assert!(missing.is_err(), "a library that will not load is an error");
        assert_eq!(loader.held(), 1, "nothing is held for a load that failed");
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

    /// Native `params:` on a `ros2:` entry is the topology check's refusal
    /// (`validate_graph`, which every run makes); the params check adds no
    /// second line for it.
    #[test]
    fn params_on_a_ros2_entry_is_left_to_the_topology_check() {
        let yaml = "prefix: demo\nnodes:\n  - id: talker\n    ros2:\n      package: demo_nodes_cpp\n      executable: talker\n    params: {rate: 2}\n";
        let config = parse_graph_raw(yaml).unwrap();
        let checks = params_checks(&config, &libraries(&[]), &table());
        assert!(
            checks.is_empty(),
            "{:?}",
            checks.iter().map(|c| c.detail.as_str()).collect::<Vec<_>>()
        );
        let err = cerulion_core::graph::validate_graph(&config).expect_err("topology refuses it");
        assert!(
            err.to_string().contains("'talker'") && err.to_string().contains("ros2.params"),
            "{err}"
        );
    }
}
