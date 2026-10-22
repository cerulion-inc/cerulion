// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph validate` reads each node library ONCE.
//!
//! Two checks read a library through the caller's inspector: the network
//! ingress gate (per node) and the params check (per type). A library's
//! static initialization and teardown run on every in-process read, so a
//! second read is a second run of them. This drives the real validate entry
//! point over a two-node graph of one type with an ingress block and counts
//! the reads with an inspector that never loads anything.
//!
//! ```bash
//! cargo test -p cerulion_cli_engine --test graph_validate_inspect_once_test
//! ```

use std::path::Path;
use std::sync::Mutex;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::graph_cmd::{
    cdylib_name, cdylib_target_base, graph_validate_with_inspector,
};
use cerulion_cli_engine::node_inspector::NodeInspector;
use cerulion_core::graph::NodeInfo;

/// Records every path it is asked to read and answers with an info that
/// declares one param, `gain: f64`, so the params check has a line to write
/// for every node of the type.
struct Counting {
    reads: Mutex<Vec<String>>,
}

impl NodeInspector for Counting {
    fn inspect(&self, cdylib: &Path, _label: &str) -> CliResult<NodeInfo> {
        use cerulion_core::graph::params::{ParamSpec, ParamType, ParamValue};
        self.reads
            .lock()
            .unwrap()
            .push(cdylib.display().to_string());
        Ok(NodeInfo::default().with_params(vec![ParamSpec {
            name: "gain".into(),
            ty: ParamType::F64,
            default: ParamValue::Float(1.0),
            min: None,
            max: None,
            description: None,
        }]))
    }
}

const GRAPH: &str = "\
prefix: once
nodes:
  - id: left
    type: cam
  - id: right
    type: cam
network:
  mode: peer
  listen:
    - tcp/127.0.0.1:0
  ingress:
    - /once/incoming
";

#[test]
fn validate_reads_each_node_library_once_across_its_checks() {
    let tmp = tempfile::tempdir().unwrap();
    let root = tmp.path();
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::write(root.join("graphs/g.yaml"), GRAPH).unwrap();
    std::fs::create_dir_all(root.join("nodes/cam/src")).unwrap();
    std::fs::write(root.join("nodes/cam/src/lib.rs"), "").unwrap();
    // The cdylib check only resolves a path; the counting inspector never
    // loads it, so an empty file is a library as far as validate can tell.
    let lib = cdylib_target_base(root)
        .join("debug")
        .join(cdylib_name("cam"));
    std::fs::create_dir_all(lib.parent().unwrap()).unwrap();
    std::fs::write(&lib, b"").unwrap();

    let counting = Counting {
        reads: Mutex::new(Vec::new()),
    };
    let report = graph_validate_with_inspector(root, "g", false, &counting).expect("validate");
    // ANTI-VACUITY: both reading checks ran (the ingress line and one params
    // line per node), so a single read is a shared
    // one and not a skipped check.
    let labels: Vec<&str> = report.checks.iter().map(|c| c.label.as_str()).collect();
    let ingress: Vec<_> = report
        .checks
        .iter()
        .filter(|c| c.label == "network ingress")
        .collect();
    assert_eq!(ingress.len(), 1, "{labels:?}");
    // The gate got node infos and judged them (no input consumes the topic,
    // so it fails), rather than failing to read the libraries at all.
    assert!(
        !ingress[0].detail.contains("could not load node libraries"),
        "the ingress gate must have read the library: {}",
        ingress[0].detail
    );
    for node in ["left", "right"] {
        let label = format!("params '{node}'");
        let lines: Vec<_> = report.checks.iter().filter(|c| c.label == label).collect();
        assert_eq!(lines.len(), 1, "exactly one {label} line: {labels:?}");
        assert!(lines[0].passed, "{label}: {}", lines[0].detail);
        assert_eq!(lines[0].detail, "gain=1.0", "{label}");
    }
    let reads = counting.reads.lock().unwrap().clone();
    assert_eq!(
        reads,
        vec![lib.display().to_string()],
        "one read for two nodes of one type across both checks"
    );
}
