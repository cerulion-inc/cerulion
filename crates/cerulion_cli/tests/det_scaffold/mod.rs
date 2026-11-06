// SPDX-License-Identifier: AGPL-3.0-only
//! The live determinism oracle's node SCAFFOLD: the graph shapes it splits,
//! the generated node sources (including the read records the oracle judges)
//! and the workspace build. Node TYPES carry a `det_` prefix
//! (`det_relay`, `det_camera`), so the cdylibs this build leaves in the shared
//! target directory never stand in for a node type another test expects to
//! be absent. The read-identity harness (`read_identity/mod.rs`) builds on its
//! record pattern and helpers.
//!
//! `#![allow(dead_code)]`: each binary uses part of it.
#![allow(dead_code)]

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use cerulion_cli_engine::node_cmd::NodeCreateOptions;
use cerulion_cli_engine::node_cmd::NodeLanguage;
use cerulion_cli_engine::{node_cmd, workspace};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::MacroPolicy;

use crate::scaffold_dir::{pin_target_dir, scaffold_target_dir, ScaffoldDir};

/// The graph topic prefix: topics resolve to `/detfree/<node>/<out>`.
pub const GRAPH_PREFIX: &str = "detfree";
/// The message schema every port carries.
pub const SCHEMA: &str = "geometry_msgs/Vector3";

// ===========================================================================
// Shapes and modes.
// ===========================================================================

/// One process-group split of a graph. Declaration order of `groups` is the
/// rank order (`g1` = rank 0).
pub struct Shape {
    /// The graph name, also the YAML file stem and the graph identity.
    pub graph: &'static str,
    pub groups: &'static [(&'static str, &'static [&'static str])],
    /// Which graph the shape splits.
    pub kind: Kind,
    /// Per-edge read delays `(node, input, steps)`, for a spawner that passes
    /// them on. Every shape here declares none.
    pub edge_delays: &'static [(&'static str, &'static str, u64)],
}

/// The graph a shape splits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// `ticker` (1 ms) -> `relay` -> `sink`, every edge a data trigger.
    Chain,
    /// The multi-rate fusion graph: `imu` (1 ms), `camera` (33 ms) and
    /// `lidar` (50 ms) Period sources, and `fusion` fired by every imu frame,
    /// reading camera and lidar as latest values. `slow_lidar` swaps in a
    /// lidar whose tick busy-waits 8 ms.
    MultiRate { slow_lidar: bool },
}

impl Shape {
    pub fn ranks(&self) -> usize {
        self.groups.len()
    }
}

pub const TWO_RANKS: Shape = Shape {
    graph: "detfree_ranks2",
    groups: &[("g1", &["ticker"]), ("g2", &["relay", "sink"])],
    kind: Kind::Chain,
    edge_delays: &[],
};

pub const THREE_RANKS: Shape = Shape {
    graph: "detfree_ranks3",
    groups: &[("g1", &["ticker"]), ("g2", &["relay"]), ("g3", &["sink"])],
    kind: Kind::Chain,
    edge_delays: &[],
};

/// The four multi-rate process groups: one per sensor, one for fusion.
pub const MULTIRATE_GROUPS: &[(&str, &[&str])] = &[
    ("g_imu", &["imu"]),
    ("g_camera", &["camera"]),
    ("g_lidar", &["lidar"]),
    ("g_fusion", &["fusion"]),
];

pub const MULTIRATE: Shape = Shape {
    graph: "detfree_multirate",
    groups: MULTIRATE_GROUPS,
    kind: Kind::MultiRate { slow_lidar: false },
    edge_delays: &[],
};

pub const MULTIRATE_SLOW_LIDAR: Shape = Shape {
    graph: "detfree_multirate_slow_lidar",
    groups: MULTIRATE_GROUPS,
    kind: Kind::MultiRate { slow_lidar: true },
    edge_delays: &[],
};

/// The camera's period, in ms (about 30 Hz).
pub const CAMERA_PERIOD_MS: u64 = 33;
/// The lidar's period, in ms (20 Hz).
pub const LIDAR_PERIOD_MS: u64 = 50;
/// How long the slow lidar's tick busy-waits, in ms.
pub const SLOW_LIDAR_TICK_MS: u64 = 8;
/// The multi-rate hand oracle's window: long enough for several camera and
/// lidar frames.
pub const MULTIRATE_HAND_WINDOW: u64 = 210;
/// The env var the fusion node writes its latency samples under (one file per
/// process, `fusion_<pid>.txt`, one nanosecond value per line).
pub const LATENCY_DIR_ENV: &str = "CER_DET_LATENCY_DIR";
/// The env var the reading nodes record their reads under (one file per
/// node and process, `<node>_<pid>.txt`, one line per fire).
pub const READS_DIR_ENV: &str = "CER_DET_READS_DIR";

// ===========================================================================
// The scaffold: one per caller, removed when its `Scaffold` drops.
// ===========================================================================

/// The scaffolded workspace holding every shape's graph.
pub struct Scaffold {
    /// Owns the directory `ws_root` lives in and removes it on drop.
    pub _dir: ScaffoldDir,
    pub ws_root: PathBuf,
    /// Where the node cdylibs are built; a process that loads them reads it
    /// as `CARGO_TARGET_DIR`.
    pub target_dir: PathBuf,
    /// Graph name -> the config written to `graphs/<name>.yaml`, reused as the
    /// monolith oracle leg's config.
    pub configs: HashMap<&'static str, GraphConfig>,
}

/// The chain's graph config split per `shape`. Serializing this exact struct
/// into the YAML the supervisor parses makes the two parse-compatible by
/// construction.
pub fn build_graph_config(shape: &Shape) -> GraphConfig {
    if let Kind::MultiRate { slow_lidar } = shape.kind {
        return build_multirate_config(shape, slow_lidar);
    }
    let out = |name: &str| OutputDef {
        name: name.to_string(),
        schema: SCHEMA.to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    };
    let input = |source: &str| InputDef {
        name: "inp".to_string(),
        source: source.to_string(),
    };
    GraphConfig {
        level_assignments: None,
        network: None,
        name: None,
        identity: shape.graph.to_string(),
        prefix: GRAPH_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                params: Default::default(),
                id: "ticker".to_string(),
                node_type: "det_ticker".to_string(),
                inputs: vec![],
                outputs: vec![out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                params: Default::default(),
                id: "relay".to_string(),
                node_type: "det_relay".to_string(),
                inputs: vec![input("ticker/out")],
                outputs: vec![out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                params: Default::default(),
                id: "sink".to_string(),
                node_type: "det_sink".to_string(),
                inputs: vec![input("relay/out")],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: vec![],
        // Insertion order defines the ranks (g1 = 0, g2 = 1, g3 = 2).
        process_groups: shape
            .groups
            .iter()
            .map(|(g, members)| {
                (
                    (*g).to_string(),
                    members.iter().map(|m| (*m).to_string()).collect(),
                )
            })
            .collect(),
        process_group_order: vec![],
    }
}

/// The multi-rate graph split per `shape`: three Period sources and the
/// imu-fired fusion node reading camera and lidar as latest values.
pub fn build_multirate_config(shape: &Shape, slow_lidar: bool) -> GraphConfig {
    let out = || OutputDef {
        name: "out".to_string(),
        schema: SCHEMA.to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    };
    let source = |id: &str, node_type: &str| NodeDef {
        fuse: None,
        ros2: None,
        params: Default::default(),
        id: id.to_string(),
        node_type: node_type.to_string(),
        inputs: vec![],
        outputs: vec![out()],
    };
    let input = |name: &str| InputDef {
        name: name.to_string(),
        source: format!("{name}/out"),
    };
    GraphConfig {
        level_assignments: None,
        network: None,
        name: None,
        identity: shape.graph.to_string(),
        prefix: GRAPH_PREFIX.to_string(),
        nodes: vec![
            source("imu", "det_imu"),
            source("camera", "det_camera"),
            source(
                "lidar",
                if slow_lidar {
                    "det_lidar_slow"
                } else {
                    "det_lidar"
                },
            ),
            NodeDef {
                fuse: None,
                ros2: None,
                params: Default::default(),
                id: "fusion".to_string(),
                node_type: "det_fusion".to_string(),
                inputs: vec![input("imu"), input("camera"), input("lidar")],
                outputs: vec![out()],
            },
        ],
        multi_publisher_topics: vec![],
        process_groups: shape
            .groups
            .iter()
            .map(|(g, members)| {
                (
                    (*g).to_string(),
                    members.iter().map(|m| (*m).to_string()).collect(),
                )
            })
            .collect(),
        process_group_order: vec![],
    }
}

/// Replace the scaffold's `// TODO: populate out (...)` line in node
/// `node`'s source with `stmt`, so the node writes (and therefore publishes) its
/// output every tick. Panics when the marker is missing: a changed scaffold
/// would otherwise build a chain that carries nothing.
pub fn write_output(nodes_dir: &Path, node: &str, stmt: &str) {
    write_at_marker(nodes_dir, node, "// TODO: populate out (Vector3)", stmt);
}

/// Replace the scaffold's `marker` line in node `node`'s source with `stmt`,
/// panicking when the marker is not there exactly once.
pub fn write_at_marker(nodes_dir: &Path, node: &str, marker: &str, stmt: &str) {
    let path = nodes_dir.join(node).join("src").join("lib.rs");
    let src = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("DET_HARNESS: read {}: {e}", path.display()));
    assert_eq!(
        src.matches(marker).count(),
        1,
        "DET_HARNESS: {} carries the scaffold's marker `{marker}` exactly once",
        path.display()
    );
    std::fs::write(&path, src.replace(marker, stmt))
        .unwrap_or_else(|e| panic!("DET_HARNESS: write {}: {e}", path.display()));
}

/// The source of one READ RECORD, run in a node's tick: push the `values`
/// (Rust expressions over the node's inputs) onto the node's in-memory read
/// buffer when [`READS_DIR_ENV`] is set (looked up once per process). The
/// buffer is written once, at the node's shutdown ([`install_read_buffer`]),
/// so a fire pays a lock and a push, as the fusion latency buffer does, and no
/// file I/O.
pub fn read_record(values: &[&str]) -> String {
    // The port reads are hoisted into a local first: the node macro's
    // rewriter refuses an input field read inside a macro argument.
    format!(
        "{{ let cer_det_r = [{}]; \
         if *CER_DET_READS_ON.get_or_init(|| std::env::var_os(\"{READS_DIR_ENV}\").is_some()) {{ \
         CER_DET_READS.lock().unwrap_or_else(|p| p.into_inner()).extend_from_slice(&cer_det_r); }} }}",
        values.join(", ")
    )
}

/// Give node type `node` its read buffer: the two statics [`read_record`]
/// pushes to, and a `shutdown` that writes the buffer, `width` values per
/// line, to `$CER_DET_READS_DIR/<record>_<pid>.txt` (`record` is the node's
/// graph id, which the oracle matches against its fires). A process whose
/// instance never fired (the supervisor's planning build loads and tears
/// down every node) writes no file.
pub fn install_read_buffer(nodes_dir: &Path, node: &str, record: &str, width: usize) {
    write_at_marker(
        nodes_dir,
        node,
        "#[cerulion_node_impl]",
        "static CER_DET_READS: std::sync::Mutex<Vec<f64>> = std::sync::Mutex::new(Vec::new());\n\
         static CER_DET_READS_ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();\n\
         #[cerulion_node_impl]",
    );
    write_at_marker(
        nodes_dir,
        node,
        "    fn tick(&mut self) -> Result<(), NodeError> {",
        &format!(
            "    fn shutdown(&mut self) -> Result<(), NodeError> {{ \
             use std::io::Write as _; \
             if let Some(dir) = std::env::var_os(\"{READS_DIR_ENV}\") {{ \
             let buf = std::mem::take(&mut *CER_DET_READS.lock().unwrap_or_else(|p| p.into_inner())); \
             if !buf.is_empty() {{ \
             let mut out = String::new(); \
             for line in buf.chunks({width}) {{ \
             let v: Vec<String> = line.iter().map(|x| x.to_string()).collect(); \
             out.push_str(&v.join(\" \")); out.push('\\n'); }} \
             let path = std::path::Path::new(&dir).join(format!(\"{record}_{{}}.txt\", std::process::id())); \
             if let Ok(mut f) = std::fs::File::create(path) {{ let _ = f.write_all(out.as_bytes()); }} }} }} \
             Ok(()) }}\n    fn tick(&mut self) -> Result<(), NodeError> {{"
        ),
    );
}

/// Scaffold and build the workspace in a new [`ScaffoldDir`]. Every scaffold
/// of one test binary builds into one target directory
/// ([`scaffold_target_dir`], keyed by the binary's crate name), so the
/// dependencies compile once per binary and a later scaffold rebuilds only the
/// node crates and what depends on the workspace's schemas. A
/// `read_identity` workspace of the same binary builds into that directory
/// too; the node type prefixes (`det_`, `ri_`) keep the two sets of cdylibs
/// apart.
pub fn scaffold() -> Scaffold {
    let dir = ScaffoldDir::new("det_ws");
    let ws =
        workspace::workspace_create(dir.path(), "det_ws").expect("DET_HARNESS: workspace_create");
    let target_dir = scaffold_target_dir(&ws.root, env!("CARGO_CRATE_NAME"));
    pin_target_dir(&ws.root, &target_dir);
    let cargo_toml = ws.root.join("Cargo.toml");

    // ticker: pure Period 1ms source (no inputs, one output).
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "det_ticker",
        Some(MacroPolicy::Period { period_ms: 1 }),
        &NodeCreateOptions {
            outputs: vec![(SCHEMA.to_string(), "out".to_string())],
            inputs: vec![],
            trigger: None,
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create ticker");

    // relay: EXPLICIT data-trigger on `inp` (policy plus the matching
    // `#[input(trigger)]` field), not the one-input defaulting warn.
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "det_relay",
        Some(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        }),
        &NodeCreateOptions {
            outputs: vec![(SCHEMA.to_string(), "out".to_string())],
            inputs: vec![(SCHEMA.to_string(), "inp".to_string())],
            trigger: Some("inp".to_string()),
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create relay");

    // sink: EXPLICIT data-trigger on `inp`, no outputs.
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "det_sink",
        Some(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        }),
        &NodeCreateOptions {
            outputs: vec![],
            inputs: vec![(SCHEMA.to_string(), "inp".to_string())],
            trigger: Some("inp".to_string()),
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create sink");

    // Debug builds (the workers' default freshest-debug cdylib resolution).
    // The scaffold leaves every output unwritten (a TODO comment), and an
    // output nobody writes is never published. Make the chain carry data: the
    // ticker publishes its tick count, the relay forwards what it read.
    write_output(
        &ws.nodes_dir,
        "det_ticker",
        "self.out.x = f64::from(self.tick_count);",
    );
    write_output(
        &ws.nodes_dir,
        "det_relay",
        &format!("self.out.x = self.inp.x; {}", read_record(&["self.inp.x"])),
    );
    write_at_marker(
        &ws.nodes_dir,
        "det_sink",
        "// TODO: read from self.inp",
        &read_record(&["self.inp.x"]),
    );
    install_read_buffer(&ws.nodes_dir, "det_relay", "relay", 1);
    install_read_buffer(&ws.nodes_dir, "det_sink", "sink", 1);
    node_cmd::node_build(&ws.root, "det_ticker", false).expect("DET_HARNESS: node_build ticker");
    node_cmd::node_build(&ws.root, "det_relay", false).expect("DET_HARNESS: node_build relay");
    node_cmd::node_build(&ws.root, "det_sink", false).expect("DET_HARNESS: node_build sink");
    scaffold_multirate_nodes(&ws.root, &ws.nodes_dir, &cargo_toml);

    let mut configs = HashMap::new();
    for shape in [&TWO_RANKS, &THREE_RANKS, &MULTIRATE, &MULTIRATE_SLOW_LIDAR] {
        let config = build_graph_config(shape);
        let yaml =
            serde_yaml::to_string(&config).expect("DET_HARNESS: serialize graph config to YAML");
        std::fs::write(ws.graphs_dir.join(format!("{}.yaml", shape.graph)), yaml)
            .expect("DET_HARNESS: write graph YAML");
        configs.insert(shape.graph, config);
    }

    Scaffold {
        _dir: dir,
        ws_root: ws.root,
        target_dir,
        configs,
    }
}

/// The multi-rate node types: `imu`, `camera`, `lidar` and `lidar_slow`
/// Period sources and the `fusion` node, built debug like the chain's.
///
/// * `imu` publishes its tick count in `x` and its sample time in `y`: the
///   node's `real_ns()` (the host's monotonic clock, one origin for every
///   process on the machine; an `f64` holds it exactly).
/// * `camera` and `lidar` publish their tick counts; `lidar_slow` first
///   busy-waits [`SLOW_LIDAR_TICK_MS`].
/// * `fusion` fires on every imu frame, reads camera and lidar as latest
///   values, and records the wall time from the imu sample to its own fire,
///   buffered and appended to `$CER_DET_LATENCY_DIR/fusion_<pid>.txt` every
///   500 samples (the last partial buffer of a run is not written).
pub fn scaffold_multirate_nodes(ws_root: &Path, nodes_dir: &Path, cargo_toml: &Path) {
    let period = |name: &str, period_ms: u64| {
        node_cmd::node_create_with_options(
            nodes_dir,
            cargo_toml,
            name,
            Some(MacroPolicy::Period { period_ms }),
            &NodeCreateOptions {
                outputs: vec![(SCHEMA.to_string(), "out".to_string())],
                inputs: vec![],
                trigger: None,
                raw_ffi: false,
                language: NodeLanguage::Rust,
            },
        )
        .unwrap_or_else(|e| panic!("DET_HARNESS: node_create {name}: {e}"));
    };
    period("det_imu", 1);
    period("det_camera", CAMERA_PERIOD_MS);
    period("det_lidar", LIDAR_PERIOD_MS);
    period("det_lidar_slow", LIDAR_PERIOD_MS);
    node_cmd::node_create_with_options(
        nodes_dir,
        cargo_toml,
        "det_fusion",
        Some(MacroPolicy::DataTrigger {
            input_name: "imu".to_string(),
        }),
        &NodeCreateOptions {
            outputs: vec![(SCHEMA.to_string(), "out".to_string())],
            inputs: vec![
                (SCHEMA.to_string(), "imu".to_string()),
                (SCHEMA.to_string(), "camera".to_string()),
                (SCHEMA.to_string(), "lidar".to_string()),
            ],
            trigger: Some("imu".to_string()),
            raw_ffi: false,
            language: NodeLanguage::Rust,
        },
    )
    .expect("DET_HARNESS: node_create fusion");
    write_output(
        nodes_dir,
        "det_imu",
        "self.out.x = f64::from(self.tick_count); self.out.y = self.real_ns() as f64;",
    );
    write_output(
        nodes_dir,
        "det_camera",
        "self.out.x = f64::from(self.tick_count);",
    );
    write_output(
        nodes_dir,
        "det_lidar",
        "self.out.x = f64::from(self.tick_count);",
    );
    write_output(
        nodes_dir,
        "det_lidar_slow",
        &format!(
            "let t0 = self.real_ns(); \
             while self.real_ns().saturating_sub(t0) < {SLOW_LIDAR_TICK_MS} * 1_000_000 {{ \
             std::hint::spin_loop(); }} \
             self.out.x = f64::from(self.tick_count);"
        ),
    );
    write_output(
        nodes_dir,
        "det_fusion",
        &format!(
            "{{ use std::io::Write as _; \
             static SAMPLES: std::sync::Mutex<Vec<i64>> = std::sync::Mutex::new(Vec::new()); \
             let lat = self.real_ns() as i64 - self.imu.y as i64; \
             self.out.x = self.camera.x + self.lidar.x; \
             self.out.y = lat as f64; \
             let mut buf = SAMPLES.lock().unwrap_or_else(|p| p.into_inner()); \
             buf.push(lat); \
             if buf.len() >= 500 {{ \
             if let Some(dir) = std::env::var_os(\"{LATENCY_DIR_ENV}\") {{ \
             let path = std::path::Path::new(&dir) \
             .join(format!(\"fusion_{{}}.txt\", std::process::id())); \
             if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(path) {{ \
             for v in buf.iter() {{ let _ = writeln!(f, \"{{v}}\"); }} }} }} \
             buf.clear(); }} }} {}",
            read_record(&["self.imu.x", "self.camera.x", "self.lidar.x"])
        ),
    );
    install_read_buffer(nodes_dir, "det_fusion", "fusion", 3);
    for name in [
        "det_imu",
        "det_camera",
        "det_lidar",
        "det_lidar_slow",
        "det_fusion",
    ] {
        node_cmd::node_build(ws_root, name, false)
            .unwrap_or_else(|e| panic!("DET_HARNESS: node_build {name}: {e}"));
    }
}

/// Every node's recorded reads, one string per fire, in fire order.
pub type Reads = HashMap<String, Vec<String>>;

/// Read every `<node>_<pid>.txt` record under `dir`. An EMPTY file (a process
/// whose instance of the node never fired) is skipped; a node with two
/// non-empty files, read by two processes, is a harness failure: a node runs
/// in one process per run.
pub fn read_records(dir: &Path) -> Reads {
    let mut out: Reads = HashMap::new();
    let Ok(entries) = std::fs::read_dir(dir) else {
        return out;
    };
    let mut paths: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
    paths.sort();
    for path in paths {
        let Some(name) = path.file_name().map(|n| n.to_string_lossy().into_owned()) else {
            continue;
        };
        let Some((node, _pid)) = name.trim_end_matches(".txt").rsplit_once('_') else {
            continue;
        };
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|err| panic!("DET_HARNESS: read {}: {err}", path.display()));
        if text.is_empty() {
            continue;
        }
        assert!(
            out.insert(node.to_string(), text.lines().map(str::to_string).collect())
                .is_none(),
            "DET_HARNESS: node {node} recorded non-empty reads from two processes under {}",
            dir.display()
        );
    }
    out
}

/// A node's first read that differs from the monolith's: its index into the
/// node's reads and each side's read there (`None` past its end).
#[derive(Debug, PartialEq, Eq)]
pub struct ReadDiff {
    pub node: String,
    pub index: usize,
    pub got: Option<String>,
    pub want: Option<String>,
}

/// How many fires each node has in `fires` (node ids, one per fire).
pub fn fires_per_node<'a>(fires: impl IntoIterator<Item = &'a str>) -> HashMap<String, usize> {
    let mut out = HashMap::new();
    for node in fires {
        *out.entry(node.to_string()).or_insert(0) += 1;
    }
    out
}

/// The first read, over every node the monolith recorded (in name order),
/// where a split run's reads differ from the monolith's within the judged
/// prefix, or `None`.
///
/// A node records one read per TICK, and a fire whose tick did not run (a
/// node with a latest-value input that has received nothing yet fires
/// without ticking) records none; such fires come first, before the node's
/// inputs are all present. So the reads a split run must match are the node's
/// fires in the judged prefix (`prefix_fires`) less the fires the monolith
/// made without a read (its whole run's fires, `monolith_fires`, less its
/// reads). Those first `n` reads are compared; a split run that recorded fewer
/// than `n` lost a read inside the prefix and differs at the first missing
/// one.
pub fn first_read_difference(
    prefix_fires: &HashMap<String, usize>,
    monolith_fires: &HashMap<String, usize>,
    live: &Reads,
    monolith: &Reads,
) -> Option<ReadDiff> {
    let mut nodes: Vec<&String> = monolith.keys().collect();
    nodes.sort();
    for node in nodes {
        let want = &monolith[node];
        let fired = monolith_fires.get(node).copied().unwrap_or(0);
        assert!(
            want.len() <= fired,
            "DET_HARNESS: node {node} recorded {} reads in the monolith but fired {fired} times; \
             a node records at most one read per fire",
            want.len()
        );
        let tickless = fired - want.len();
        let n = prefix_fires
            .get(node)
            .copied()
            .unwrap_or(0)
            .saturating_sub(tickless);
        let got = live.get(node).map(Vec::as_slice).unwrap_or_default();
        if let Some(index) = (0..n).find(|&i| got.get(i) != want.get(i)) {
            return Some(ReadDiff {
                node: node.clone(),
                index,
                got: got.get(index).cloned(),
                want: want.get(index).cloned(),
            });
        }
    }
    None
}
