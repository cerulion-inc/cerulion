// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion_py_fixture` - the Rust peer for the cerulion_py test suite.
//!
//! Modes: `write-bag` writes a deterministic oracle MCAP bag; `publish`
//! stamps deterministic-pattern wire frames a Python subscriber can
//! oracle-check; `subscribe` receives frames and prints a
//! one-line digest (sequence, header fields, FNV-1a of the body) Python
//! publishers can be asserted against.

// A bin's job is to print: every output line IS the test interface.
#![allow(clippy::print_stdout)]

// One implementation of the aligned-copy helper for the fixture and the
// extension module; its unit tests run here (see `align.rs`).
#[path = "../../src/align.rs"]
mod align;

use cerulion_core::clock::real_ns;
use cerulion_core::clock::RealClock;
use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, MessageSchema};
use cerulion_core::dynamic::{FrameEncoder, FrameView, SchemaSet};
use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry, ShutdownSignal,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::publisher::CerulionPublisher;
use cerulion_core::transport::subscriber::CerulionSubscriber;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{SyncHeadOp, SyncOpAnswer, TransportConfig};
use native_ros2_messages::{geometry_msgs, sensor_msgs};
use std::io::Write;
use std::process::ExitCode;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Deterministic body byte `k` of frame `i` - the Python oracle
/// recomputes the same formula independently.
fn pattern(size: usize, i: u64) -> Vec<u8> {
    (0..size)
        .map(|k| {
            ((k as u64)
                .wrapping_mul(7)
                .wrapping_add(3)
                .wrapping_add(i.wrapping_mul(11))
                & 0xFF) as u8
        })
        .collect()
}

/// FNV-1a 64-bit over `bytes` (offset basis 0xcbf29ce484222325).
fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

fn builtin_schemas() -> Result<Vec<MessageSchema>, String> {
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|&(package, name, text)| {
            parse_rosmsg(text, name, Some(package)).map_err(|e| format!("{package}/{name}: {e}"))
        })
        .collect()
}

fn fixture_schemas(workspace: &std::path::Path) -> Result<SchemaSet, String> {
    SchemaSet::from_workspace_with_builtins(workspace, builtin_schemas()?)
        .map(|(schemas, _warnings)| schemas)
        .map_err(|error| error.to_string())
}

#[derive(Debug)]
struct Args {
    flags: std::collections::HashMap<String, String>,
}

fn parse_args(argv: &[String]) -> Result<Args, String> {
    let mut args = Args {
        flags: std::collections::HashMap::new(),
    };
    let mut it = argv.iter();
    while let Some(a) = it.next() {
        if let Some(name) = a.strip_prefix("--") {
            let value = it
                .next()
                .ok_or_else(|| format!("flag --{name} needs a value"))?;
            args.flags.insert(name.to_string(), value.clone());
        } else {
            return Err(format!("unexpected positional argument '{a}'"));
        }
    }
    Ok(args)
}

fn flag<'a>(args: &'a Args, name: &str) -> Result<&'a str, String> {
    args.flags
        .get(name)
        .map(String::as_str)
        .ok_or_else(|| format!("missing required flag --{name}"))
}

fn flag_u64(args: &Args, name: &str) -> Result<u64, String> {
    flag(args, name)?
        .parse::<u64>()
        .map_err(|e| format!("--{name}: {e}"))
}

fn flag_usize(args: &Args, name: &str) -> Result<usize, String> {
    flag(args, name)?
        .parse::<usize>()
        .map_err(|e| format!("--{name}: {e}"))
}

/// Pure CLI validation: the mode and every flag the mode requires,
/// checked before any transport initialisation so a bad command line
/// never opens shared memory.
fn parse_cli(argv: &[String]) -> Result<(&str, Args), String> {
    let (mode, rest) = argv
        .split_first()
        .map(|(m, r)| (m.as_str(), r))
        .unwrap_or(("", &[]));
    match mode {
        "publish" | "subscribe" | "publish-typed" | "subscribe-typed" | "host-pynode"
        | "write-bag" => {}
        _ => return Err(format!("unknown mode '{mode}'\n{USAGE}")),
    }
    if mode == "host-pynode" {
        parse_host_pynode(rest)?;
        return Ok((
            mode,
            Args {
                flags: std::collections::HashMap::new(),
            },
        ));
    }
    let args = parse_args(rest).map_err(|e| format!("{e}\n{USAGE}"))?;
    // Required-flag and numeric checks run here (the results are
    // discarded - the mode handler re-reads them) so an invalid
    // invocation fails before `TransportManager::init`.
    match mode {
        "write-bag" => {
            flag(&args, "path")?;
        }
        "publish" => {
            flag(&args, "topic")?;
            flag_u64(&args, "schema-hash")?;
            flag_usize(&args, "count")?;
            flag_usize(&args, "size")?;
            for opt in ["timestamp-ns", "linger-ms"] {
                if let Some(v) = args.flags.get(opt) {
                    v.parse::<u64>().map_err(|e| format!("--{opt}: {e}"))?;
                }
            }
        }
        "subscribe" => {
            flag(&args, "topic")?;
            flag_usize(&args, "count")?;
            flag_u64(&args, "timeout-ms")?;
        }
        "publish-typed" | "subscribe-typed" => {
            flag(&args, "topic")?;
            flag(&args, "schema")?;
            flag_usize(&args, "count")?;
            if mode == "subscribe-typed" {
                flag_u64(&args, "timeout-ms")?;
            } else {
                for opt in ["wait-ms", "linger-ms"] {
                    if let Some(value) = args.flags.get(opt) {
                        value.parse::<u64>().map_err(|e| format!("--{opt}: {e}"))?;
                    }
                }
            }
        }
        _ => unreachable!("mode already validated"),
    }
    Ok((mode, args))
}

fn run() -> Result<ExitCode, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let (mode, args) = parse_cli(&argv)?;
    let _ = tracing_subscriber::fmt()
        .with_max_level(tracing_subscriber::filter::LevelFilter::WARN)
        .with_writer(std::io::stderr)
        .try_init();

    if mode == "write-bag" {
        return cmd_write_bag(&args);
    }

    TransportManager::init(TransportConfig {
        node_name: "cerulion_py_fixture".to_string(),
        ..Default::default()
    })
    .map_err(|e| e.to_string())?;
    let mgr = TransportManager::get().map_err(|e| e.to_string())?;

    match mode {
        "publish" => cmd_publish(&mgr, &args),
        "subscribe" => cmd_subscribe(&mgr, &args),
        "publish-typed" => cmd_publish_typed(&mgr, &args),
        "subscribe-typed" => cmd_subscribe_typed(&mgr, &args),
        "host-pynode" => cmd_host_pynode(&mgr, &argv[1..]),
        _ => unreachable!("mode already validated"),
    }
}

/// `host-pynode <path> <ticks> [--also <path>] [--bench] [--seed <n>]
/// [--snapshot] [--input-ticks <n>] [--sync-probe] [--interleave]`.
struct HostPynodeArgs<'a> {
    path: &'a str,
    ticks: usize,
    also: Option<&'a str>,
    bench: bool,
    seed: Option<u32>,
    /// Freeze every input at each tick boundary through the node's snapshot
    /// exports, as the scheduler does for a periodic node's non-trigger inputs.
    snapshot: bool,
    /// Publish an input frame only on the first `n` ticks; later ticks read
    /// whatever the node's input discipline serves (a held frame, or nothing).
    input_ticks: Option<usize>,
    /// Before each tick, fill each input's head through the node's sync-head
    /// exports and probe for a frame behind it, printing both answers.
    sync_probe: bool,
    /// Tick the loaded nodes in turn (tick 0 of each, then tick 1 of each),
    /// as a single-process graph schedules them, instead of one node at a time.
    interleave: bool,
}

fn parse_host_pynode(argv: &[String]) -> Result<HostPynodeArgs<'_>, String> {
    let [path, ticks, options @ ..] = argv else {
        return Err("host-pynode requires <path.so> <ticks>".to_string());
    };
    let mut args = HostPynodeArgs {
        path,
        ticks: ticks
            .parse::<usize>()
            .map_err(|error| format!("invalid tick count: {error}"))?,
        also: None,
        bench: false,
        seed: None,
        snapshot: false,
        input_ticks: None,
        sync_probe: false,
        interleave: false,
    };
    let mut options = options.iter();
    while let Some(option) = options.next() {
        match option.as_str() {
            "--also" => {
                let path = options.next().ok_or("--also requires a node path")?;
                args.also = Some(path);
            }
            "--bench" => args.bench = true,
            "--snapshot" => args.snapshot = true,
            "--sync-probe" => args.sync_probe = true,
            "--interleave" => args.interleave = true,
            "--input-ticks" => {
                let count = options
                    .next()
                    .ok_or("--input-ticks requires a tick count")?;
                args.input_ticks = Some(
                    count
                        .parse::<usize>()
                        .map_err(|error| format!("--input-ticks: {error}"))?,
                );
            }
            "--seed" => {
                let seed = options.next().ok_or("--seed requires a sequence number")?;
                args.seed = Some(
                    seed.parse::<u32>()
                        .map_err(|error| format!("--seed: {error}"))?,
                );
            }
            other => return Err(format!("unknown host-pynode option '{other}'\n{USAGE}")),
        }
    }
    Ok(args)
}

/// One loaded node with the harness ports around it: a publisher feeding each
/// input and a subscriber reading each output. `context` is the node's own
/// ports, handed to `init` once every node's ports exist.
struct LoadedNode {
    entry: DylibNodeEntry,
    context: Option<NodeContext>,
    schemas: SchemaSet,
    input_publishers: Vec<(String, CerulionPublisher, u64)>,
    output_subscribers: Vec<(String, CerulionSubscriber)>,
    /// Wall time of each tick, for the `--bench` digest.
    elapsed_ns: Vec<u64>,
}

const BAG_HASH_A: u64 = 0x0BAD_C0DE_0BAD_C0DE;
const BAG_HASH_B: u64 = 0x1234_5678_9ABC_DEF0;
const VECTOR3_MSG: &str = "float64 x\nfloat64 y\nfloat64 z\n";

fn bag_oracle_frame(hash: u64, sequence: u32) -> Vec<u8> {
    let payload: Vec<u8> = (0..(8 + (sequence as usize % 5)))
        .map(|i| (sequence as usize * 7 + i) as u8)
        .collect();
    let total = WireHeader::SIZE + payload.len();
    let mut frame = vec![0u8; total];
    WireHeader {
        schema_hash: hash,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence,
        timestamp_ns: 2_000_000_000 + u64::from(sequence) * 10_000_000,
    }
    .write_to_buf(&mut frame);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

fn vector3_bag_frame() -> Result<Vec<u8>, String> {
    let schema = parse_rosmsg(VECTOR3_MSG, "Vector3", Some("geometry_msgs"))
        .map_err(|error| error.to_string())?;
    let (schemas, _) = SchemaSet::from_schemas(vec![schema]).map_err(|error| error.to_string())?;
    let layout = schemas
        .layout("geometry_msgs/Vector3")
        .ok_or("Vector3 layout unavailable")?;
    let encoder = FrameEncoder::new(layout).map_err(|error| error.to_string())?;
    let total = encoder
        .required_len(&[])
        .map_err(|error| error.to_string())?;
    let mut frame = vec![0u8; total];
    let mut cursor = encoder
        .begin(&mut frame, &[], 3_000_000_000)
        .map_err(|error| error.to_string())?;
    cursor.set_sequence(0);
    cursor
        .fixed_field_mut("x")
        .map_err(|error| error.to_string())?
        .copy_from_slice(&1.5f64.to_le_bytes());
    cursor
        .fixed_field_mut("y")
        .map_err(|error| error.to_string())?
        .copy_from_slice(&(-2.0f64).to_le_bytes());
    cursor
        .fixed_field_mut("z")
        .map_err(|error| error.to_string())?
        .copy_from_slice(&0.25f64.to_le_bytes());
    Ok(frame)
}

fn cmd_write_bag(args: &Args) -> Result<ExitCode, String> {
    let path = flag(args, "path")?;
    let vector3 = vector3_bag_frame()?;
    let vector3_hash = WireHeader::read_from_buf(&vector3)
        .ok_or("Vector3 frame header unavailable")?
        .schema_hash;
    let topics = [
        cerulion_bag::TopicSchema {
            topic: "/py_bag/a".to_string(),
            schema_name: "py_bag/A".to_string(),
            schema_hash: BAG_HASH_A,
            wire_fixed_size: 0,
        },
        cerulion_bag::TopicSchema {
            topic: "/py_bag/b".to_string(),
            schema_name: "py_bag/B".to_string(),
            schema_hash: BAG_HASH_B,
            wire_fixed_size: 0,
        },
        cerulion_bag::TopicSchema {
            topic: "/py_bag/vec".to_string(),
            schema_name: "geometry_msgs/Vector3".to_string(),
            schema_hash: vector3_hash,
            wire_fixed_size: 24,
        },
    ];
    let mut writer =
        cerulion_bag::BagWriter::create(path, cerulion_bag::BagWriterConfig::default(), &topics)
            .map_err(|error| error.to_string())?;
    let order = [
        ("/py_bag/a", BAG_HASH_A, 0u32),
        ("/py_bag/b", BAG_HASH_B, 0),
        ("/py_bag/a", BAG_HASH_A, 1),
        ("/py_bag/a", BAG_HASH_A, 2),
        ("/py_bag/b", BAG_HASH_B, 1),
        ("/py_bag/a", BAG_HASH_A, 3),
        ("/py_bag/b", BAG_HASH_B, 2),
        ("/py_bag/a", BAG_HASH_A, 4),
    ];
    for (topic, hash, sequence) in order {
        let frame = bag_oracle_frame(hash, sequence);
        let timestamp = 2_000_000_000 + u64::from(sequence) * 10_000_000;
        writer
            .write_message(topic, sequence, timestamp, timestamp, &[&frame])
            .map_err(|error| error.to_string())?;
    }
    writer
        .write_message("/py_bag/vec", 0, 3_000_000_000, 3_000_000_000, &[&vector3])
        .map_err(|error| error.to_string())?;
    writer.finalize().map_err(|error| error.to_string())?;
    println!("WROTE 9");
    Ok(ExitCode::SUCCESS)
}

fn cmd_host_pynode(mgr: &TransportManager, argv: &[String]) -> Result<ExitCode, String> {
    let HostPynodeArgs {
        path,
        ticks,
        also,
        bench,
        seed,
        snapshot,
        input_ticks,
        sync_probe,
        interleave,
    } = parse_host_pynode(argv)?;
    // Every node's ports are created before any node runs `init`, every node
    // is initialised before any node ticks, and every node is shut down after
    // every node has ticked: the shape of a single-process graph, whose nodes
    // are all wired and alive together. A failed init is reported and the
    // remaining nodes still run, so a test can show what a node initialised
    // AFTER a failed one observes; the exit code still reports the failure.
    let mut nodes = Vec::new();
    for node_path in [Some(path), also].into_iter().flatten() {
        nodes.push(load_pynode(mgr, node_path, seed)?);
    }
    let mut init_failed = false;
    let mut initialised = Vec::with_capacity(nodes.len());
    for mut node in nodes {
        let context = node
            .context
            .take()
            .ok_or("node context was already taken")?;
        match node.entry.init(context) {
            Ok(()) => initialised.push(node),
            Err(error) => {
                eprintln!("init failed: {error}");
                init_failed = true;
            }
        }
    }
    let mut nodes = initialised;
    let options = TickOptions {
        bench,
        seed,
        snapshot,
        input_ticks,
        sync_probe,
    };
    for node in &mut nodes {
        node.print_capabilities(sync_probe);
    }
    if interleave {
        for tick in 0..ticks {
            for node in &mut nodes {
                node.run_tick(tick, &options)?;
            }
        }
    } else {
        for node in &mut nodes {
            for tick in 0..ticks {
                node.run_tick(tick, &options)?;
            }
        }
    }
    for node in &mut nodes {
        node.print_bench(bench);
    }
    for node in &mut nodes {
        if let Err(error) = node.entry.shutdown() {
            println!("shutdown code=1 err={error}");
            return Ok(ExitCode::FAILURE);
        }
    }
    Ok(if init_failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    })
}

/// Load the node at `node_path` and wire a harness port to each of its ports.
fn load_pynode(
    mgr: &TransportManager,
    node_path: &str,
    seed: Option<u32>,
) -> Result<LoadedNode, String> {
    let entry =
        DylibNodeEntry::load(std::path::Path::new(node_path)).map_err(|error| error.to_string())?;
    let info = entry.info_json().map_err(|error| error.to_string())?;
    let document: serde_json::Value =
        serde_json::from_str(&info).map_err(|error| error.to_string())?;
    let node_name = std::path::Path::new(node_path)
        .file_stem()
        .and_then(|name| name.to_str())
        .unwrap_or("pynode");
    let workspace = std::env::var("CERULION_WORKSPACE")
        .map(std::path::PathBuf::from)
        .unwrap_or(std::env::current_dir().map_err(|error| error.to_string())?);
    let fixture_name = node_name
        .strip_prefix("libcerulion_pynode_")
        .unwrap_or(node_name);
    let fixture_workspace = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("pynodes")
        .join(fixture_name);
    let workspace = if fixture_workspace.is_dir() {
        fixture_workspace
    } else {
        workspace
    };
    let schemas = fixture_schemas(&workspace)?;
    let mut publishers = indexmap::IndexMap::new();
    let mut subscribers = indexmap::IndexMap::new();
    let mut input_publishers = Vec::new();
    let mut output_subscribers = Vec::new();
    for input in document["inputs"]
        .as_array()
        .ok_or("node info inputs is not an array")?
    {
        let name = input["name"].as_str().ok_or("input has no name")?;
        let topic = format!("{node_name}/{name}");
        let declared_hash = input["schema_hash"]
            .as_u64()
            .ok_or("input has no schema hash")?;
        let layout = schemas
            .layout_for_hash(declared_hash)
            .or_else(|| schemas.layout("Probe"))
            .ok_or_else(|| format!("input schema hash {declared_hash:#x} is unavailable"))?;
        let hash = layout.schema_hash;
        let total = minimum_frame_len(layout)?;
        let max_len = MaxSliceLen::try_new(total as u32).ok_or("input frame is too large")?;
        let harness = mgr
            .create_publisher(&topic, max_len, 1)
            .map_err(|error| error.to_string())?;
        let node_sub = mgr
            .create_subscriber(&topic)
            .map_err(|error| error.to_string())?;
        input_publishers.push((name.to_string(), harness, hash));
        subscribers.insert(name.to_string(), AnySubscriber::Ipc(node_sub));
    }
    let outputs = document["outputs"]
        .as_array()
        .ok_or("node info outputs is not an array")?;
    // ONE seed map for every output: the setter replaces the whole map, so
    // a per-output call would leave only the last output seeded.
    if let Some(seed) = seed {
        let seeds = outputs
            .iter()
            .map(|output| {
                let name = output["name"].as_str().ok_or("output has no name")?;
                Ok((format!("{node_name}/{name}"), seed))
            })
            .collect::<Result<std::collections::BTreeMap<_, _>, String>>()?;
        mgr.set_replay_sequence_seeds(seeds);
    }
    for output in outputs {
        let name = output["name"].as_str().ok_or("output has no name")?;
        let topic = format!("{node_name}/{name}");
        let declared_hash = output["schema_hash"]
            .as_u64()
            .ok_or("output has no schema hash")?;
        let layout = schemas
            .layout_for_hash(declared_hash)
            .or_else(|| schemas.layout("Probe"))
            .ok_or_else(|| format!("output schema hash {declared_hash:#x} is unavailable"))?;
        let total = minimum_frame_len(layout)?;
        let declared_max = output["max_slice_len_default"].as_u64();
        let max_len = match declared_max {
            Some(value) => {
                let value = u32::try_from(value)
                    .map_err(|_| "output max_slice_len_default is too large")?;
                if value < total as u32 {
                    return Err(format!(
                        "output '{name}' max_slice_len_default {value} is smaller than required frame length {total}"
                    ));
                }
                MaxSliceLen::try_new(value).ok_or("output frame is too large")?
            }
            None => MaxSliceLen::try_new(total as u32).ok_or("output frame is too large")?,
        };
        let node_pub = mgr
            .create_publisher(&topic, max_len, 1)
            .map_err(|error| error.to_string())?;
        let harness = mgr
            .create_subscriber(&topic)
            .map_err(|error| error.to_string())?;
        publishers.insert(name.to_string(), AnyPublisher::Ipc(node_pub));
        output_subscribers.push((name.to_string(), harness));
    }
    println!("node_info={info}");
    let mut runtime_env: std::collections::HashMap<String, String> = std::env::vars().collect();
    runtime_env.insert(
        "CERULION_WORKSPACE".to_string(),
        workspace.to_string_lossy().into_owned(),
    );
    let context = NodeContext::with_runtime_env(
        publishers,
        subscribers,
        Arc::new(RealClock),
        ShutdownSignal::new(),
        Arc::new(runtime_env),
    );
    Ok(LoadedNode {
        entry,
        context: Some(context),
        schemas,
        input_publishers,
        output_subscribers,
        elapsed_ns: Vec::new(),
    })
}

/// What every tick of every node does with its inputs and its output line.
struct TickOptions {
    bench: bool,
    seed: Option<u32>,
    snapshot: bool,
    input_ticks: Option<usize>,
    sync_probe: bool,
}

impl LoadedNode {
    fn input_names(&self) -> Vec<String> {
        self.input_publishers
            .iter()
            .map(|(name, _, _)| name.clone())
            .collect()
    }

    /// The input-discipline capabilities the loader resolved for this node.
    fn print_capabilities(&self, sync_probe: bool) {
        if sync_probe {
            println!(
                "sync_head_ops={} unified_drain={}",
                self.entry.supports_sync_head_ops(),
                self.entry.unifies_trigger_drain()
            );
        }
    }

    /// Feed the node's inputs for `tick`, tick it once and print the output.
    fn run_tick(&mut self, tick: usize, options: &TickOptions) -> Result<(), String> {
        let input_names = self.input_names();
        let publish_inputs = options.input_ticks.is_none_or(|count| tick < count);
        for (_, publisher, hash) in self.input_publishers.iter_mut().filter(|_| publish_inputs) {
            let layout = self
                .schemas
                .layout_for_hash(*hash)
                .ok_or("input schema disappeared")?;
            let encoder = FrameEncoder::new(layout).map_err(|error| error.to_string())?;
            let zero_lengths = vec![0; layout.variable_fields.len()];
            let total = encoder
                .required_len(&zero_lengths)
                .map_err(|error| error.to_string())?;
            let mut loan = publisher
                .loan_raw_uninit(total)
                .map_err(|error| error.to_string())?;
            for byte in loan.bytes_uninit_mut() {
                byte.write(0);
            }
            let mut loan = unsafe {
                // SAFETY: every byte in the exact-size loan was initialized above.
                loan.assume_init()
            };
            let mut cursor = encoder
                .begin(loan.bytes_mut(), &zero_lengths, tick as u64)
                .map_err(|error| error.to_string())?;
            let value = cursor
                .fixed_field_mut("value")
                .map_err(|error| error.to_string())?;
            match value.len() {
                4 => value.copy_from_slice(&(tick as u32).to_le_bytes()),
                8 => value.copy_from_slice(&(tick as i64).to_le_bytes()),
                size => return Err(format!("unsupported fixture value width {size}")),
            }
            publisher
                .send_raw_loan(loan)
                .map_err(|error| error.to_string())?;
            publisher.check_subscriber_events();
            publisher
                .notify_sent_sample()
                .map_err(|error| error.to_string())?;
        }
        if options.snapshot {
            self.entry.snapshot_inputs(&input_names);
        }
        if options.sync_probe {
            for name in &input_names {
                let fill = self.entry.sync_head_op(name, SyncHeadOp::FillBoundary);
                let probe = self.entry.sync_head_op(name, SyncHeadOp::ProbeNext);
                println!(
                    "sync={name} fill={} probe={}",
                    sync_answer_kind(&fill),
                    sync_answer_kind(&probe)
                );
            }
        }
        let tick_on_worker = std::env::var_os("CERULION_PYNODE_TICK_THREAD").is_some();
        let started = Instant::now();
        let entry = &mut self.entry;
        let tick_result = if tick_on_worker {
            std::thread::scope(|scope| scope.spawn(|| entry.tick()).join())
                .map_err(|_| "tick worker thread panicked")?
        } else {
            entry.tick()
        };
        self.elapsed_ns.push(started.elapsed().as_nanos() as u64);
        match tick_result {
            Ok(()) => {
                let mut output = String::new();
                let mut sequence = None;
                for (_, subscriber) in self.output_subscribers.iter_mut() {
                    if let Some(sample) = subscriber
                        .try_receive_one_owned()
                        .map_err(|error| error.to_string())?
                    {
                        sequence = WireHeader::read_from_buf(sample.payload())
                            .map(|header| header.sequence);
                        output = sample.payload()[WireHeader::SIZE..]
                            .iter()
                            .map(|byte| format!("{byte:02x}"))
                            .collect();
                    }
                }
                match (options.bench, options.seed, sequence) {
                    (true, _, _) => {}
                    (false, Some(_), Some(sequence)) => {
                        println!("tick={tick} code=0 out={output} seq={sequence}")
                    }
                    (false, _, _) => println!("tick={tick} code=0 out={output}"),
                }
            }
            Err(error) => {
                let text = error.to_string();
                let text = text
                    .split_once(" error: ")
                    .map(|(_, detail)| detail)
                    .unwrap_or(&text);
                let mut output_frames = 0;
                for (_, subscriber) in self.output_subscribers.iter_mut() {
                    if subscriber
                        .try_receive_one_owned()
                        .map_err(|error| error.to_string())?
                        .is_some()
                    {
                        output_frames += 1;
                    }
                }
                if options.bench {
                    eprintln!("tick={tick} code=1 err={text} out_frames={output_frames}");
                } else {
                    println!("tick={tick} code=1 err={text} out_frames={output_frames}");
                }
            }
        }
        Ok(())
    }

    /// The tick-latency digest a `--bench` run prints once per node.
    fn print_bench(&mut self, bench: bool) {
        if !bench || self.elapsed_ns.is_empty() {
            return;
        }
        let elapsed_ns = &mut self.elapsed_ns;
        elapsed_ns.sort_unstable();
        let mean =
            elapsed_ns.iter().map(|value| *value as f64).sum::<f64>() / elapsed_ns.len() as f64;
        let percentile = |fraction: f64| {
            let index = ((elapsed_ns.len() - 1) as f64 * fraction).round() as usize;
            elapsed_ns[index]
        };
        println!(
            "bench_ticks={} mean_ns={mean:.1} p50_ns={} p99_ns={}",
            elapsed_ns.len(),
            percentile(0.50),
            percentile(0.99)
        );
    }
}

/// The smallest frame a layout encodes: every variable field empty. A schema
/// without variable fields has exactly one frame length, which this is.
fn minimum_frame_len(layout: &cerulion_core::dynamic::WireLayout) -> Result<usize, String> {
    FrameEncoder::new(layout)
        .map_err(|error| error.to_string())?
        .required_len(&vec![0; layout.variable_fields.len()])
        .map_err(|error| error.to_string())
}

/// A sync-op answer's kind, without its stamp, so a run prints the same text
/// whatever the clock says.
fn sync_answer_kind(answer: &SyncOpAnswer) -> &'static str {
    match answer {
        SyncOpAnswer::Head(_) => "Head",
        SyncOpAnswer::Nothing => "Nothing",
        SyncOpAnswer::Present => "Present",
        SyncOpAnswer::Stamp(_) => "Stamp",
        SyncOpAnswer::Failed => "Failed",
    }
}

// Vector3 has three f64 fields, so its fixed body is 3 * 8 = 24 bytes.
// These are the little-endian encodings of 1.5, -2.25, and 1e-3.
const VECTOR3_BODY: [u8; 3 * std::mem::size_of::<f64>()] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xC0,
    0xFC, 0xA9, 0xF1, 0xD2, 0x4D, 0x62, 0x50, 0x3F,
];

// Header body arithmetic: sec (4) + nanosec (4) + offset (4) + length (4)
// + five frame_id bytes = 21 bytes. The offset points past the 16-byte fixed
// prefix to the UTF-8 bytes for "laser".
const LASER_HEADER_BODY: [u8; 21] = [
    0x07, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
    b'l', b'a', b's', b'e', b'r',
];

fn frame_hex(bytes: &[u8]) -> String {
    let mut copy = bytes.to_vec();
    if copy.len() >= 32 {
        copy[20..32].fill(0);
    }
    copy.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn typed_schema(args: &Args) -> Result<&str, String> {
    match flag(args, "schema")? {
        "geometry_msgs/Vector3" | "sensor_msgs/LaserScan" => Ok(flag(args, "schema")?),
        other => Err(format!("unsupported typed fixture schema '{other}'")),
    }
}

fn cmd_publish_typed(mgr: &TransportManager, args: &Args) -> Result<ExitCode, String> {
    let topic = flag(args, "topic")?;
    let count = flag_usize(args, "count")?;
    let wait_ms = args
        .flags
        .get("wait-ms")
        .map(|s| s.parse::<u64>().map_err(|e| format!("--wait-ms: {e}")))
        .transpose()?
        .unwrap_or(500);
    let linger_ms = args
        .flags
        .get("linger-ms")
        .map(|s| s.parse::<u64>().map_err(|e| format!("--linger-ms: {e}")))
        .transpose()?
        .unwrap_or(1000);
    let schema = typed_schema(args)?;
    match schema {
        "geometry_msgs/Vector3" => {
            let mut publisher = mgr
                .create_publisher_typed::<geometry_msgs::Vector3>(topic, None)
                .map_err(|e| e.to_string())?;
            println!("READY");
            std::io::stdout().flush().map_err(|e| e.to_string())?;
            std::thread::sleep(Duration::from_millis(wait_ms));
            for _ in 0..count {
                let mut proxy = publisher
                    .loan_proxy::<geometry_msgs::Vector3>()
                    .map_err(|e| e.to_string())?;
                let snapshot = geometry_msgs::Vector3Snapshot {
                    x: 1.5,
                    y: -2.25,
                    z: 1e-3,
                };
                proxy.write_from_snapshot(&snapshot);
                drop(proxy);
                publisher.check_subscriber_events();
                publisher.notify_sent_sample().map_err(|e| e.to_string())?;
            }
            // Linger INSIDE the arm: `publisher` owns the SHM segment the
            // frames live in, and the arm's end drops it (see `cmd_publish`).
            std::thread::sleep(Duration::from_millis(linger_ms));
        }
        "sensor_msgs/LaserScan" => {
            let mut publisher = mgr
                .create_publisher_typed::<sensor_msgs::LaserScan>(
                    topic,
                    Some(MaxSliceLen::const_new(16 * 1024)),
                )
                .map_err(|e| e.to_string())?;
            println!("READY");
            std::io::stdout().flush().map_err(|e| e.to_string())?;
            std::thread::sleep(Duration::from_millis(wait_ms));
            for _ in 0..count {
                let mut proxy = publisher
                    .loan_proxy::<sensor_msgs::LaserScan>()
                    .map_err(|e| e.to_string())?;
                let snapshot = sensor_msgs::LaserScanSnapshot {
                    header: LASER_HEADER_BODY.to_vec(),
                    angle_min: -1.5,
                    angle_max: 1.5,
                    angle_increment: 0.25,
                    time_increment: 0.001,
                    scan_time: 0.1,
                    range_min: 0.2,
                    range_max: 30.0,
                    ranges: vec![1.5, 2.25, 3.0, 0.5],
                    intensities: vec![10.0, 20.0, 30.0, 40.0],
                };
                proxy
                    .write_from_snapshot(&snapshot)
                    .map_err(|e| e.to_string())?;
                drop(proxy);
                publisher.check_subscriber_events();
                publisher.notify_sent_sample().map_err(|e| e.to_string())?;
            }
            // Linger INSIDE the arm: `publisher` owns the SHM segment the
            // frames live in, and the arm's end drops it (see `cmd_publish`).
            std::thread::sleep(Duration::from_millis(linger_ms));
        }
        _ => unreachable!(),
    }
    Ok(ExitCode::SUCCESS)
}

/// The vendored ROS 2 corpus as a dynamic schema set: `FrameView` checks a
/// received frame's offset table and nested bodies before the generated
/// reader, which trusts them, touches it.
fn builtin_schema_set() -> Result<SchemaSet, String> {
    SchemaSet::from_schemas(builtin_schemas()?)
        .map(|(set, _)| set)
        .map_err(|e| e.to_string())
}

/// Reject a frame whose wire header does not carry `T`'s schema hash, or
/// whose body is shorter than `T`'s fixed section, BEFORE the typed
/// `from_bytes` reinterprets it - a wrong-schema or truncated frame must
/// fail loudly instead of decoding garbage bytes, and so must one whose
/// offset table or nested bodies do not validate.
/// Split an encoded `std_msgs/Header` body (`stamp.sec`, `stamp.nanosec`,
/// then the `frame_id` string) without trusting its length: the top-level
/// `FrameView` check does not validate a nested entry's contents.
/// The LaserScan's `std_msgs/Header` (`stamp.sec`, `stamp.nanosec`,
/// `frame_id`), read through a full walk so the nested header's own offset
/// table is bounds-checked rather than assumed.
fn header_fields(set: &SchemaSet, bytes: &[u8]) -> Result<(i32, u32, String), String> {
    let decoded = FrameView::new(set.walker(), bytes)
        .and_then(|view| view.decode(set.walker()))
        .map_err(|e| format!("invalid typed frame: {e}"))?;
    let Some(FrameValueKind::Nested(header)) = decoded.field("header") else {
        return Err("LaserScan has no nested header".to_string());
    };
    let Some(FrameValueKind::Nested(stamp)) = header.field("stamp") else {
        return Err("header has no nested stamp".to_string());
    };
    let (Some(FrameValueKind::I32(sec)), Some(FrameValueKind::U32(nanosec))) =
        (stamp.field("sec"), stamp.field("nanosec"))
    else {
        return Err("header stamp is not (int32 sec, uint32 nanosec)".to_string());
    };
    let Some(FrameValueKind::Str(frame_id)) = header.field("frame_id") else {
        return Err("header frame_id is not a UTF-8 string".to_string());
    };
    Ok((*sec, *nanosec, (*frame_id).to_string()))
}

fn check_typed_frame<T: ShmMessage>(set: &SchemaSet, bytes: &[u8]) -> Result<(), String> {
    // `read_from_buf` copies into an owned header: `from_bytes` reinterprets
    // the slice in place and refuses non-8-byte-aligned buffers, which a
    // received SHM slice is not guaranteed to be.
    let header = WireHeader::read_from_buf(bytes).ok_or("frame shorter than wire header")?;
    if header.schema_hash != T::SCHEMA_HASH {
        return Err(format!(
            "schema hash mismatch: expected {:#x} got {:#x}",
            T::SCHEMA_HASH,
            header.schema_hash
        ));
    }
    // The body floor is the fixed section PLUS the offset table (8 bytes
    // per variable field; zero for fixed schemas) - a frame any shorter
    // cannot carry a well-formed typed body at all.
    if bytes.len() < WireHeader::SIZE + T::WIRE_FIXED_SIZE + 8 * T::VARIABLE_FIELD_COUNT {
        return Err("body shorter than the fixed section + offset table".to_string());
    }
    FrameView::new(set.walker(), bytes).map_err(|e| format!("invalid typed frame: {e}"))?;
    Ok(())
}

fn cmd_subscribe_typed(mgr: &TransportManager, args: &Args) -> Result<ExitCode, String> {
    let topic = flag(args, "topic")?;
    let count = flag_usize(args, "count")?;
    let timeout_ms = flag_u64(args, "timeout-ms")?;
    let schema = typed_schema(args)?;
    let set = builtin_schema_set()?;
    let mut scratch = Vec::new();
    let sub = mgr.create_subscriber(topic).map_err(|e| e.to_string())?;
    println!("READY");
    std::io::stdout().flush().map_err(|e| e.to_string())?;
    // An unrepresentable deadline (e.g. `u64::MAX` ms) waits without bound.
    let deadline = Instant::now().checked_add(Duration::from_millis(timeout_ms));
    match schema {
        "geometry_msgs/Vector3" => {
            for i in 0..count {
                loop {
                    if let Some(sample) = sub.try_receive_one_owned().map_err(|e| e.to_string())? {
                        let bytes = align::aligned_for_validation(sample.payload(), &mut scratch);
                        check_typed_frame::<geometry_msgs::Vector3>(&set, bytes)?;
                        let value =
                            geometry_msgs::Vector3Shm::from_bytes(&bytes[WireHeader::SIZE..])
                                .snapshot();
                        if bytes[WireHeader::SIZE..] != VECTOR3_BODY {
                            return Err("Vector3 body oracle mismatch".to_string());
                        }
                        println!(
                            "frame index={i} schema={schema} values=x={} y={} z={}",
                            value.x, value.y, value.z
                        );
                        println!("frame_hex={}", frame_hex(bytes));
                        break;
                    }
                    if deadline.is_some_and(|d| Instant::now() >= d) {
                        println!("TIMEOUT");
                        return Ok(ExitCode::from(2));
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        }
        "sensor_msgs/LaserScan" => {
            for i in 0..count {
                loop {
                    if let Some(sample) = sub.try_receive_one_owned().map_err(|e| e.to_string())? {
                        let bytes = align::aligned_for_validation(sample.payload(), &mut scratch);
                        check_typed_frame::<sensor_msgs::LaserScan>(&set, bytes)?;
                        let value =
                            sensor_msgs::LaserScanShm::from_bytes(&bytes[WireHeader::SIZE..])
                                .snapshot();
                        let (stamp_sec, stamp_nanosec, frame_id) = header_fields(&set, bytes)?;
                        println!(
                            "frame index={i} schema={schema} values=angle_min={} angle_max={} angle_increment={} time_increment={} scan_time={} range_min={} range_max={} ranges={:?} intensities={:?} header_stamp_sec={} header_stamp_nanosec={} frame_id={}",
                            value.angle_min,
                            value.angle_max,
                            value.angle_increment,
                            value.time_increment,
                            value.scan_time,
                            value.range_min,
                            value.range_max,
                            value.ranges,
                            value.intensities,
                            stamp_sec,
                            stamp_nanosec,
                            frame_id,
                        );
                        println!("frame_hex={}", frame_hex(bytes));
                        break;
                    }
                    if deadline.is_some_and(|d| Instant::now() >= d) {
                        println!("TIMEOUT");
                        return Ok(ExitCode::from(2));
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        }
        _ => unreachable!(),
    }
    Ok(ExitCode::SUCCESS)
}

fn cmd_publish(mgr: &TransportManager, args: &Args) -> Result<ExitCode, String> {
    let topic = flag(args, "topic")?;
    let schema_hash = flag_u64(args, "schema-hash")?;
    let count = flag_usize(args, "count")?;
    let size = flag_usize(args, "size")?;
    let timestamp_ns = args
        .flags
        .get("timestamp-ns")
        .map(|s| s.parse::<u64>().map_err(|e| format!("--timestamp-ns: {e}")))
        .transpose()?;
    let linger_ms = args
        .flags
        .get("linger-ms")
        .map(|s| s.parse::<u64>().map_err(|e| format!("--linger-ms: {e}")))
        .transpose()?
        .unwrap_or(1000);

    let slot_len = (size as u64)
        .checked_add(WireHeader::SIZE as u64)
        .and_then(|n| u32::try_from(n).ok())
        .and_then(MaxSliceLen::try_new)
        .ok_or("--size does not fit the wire format")?;
    let mut publisher = mgr
        .create_publisher(topic, slot_len, 0)
        .map_err(|e| e.to_string())?;

    for i in 0..count {
        let body = pattern(size, i as u64);
        let mut header =
            WireHeader::new(schema_hash, i as u32, timestamp_ns.unwrap_or_else(real_ns));
        header.total_size = (WireHeader::SIZE + size) as u32;
        let mut frame = vec![0u8; WireHeader::SIZE + size];
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        frame[WireHeader::SIZE..].copy_from_slice(&body);
        publisher.publish_raw(&frame).map_err(|e| e.to_string())?;
        publisher.check_subscriber_events();
        if let Err(e) = publisher.notify_sent_sample() {
            eprintln!("notify failed: {e:?}");
        }
        println!("published seq={i} size={size}");
    }
    // A published sample's bytes live in THIS process's SHM segment -
    // the subscriber maps that segment lazily on its first
    // `update_connections` (i.e. on its next receive). Exiting before
    // that mapping happens destroys the segment under the queued
    // offsets and the frame is silently undeliverable (measured: an
    // instant-exit publisher loses every frame). Linger so receivers
    // parked on the notify can connect and map.
    std::thread::sleep(Duration::from_millis(linger_ms));
    Ok(ExitCode::SUCCESS)
}

fn cmd_subscribe(mgr: &TransportManager, args: &Args) -> Result<ExitCode, String> {
    let topic = flag(args, "topic")?;
    let count = flag_usize(args, "count")?;
    let timeout_ms = flag_u64(args, "timeout-ms")?;

    let sub = mgr.create_subscriber(topic).map_err(|e| e.to_string())?;
    // Flush so a spawning parent can read READY before publishing.
    println!("READY");
    std::io::stdout().flush().map_err(|e| e.to_string())?;

    // Consume EXACTLY `--count` frames; `--timeout-ms` is the overall
    // budget from READY to the last frame, not a per-frame deadline.
    // An unrepresentable deadline (e.g. `u64::MAX` ms) waits without bound.
    let deadline = Instant::now().checked_add(Duration::from_millis(timeout_ms));
    let mut received = 0usize;
    loop {
        // `--count 0` exits immediately - consume nothing.
        if received == count {
            break;
        }
        if let Some(sample) = sub.try_receive_one_owned().map_err(|e| e.to_string())? {
            let header = sample
                .wire_header()
                .ok_or("received a frame with a malformed wire header")?;
            let payload = sample.payload();
            let body = &payload[WireHeader::SIZE..];
            println!(
                "frame seq={} schema_hash={} timestamp_ns={} total_size={} body_len={} fnv1a64={:016x}",
                header.sequence,
                header.schema_hash,
                header.timestamp_ns,
                header.total_size,
                body.len(),
                fnv1a64(body),
            );
            // The digest line IS the fixture's output: a parent that cannot
            // read it has nothing to diff, so a failed flush (a closed pipe)
            // is an error, not a frame counted and a success exit.
            std::io::stdout().flush().map_err(|e| e.to_string())?;
            received += 1;
            drop(sample);
            continue;
        }
        if deadline.is_some_and(|d| Instant::now() >= d) {
            println!("TIMEOUT");
            return Ok(ExitCode::from(2));
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    Ok(ExitCode::SUCCESS)
}

const USAGE: &str = "usage:\n  cerulion_py_fixture publish --topic T --schema-hash H --count N --size S [--timestamp-ns TS] [--linger-ms L]\n  cerulion_py_fixture subscribe --topic T --count N --timeout-ms M\n  cerulion_py_fixture publish-typed --topic T --schema geometry_msgs/Vector3|sensor_msgs/LaserScan --count N [--wait-ms W] [--linger-ms L]\n  cerulion_py_fixture subscribe-typed --topic T --schema geometry_msgs/Vector3|sensor_msgs/LaserScan --count N --timeout-ms M\n  cerulion_py_fixture host-pynode <path> <ticks> [--also <path>] [--bench] [--seed <n>] [--snapshot] [--input-ticks <n>] [--sync-probe] [--interleave]\n  cerulion_py_fixture write-bag --path FILE";

fn main() -> ExitCode {
    match run() {
        Ok(code) => code,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn argv(parts: &[&str]) -> Vec<String> {
        parts.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn parse_args_empty_slice() {
        let args = parse_args(&[]).unwrap();
        assert!(args.flags.is_empty());
    }

    #[test]
    fn parse_args_flag_with_value() {
        let args = parse_args(&argv(&["--topic", "/t"])).unwrap();
        assert_eq!(args.flags.get("topic").map(String::as_str), Some("/t"));
    }

    #[test]
    fn parse_args_missing_value() {
        let err = parse_args(&argv(&["--topic"])).unwrap_err();
        assert!(err.contains("--topic needs a value"), "{err}");
    }

    #[test]
    fn parse_args_positional_rejected() {
        let err = parse_args(&argv(&["oops"])).unwrap_err();
        assert!(err.contains("unexpected positional"), "{err}");
    }

    #[test]
    fn parse_cli_empty_argv_is_usage_error() {
        let err = parse_cli(&[]).unwrap_err();
        assert!(err.contains("unknown mode"), "{err}");
        assert!(err.contains("usage:"), "{err}");
    }

    #[test]
    fn parse_cli_unknown_mode_is_usage_error() {
        let err = parse_cli(&argv(&["frobnicate"])).unwrap_err();
        assert!(err.contains("unknown mode 'frobnicate'"), "{err}");
    }

    #[test]
    fn parse_cli_publish_missing_flag() {
        let err = parse_cli(&argv(&["publish", "--topic", "/t"])).unwrap_err();
        assert!(err.contains("--schema-hash"), "{err}");
    }

    #[test]
    fn parse_cli_subscribe_bad_timeout() {
        let err = parse_cli(&argv(&[
            "subscribe",
            "--topic",
            "/t",
            "--count",
            "1",
            "--timeout-ms",
            "abc",
        ]))
        .unwrap_err();
        assert!(err.contains("--timeout-ms"), "{err}");
    }

    #[test]
    fn parse_cli_publish_valid() {
        let argv = argv(&[
            "publish",
            "--topic",
            "/t",
            "--schema-hash",
            "1",
            "--count",
            "2",
            "--size",
            "64",
            "--timestamp-ns",
            "1000",
            "--linger-ms",
            "0",
        ]);
        let (mode, args) = parse_cli(&argv).unwrap();
        assert_eq!(mode, "publish");
        assert_eq!(args.flags.len(), 6);
    }

    #[test]
    fn parse_cli_subscribe_valid() {
        let argv = argv(&[
            "subscribe",
            "--topic",
            "/t",
            "--count",
            "1",
            "--timeout-ms",
            "500",
        ]);
        let (mode, _) = parse_cli(&argv).unwrap();
        assert_eq!(mode, "subscribe");
    }
}
