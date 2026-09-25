// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion_py_fixture` - the Rust peer for the cerulion_py test suite.
//!
//! Two modes: `publish` stamps deterministic-pattern wire frames a Python
//! subscriber can oracle-check; `subscribe` receives frames and prints a
//! one-line digest (sequence, header fields, FNV-1a of the body) Python
//! publishers can be asserted against.

// A bin's job is to print: every output line IS the test interface.
#![allow(clippy::print_stdout)]

use cerulion_core::clock::real_ns;
use cerulion_core::clock::RealClock;
use cerulion_core::codegen::{parse_rosmsg, MessageSchema};
use cerulion_core::dynamic::{FrameEncoder, FrameView, SchemaSet};
use cerulion_core::graph::node::{
    AnyPublisher, AnySubscriber, DylibNodeEntry, NodeContext, NodeEntry, ShutdownSignal,
};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportConfig;
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
    let builtin = builtin_schemas()?;
    let (workspace_set, _warnings) =
        SchemaSet::from_workspace_dir(workspace).map_err(|error| error.to_string())?;
    let mut schemas = builtin;
    for schema in workspace_set.schemas() {
        let qualified_name = schema.qualified_name();
        schemas.retain(|builtin| builtin.qualified_name() != qualified_name);
        schemas.push(schema.clone());
    }
    SchemaSet::from_schemas(schemas)
        .map(|(schemas, _)| schemas)
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
        "publish" | "subscribe" | "publish-typed" | "subscribe-typed" | "host-pynode" => {}
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

/// `host-pynode <path> <ticks> [--also <path>] [--bench] [--seed <n>]`.
struct HostPynodeArgs<'a> {
    path: &'a str,
    ticks: usize,
    also: Option<&'a str>,
    bench: bool,
    seed: Option<u32>,
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
    };
    let mut options = options.iter();
    while let Some(option) = options.next() {
        match option.as_str() {
            "--also" => {
                let path = options.next().ok_or("--also requires a node path")?;
                args.also = Some(path);
            }
            "--bench" => args.bench = true,
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

fn cmd_host_pynode(mgr: &TransportManager, argv: &[String]) -> Result<ExitCode, String> {
    let HostPynodeArgs {
        path,
        ticks,
        also,
        bench,
        seed,
    } = parse_host_pynode(argv)?;
    for node_path in [Some(path), also].into_iter().flatten() {
        let mut entry = DylibNodeEntry::load(std::path::Path::new(node_path))
            .map_err(|error| error.to_string())?;
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
            let total = FrameEncoder::new(layout)
                .map_err(|error| error.to_string())?
                .required_len(&[])
                .map_err(|error| error.to_string())?;
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
        for output in document["outputs"]
            .as_array()
            .ok_or("node info outputs is not an array")?
        {
            let name = output["name"].as_str().ok_or("output has no name")?;
            let topic = format!("{node_name}/{name}");
            let declared_hash = output["schema_hash"]
                .as_u64()
                .ok_or("output has no schema hash")?;
            let layout = schemas
                .layout_for_hash(declared_hash)
                .or_else(|| schemas.layout("Probe"))
                .ok_or_else(|| format!("output schema hash {declared_hash:#x} is unavailable"))?;
            let total = FrameEncoder::new(layout)
                .map_err(|error| error.to_string())?
                .required_len(&[])
                .map_err(|error| error.to_string())?;
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
            if let Some(seed) = seed {
                mgr.set_replay_sequence_seeds([(topic.clone(), seed)].into());
            }
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
        if let Err(error) = entry.init(context) {
            eprintln!("init failed: {error}");
            return Ok(ExitCode::FAILURE);
        }
        let tick_on_worker = std::env::var_os("CERULION_PYNODE_TICK_THREAD").is_some();
        let mut elapsed_ns = Vec::with_capacity(ticks);
        for tick in 0..ticks {
            for (_, publisher, hash) in &mut input_publishers {
                let layout = schemas
                    .layout_for_hash(*hash)
                    .ok_or("input schema disappeared")?;
                let encoder = FrameEncoder::new(layout).map_err(|error| error.to_string())?;
                let total = encoder
                    .required_len(&[])
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
                    .begin(loan.bytes_mut(), &[], tick as u64)
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
            let started = Instant::now();
            let tick_result = if tick_on_worker {
                std::thread::scope(|scope| scope.spawn(|| entry.tick()).join())
                    .map_err(|_| "tick worker thread panicked")?
            } else {
                entry.tick()
            };
            elapsed_ns.push(started.elapsed().as_nanos() as u64);
            match tick_result {
                Ok(()) => {
                    let mut output = String::new();
                    let mut sequence = None;
                    for (_, subscriber) in &mut output_subscribers {
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
                    match (bench, seed, sequence) {
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
                    for (_, subscriber) in &mut output_subscribers {
                        if subscriber
                            .try_receive_one_owned()
                            .map_err(|error| error.to_string())?
                            .is_some()
                        {
                            output_frames += 1;
                        }
                    }
                    if bench {
                        eprintln!("tick={tick} code=1 err={text} out_frames={output_frames}");
                    } else {
                        println!("tick={tick} code=1 err={text} out_frames={output_frames}");
                    }
                }
            }
        }
        if bench && !elapsed_ns.is_empty() {
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
        if let Err(error) = entry.shutdown() {
            println!("shutdown code=1 err={error}");
            return Ok(ExitCode::FAILURE);
        }
    }
    Ok(ExitCode::SUCCESS)
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
        }
        _ => unreachable!(),
    }
    std::thread::sleep(Duration::from_millis(linger_ms));
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

/// Copy the whole frame into `scratch` at an 8-byte-aligned start: both
/// `FrameView` and the generated `from_bytes` require element alignment in
/// memory, and a received SHM slice carries none. The body starts at
/// `WireHeader::SIZE`, a multiple of 8, so it stays aligned too.
fn aligned_frame<'s>(bytes: &[u8], scratch: &'s mut Vec<u8>) -> &'s [u8] {
    const ALIGN: usize = 8;
    scratch.clear();
    scratch.resize(bytes.len() + ALIGN, 0);
    let start = scratch.as_ptr().align_offset(ALIGN);
    scratch[start..start + bytes.len()].copy_from_slice(bytes);
    &scratch[start..start + bytes.len()]
}

/// Reject a frame whose wire header does not carry `T`'s schema hash, or
/// whose body is shorter than `T`'s fixed section, BEFORE the typed
/// `from_bytes` reinterprets it - a wrong-schema or truncated frame must
/// fail loudly instead of decoding garbage bytes, and so must one whose
/// offset table or nested bodies do not validate.
/// Split an encoded `std_msgs/Header` body (`stamp.sec`, `stamp.nanosec`,
/// then the `frame_id` string) without trusting its length: the top-level
/// `FrameView` check does not validate a nested entry's contents.
fn header_fields(header: &[u8]) -> Result<(i32, u32, &str), String> {
    let short = || format!("nested header is {} bytes, shorter than 16", header.len());
    let sec = header.get(0..4).ok_or_else(short)?;
    let nanosec = header.get(4..8).ok_or_else(short)?;
    let frame_id = header.get(16..).ok_or_else(short)?;
    Ok((
        i32::from_le_bytes(sec.try_into().map_err(|_| short())?),
        u32::from_le_bytes(nanosec.try_into().map_err(|_| short())?),
        std::str::from_utf8(frame_id).map_err(|e| e.to_string())?,
    ))
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
    let deadline = Instant::now() + Duration::from_millis(timeout_ms);
    match schema {
        "geometry_msgs/Vector3" => {
            for i in 0..count {
                loop {
                    if let Some(sample) = sub.try_receive_one_owned().map_err(|e| e.to_string())? {
                        let bytes = aligned_frame(sample.payload(), &mut scratch);
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
                    if Instant::now() >= deadline {
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
                        let bytes = aligned_frame(sample.payload(), &mut scratch);
                        check_typed_frame::<sensor_msgs::LaserScan>(&set, bytes)?;
                        let value =
                            sensor_msgs::LaserScanShm::from_bytes(&bytes[WireHeader::SIZE..])
                                .snapshot();
                        let (stamp_sec, stamp_nanosec, frame_id) = header_fields(&value.header)?;
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
                    if Instant::now() >= deadline {
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
            let _ = std::io::stdout().flush();
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

const USAGE: &str = "usage:\n  cerulion_py_fixture publish --topic T --schema-hash H --count N --size S [--timestamp-ns TS] [--linger-ms L]\n  cerulion_py_fixture subscribe --topic T --count N --timeout-ms M\n  cerulion_py_fixture publish-typed --topic T --schema geometry_msgs/Vector3|sensor_msgs/LaserScan --count N [--wait-ms W] [--linger-ms L]\n  cerulion_py_fixture subscribe-typed --topic T --schema geometry_msgs/Vector3|sensor_msgs/LaserScan --count N --timeout-ms M\n  cerulion_py_fixture host-pynode <path> <ticks> [--also <path>] [--bench] [--seed <n>]";

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
