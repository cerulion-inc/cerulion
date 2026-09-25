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
use cerulion_core::codegen::{parse_rosmsg, FrameValueKind};
use cerulion_core::dynamic::{FrameView, SchemaSet};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportConfig;
use native_ros2_messages::{geometry_msgs, sensor_msgs};
use std::io::Write;
use std::process::ExitCode;
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
        "publish" | "subscribe" | "publish-typed" | "subscribe-typed" => {}
        _ => return Err(format!("unknown mode '{mode}'\n{USAGE}")),
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
        _ => unreachable!("mode already validated"),
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
    let schemas = native_ros2_messages::BUILTIN_MSGS
        .iter()
        .map(|&(package, name, text)| {
            parse_rosmsg(text, name, Some(package)).map_err(|e| format!("{package}/{name}: {e}"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    SchemaSet::from_schemas(schemas)
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

const USAGE: &str = "usage:\n  cerulion_py_fixture publish --topic T --schema-hash H --count N --size S [--timestamp-ns TS] [--linger-ms L]\n  cerulion_py_fixture subscribe --topic T --count N --timeout-ms M\n  cerulion_py_fixture publish-typed --topic T --schema geometry_msgs/Vector3|sensor_msgs/LaserScan --count N [--wait-ms W] [--linger-ms L]\n  cerulion_py_fixture subscribe-typed --topic T --schema geometry_msgs/Vector3|sensor_msgs/LaserScan --count N --timeout-ms M";

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
