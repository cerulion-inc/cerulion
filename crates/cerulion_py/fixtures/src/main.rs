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
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::TransportConfig;
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
        "publish" | "subscribe" => {}
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
        _ => unreachable!("mode already validated"),
    }
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

const USAGE: &str = "usage:\n  cerulion_py_fixture publish --topic T --schema-hash H --count N --size S [--timestamp-ns TS] [--linger-ms L]\n  cerulion_py_fixture subscribe --topic T --count N --timeout-ms M";

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
