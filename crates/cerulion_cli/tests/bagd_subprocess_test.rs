// SPDX-License-Identifier: AGPL-3.0-only
//! Subprocess tests for the `cerulion bagd` SUBCOMMAND:
//! spawn the REAL `cerulion` binary with the
//! `bagd` first arg (there is no standalone `bagd` binary), joined to this
//! test's ISOLATED iceoryx2 namespace (via the hidden `--test-iox2-config`
//! flag), publish frames from the parent, send SIGTERM, and prove the recorder
//! drains + finalizes cleanly (exit 0, a `Finalized` bag with every frame).
//! Moved verbatim from `cerulion_bagd/tests/subprocess_test.rs` — only the
//! spawn line + arg vector changed (`bagd` prepended).
//!
//! Also holds the `#[ignore]` placeholder for the memcpy-interposition
//! zero-copy proof (needs Linux with memcpy interposition; not run in CI).

#![cfg(unix)]

mod common;

use std::io::Write;
use std::path::PathBuf;
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use cerulion_bag::BagReader;
use serial_test::serial;

use common::*;

/// SIGKILL + reap on drop so a panicking test never leaks the child.
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Poll `try_wait` until the child exits or `timeout` elapses.
fn wait_bounded(child: &mut Child, timeout: Duration) -> Option<ExitStatus> {
    let start = Instant::now();
    loop {
        match child.try_wait().expect("try_wait") {
            Some(status) => return Some(status),
            None if start.elapsed() > timeout => return None,
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
}

fn send_sigterm(child: &Child) {
    // SAFETY: `kill(2)` with a valid pid + signal; no memory is touched.
    let ret = unsafe { libc::kill(child.id() as libc::pid_t, libc::SIGTERM) };
    assert_eq!(
        ret,
        0,
        "kill(SIGTERM) failed: {}",
        std::io::Error::last_os_error()
    );
}

/// The recorder is the `cerulion bagd` SUBCOMMAND —
/// every test spawns the real `cerulion` binary with `bagd` prepended.
fn bagd_cmd() -> Command {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.arg("bagd");
    cmd
}

#[test]
#[serial]
fn sigterm_finalizes_bag_from_the_real_binary_subprocess() {
    const K: usize = 8;
    let (mgr, ix_json) = make_manager_with_json(16);
    let topic = unique_topic("subproc");
    let out = unique_out("subproc");
    let ready = unique_out("subproc_ready");
    let _ = std::fs::remove_file(&ready);

    // Parent creates the service FIRST so the child's open-only tap can attach.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 3, K + 16, 512);

    // Serialize the shared iceoryx2 config to a temp file for the child.
    let cfg_path: PathBuf = unique_out("subproc_cfg");
    {
        let mut f = std::fs::File::create(&cfg_path).expect("create cfg file");
        f.write_all(ix_json.as_bytes()).expect("write cfg");
    }

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--ready-file")
        .arg(&ready)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .arg("--flush-interval-ms")
        .arg("20")
        .arg("--schema-wait-timeout-ms")
        .arg("400")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    // Wait for the child to arm its tap.
    assert!(
        wait_for_file(&ready, Duration::from_secs(15)),
        "child never wrote the ready-file"
    );
    settle();

    // Publish K distinct frames from the parent.
    let mut expect: Vec<Vec<u8>> = Vec::with_capacity(K);
    for i in 0..K {
        let frame = build_frame(
            0x5162 + i as u64,
            i as u32,
            100 + i as u64,
            &[(i as u8) + 1; 24],
        );
        pubr.publish_raw(&frame).expect("publish");
        expect.push(frame);
    }
    settle();
    settle(); // give the child time to drain them

    // SIGTERM → the child's ctrlc(termination) handler flips the shutdown flag.
    send_sigterm(&guard.0);

    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound after SIGTERM");
    assert!(status.success(), "child exited non-zero: {status:?}");

    // Read the finalized bag: it must be Finalized with all K frames.
    let reader = BagReader::open(&out).expect("open bag from subprocess");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "SIGTERM must finalize the bag, got {completeness:?}"
    );
    let got: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(got.len(), K, "all K frames recorded by the subprocess");
    assert_eq!(
        got, expect,
        "subprocess-recorded frames byte-match published"
    );

    drop(guard);
    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_file(&cfg_path);
    drop(pubr);
    drop(mgr);
}

/// The headline: the EXACT-mode (replay-grade) recorder contract
/// over the REAL `bagd` binary. Spawns bagd with `--topics-json` (the exact
/// per-topic schema), a scheduler-trace `--ring`, `--attach graph.yaml:...`, and
/// `--attach env.json:...`; waits for the ready handshake; publishes a frame
/// AFTER ready; SIGTERMs; then asserts the finalized bag carries (a) the EXACT
/// channel descriptor (real name + hash + wire_fixed_size, NOT the attach-mode
/// `"unknown"`/0 placeholder), (b) both metadata attachments verbatim, and (c)
/// the published frame — the frame published AFTER ready proving the taps-ready
/// handshake preceded the first publish (step 0 is always in the bag).
#[test]
#[serial]
fn exact_mode_topics_json_records_real_schema_ring_and_attachments() {
    use cerulion_bagd::RecordedTopic;
    use cerulion_core::trace_ring::{default_capacity_records, TraceRingOwner};

    let (mgr, ix_json) = make_manager_with_json(16);
    let topic = unique_topic("exact");
    let out = unique_out("exact");
    let ready = unique_out("exact_ready");
    let _ = std::fs::remove_file(&ready);

    // The graph-owned service (recorder taps it open-only). Provision the
    // recording borrow headroom (mirrors the runtime's --record raise to 8).
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 8, 16, 512);

    // A scheduler-trace ring (rank 0). The scheduler has no producer hook yet,
    // so it stays EMPTY: bagd opens + drains it (0 records)
    // and writes its manifest attachment. Held to the end (Drop unlinks it).
    let ring = TraceRingOwner::create(
        &format!("cer_exact_{}", std::process::id()),
        default_capacity_records(),
        0,
        &["nodeA", "nodeB"],
    )
    .expect("create trace ring");

    // The exact-mode tap set (the CLI writes this JSON).
    const SCHEMA_NAME: &str = "sensor_msgs/Image";
    const SCHEMA_HASH: u64 = 0x0102_0304_0506_0708;
    const WIRE_FIXED: u32 = 40;
    let recorded = vec![RecordedTopic {
        format: cerulion_bagd::RECORDED_TOPIC_FORMAT,
        topic: topic.clone(),
        schema_name: SCHEMA_NAME.to_string(),
        schema_hash: SCHEMA_HASH,
        wire_fixed_size: WIRE_FIXED,
        multi_publisher: false,
    }];
    let topics_json = unique_out("exact_topics");
    std::fs::write(&topics_json, serde_json::to_vec(&recorded).unwrap())
        .expect("write topics-json");

    // The two metadata attachments.
    let graph_yaml = unique_out("exact_graph");
    let graph_bytes = b"name: exact\nnodes: []\n";
    std::fs::write(&graph_yaml, graph_bytes).expect("write graph.yaml");
    let env_json = unique_out("exact_env");
    let env_bytes = br#"{"CERULION_TEST":"1"}"#;
    std::fs::write(&env_json, env_bytes).expect("write env.json");

    let cfg_path: PathBuf = unique_out("exact_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topics-json")
        .arg(&topics_json)
        .arg("--ring")
        .arg(ring.name())
        .arg("--ready-file")
        .arg(&ready)
        .arg("--attach")
        .arg(format!("graph.yaml:{}", graph_yaml.display()))
        .arg("--attach")
        .arg(format!("env.json:{}", env_json.display()))
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .arg("--flush-interval-ms")
        .arg("20")
        .arg("--schema-wait-timeout-ms")
        .arg("400")
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    // HANDSHAKE: taps + ring armed BEFORE step 0 is published.
    assert!(
        wait_for_file(&ready, Duration::from_secs(15)),
        "bagd never wrote the ready-file"
    );
    settle();

    // Publish ONE frame AFTER ready (proving the tap was live before it).
    let frame = build_frame(SCHEMA_HASH, 7, 12_345, &[0xAB; 24]);
    pubr.publish_raw(&frame).expect("publish");
    settle();
    settle();

    send_sigterm(&guard.0);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound after SIGTERM");
    assert!(status.success(), "child exited non-zero: {status:?}");

    let reader = BagReader::open(&out).expect("open exact-mode bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "SIGTERM must finalize the bag, got {completeness:?}"
    );

    // (a) EXACT channel descriptor — real name + hash + wire_fixed_size.
    let channels = reader.channels().expect("channels");
    let ch = channels
        .iter()
        .find(|c| c.topic == topic)
        .expect("the recorded topic must have a channel");
    assert_eq!(
        ch.schema_name, SCHEMA_NAME,
        "exact-mode records the REAL schema name"
    );
    let desc = ch.descriptor.expect("cerulion descriptor");
    assert_eq!(desc.schema_hash, SCHEMA_HASH, "exact schema hash");
    assert_eq!(desc.wire_fixed_size, WIRE_FIXED, "exact wire_fixed_size");

    // (b) both metadata attachments present + verbatim.
    let ga = reader
        .attachment("graph.yaml")
        .expect("attachments")
        .expect("graph.yaml attachment present");
    assert_eq!(ga.data, graph_bytes, "graph.yaml embedded verbatim");
    let ea = reader
        .attachment("env.json")
        .expect("attachments")
        .expect("env.json attachment present");
    assert_eq!(ea.data, env_bytes, "env.json embedded verbatim");
    // The trace ring's manifest attachment is present too (rank 0).
    assert!(
        reader
            .attachment("__cerulion/trace_manifest_rank0.json")
            .expect("attachments")
            .is_some(),
        "the trace-ring manifest attachment must be present"
    );

    // (c) the frame published AFTER ready IS in the bag (handshake ordering).
    let got: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        got,
        vec![frame],
        "the post-ready frame must be recorded (bag preceded step 0)"
    );

    drop(guard);
    cleanup(&out);
    for f in [&ready, &topics_json, &graph_yaml, &env_json, &cfg_path] {
        let _ = std::fs::remove_file(f);
    }
    drop(ring);
    drop(pubr);
    drop(mgr);
}

/// `--topics-json` pointing at MALFORMED JSON is a loud
/// non-zero exit (before recording starts) — bagd must not silently record an
/// empty/attach-mode bag on a corrupt tap set.
#[test]
#[serial]
fn topics_json_malformed_exits_nonzero() {
    let (_mgr, ix_json) = make_manager_with_json(16);
    let out = unique_out("badjson");
    let bad = unique_out("badjson_topics");
    std::fs::write(&bad, b"{ this is not valid json ]").expect("write bad json");
    let cfg_path = unique_out("badjson_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let mut child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topics-json")
        .arg(&bad)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let status = wait_bounded(&mut child, Duration::from_secs(15))
        .expect("bagd must exit promptly on malformed --topics-json");
    assert!(
        !status.success(),
        "malformed --topics-json must exit non-zero"
    );

    for f in [&out, &bad, &cfg_path] {
        let _ = std::fs::remove_file(f);
    }
}

/// An unreadable `--schema-catalog` is a LOUD startup failure naming
/// the flag — never a recording that silently lacks its provenance.
///
/// The decision this pins is the CALL SITE's, not the loader's: the pure loader
/// arms (`cerulion_bagd::schema_catalog_arg_tests`) assert it returns
/// `Err`, but a `.ok()` at the call site would swallow that and record a bag
/// with no `__cerulion/schemas.json` — the exact success-shaped failure the schema
/// attachment exists to close, and invisible to every other test (the CLI never writes a
/// corrupt catalog, so only a direct `cerulion bagd` invocation can produce
/// one). Stderr is asserted as well as the exit code: a nonzero exit alone
/// would also be satisfied by an unrelated argument error.
#[test]
#[serial]
fn schema_catalog_malformed_exits_nonzero_naming_the_flag() {
    let (mgr, ix_json) = make_manager_with_json(16);
    let out = unique_out("badcat");
    // A LIVE topic, so the run is refused for the catalog and nothing else: the
    // catalog is read while the config is built, BEFORE any tap opens, and this
    // publisher makes even that later step healthy.
    let topic = unique_topic("badcat");
    let _pubr = publisher_with_provisioning(&mgr, &topic, 3, 24, 512);
    let bad = unique_out("badcat_catalog");
    std::fs::write(&bad, b"@@@ not a catalog").expect("write bad catalog");
    let cfg_path = unique_out("badcat_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");
    let err_path = unique_out("badcat_stderr");

    let mut child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--schema-catalog")
        .arg(&bad)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .stdout(Stdio::null())
        .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
        .spawn()
        .expect("spawn bagd");
    let status = wait_bounded(&mut child, Duration::from_secs(15))
        .expect("bagd must exit promptly on a malformed --schema-catalog");
    assert!(
        !status.success(),
        "a malformed --schema-catalog must exit non-zero, got {status:?}"
    );
    let err = std::fs::read_to_string(&err_path).unwrap_or_default();
    // The LOADER's own words, not just the flag token. `--schema-catalog` alone
    // is satisfied by clap's unknown-argument error ("a similar argument exists:
    // '--schema-catalog'"), so deleting the arg entirely would still pass
    // a bare-token predicate: nonzero exit, flag echoed, no bag written.
    assert!(
        err.contains("--schema-catalog") && err.contains("is not a readable schema catalog"),
        "the failure must be the CATALOG LOADER refusing this file, naming it — not any \
         other nonzero exit that happens to echo the flag; stderr was:\n{err}"
    );
    assert!(
        !out.exists(),
        "no bag may be written on the refused-catalog path"
    );

    for f in [&out, &bad, &cfg_path, &err_path] {
        let _ = std::fs::remove_file(f);
    }
}

/// `--topic` and `--topics-json` are mutually exclusive (clap
/// `conflicts_with`) — supplying both is a usage error (non-zero exit).
#[test]
#[serial]
fn topic_and_topics_json_conflict_exits_nonzero() {
    let out = unique_out("conflict");
    let topics = unique_out("conflict_topics");
    std::fs::write(&topics, b"[]").expect("write topics");
    let mut child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg("/x")
        .arg("--topics-json")
        .arg(&topics)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let status = wait_bounded(&mut child, Duration::from_secs(10))
        .expect("bagd must reject the conflicting flags promptly");
    assert!(
        !status.success(),
        "--topic + --topics-json must be a clap conflict (non-zero exit)"
    );
    for f in [&out, &topics] {
        let _ = std::fs::remove_file(f);
    }
}

/// The HARD handshake pin: the FIRST frame,
/// published IMMEDIATELY on ready-file observation with NO settle() grace, is
/// in the bag. The publisher pre-exists (created before spawn), so bagd's
/// subscriber connection is established inside `Recorder::setup` — BEFORE the
/// ready-file write. A regression that writes the ready file before arming the
/// taps (moving the write above `Recorder::setup`) makes this frame
/// undeliverable and fails here; the settle()-softened sibling above would let
/// it slip on any developer machine.
#[test]
#[serial]
fn first_frame_published_immediately_on_ready_is_in_the_bag() {
    let (mgr, ix_json) = make_manager_with_json(16);
    let topic = unique_topic("step0");
    let out = unique_out("step0");
    let ready = unique_out("step0_ready");
    let _ = std::fs::remove_file(&ready);

    // The service exists BEFORE spawn — bagd's open-only tap connects at its
    // subscriber creation (inside setup), making an immediately-published
    // frame deliverable iff setup preceded the ready write.
    let mut pubr = publisher_with_provisioning(&mgr, &topic, 3, 16, 256);

    let cfg_path: PathBuf = unique_out("step0_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--ready-file")
        .arg(&ready)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .arg("--flush-interval-ms")
        .arg("20")
        .arg("--schema-wait-timeout-ms")
        .arg("400")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(15)),
        "child never wrote the ready-file"
    );
    // NO settle() — publish frame 0 IMMEDIATELY on observing ready.
    let frame = build_frame(0x57E9, 0, 42, &[0xF0; 16]);
    pubr.publish_raw(&frame).expect("publish frame 0");

    // Give the child time to DRAIN (post-publish grace is fine — the pin is
    // that the tap was armed BEFORE ready, i.e. before the publish).
    settle();
    settle();
    send_sigterm(&guard.0);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound after SIGTERM");
    assert!(status.success(), "child exited non-zero: {status:?}");

    let reader = BagReader::open(&out).expect("open bag");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(completeness.is_finalized(), "got {completeness:?}");
    let got: Vec<Vec<u8>> = msgs
        .into_iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.data)
        .collect();
    assert_eq!(
        got,
        vec![frame],
        "the FIRST frame (published with zero grace after ready) must be in the bag — \
         taps are armed strictly BEFORE the ready-file write"
    );

    drop(guard);
    cleanup(&out);
    for f in [&ready, &cfg_path] {
        let _ = std::fs::remove_file(f);
    }
    drop(pubr);
    drop(mgr);
}

/// Linux-only (`#[ignore]`): the memcpy-interposition zero-copy proof.
///
/// The recorder's headline claim is that a tapped SHM payload rides to
/// `writev(2)` with ZERO userspace copies. Proving it requires interposing on
/// `memcpy`/`memmove` (an `LD_PRELOAD` shim or a `perf`/`bpftrace` uprobe) while
/// `bagd` records a large-payload topic and asserting the payload bytes are
/// NEVER copied between the SHM slot and the kernel — an environment-specific
/// harness that cannot run in the portable CI matrix. Run it separately on Linux
/// with interposition tooling; this placeholder pins the intent + keeps the
/// test name discoverable.
#[test]
#[ignore = "box-only: needs memcpy/memmove interposition (LD_PRELOAD / bpftrace) on a Linux box"]
fn box_only_memcpy_interposition_proves_zero_copy_write_path() {
    // Intentionally empty: the real proof runs under an interposition harness on
    // Linux, not in CI. See the doc comment above.
}

// ============================================================
// SIGTERM before ANY message: the shutdown pass must still
// create (placeholder channels) + finalize a bag and exit 0.
// ============================================================

#[test]
#[serial]
fn sigterm_with_zero_publishes_finalizes_placeholder_bag() {
    let (mgr, ix_json) = make_manager_with_json(16);
    let topic = unique_topic("zeropub");
    let out = unique_out("zeropub");
    let ready = unique_out("zeropub_ready");
    let _ = std::fs::remove_file(&ready);

    // The topic's service must EXIST (open-only tap) but never speaks.
    let _pubr = publisher(&mgr, &topic, 256);

    let cfg_path: PathBuf = unique_out("zeropub_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--ready-file")
        .arg(&ready)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .arg("--flush-interval-ms")
        .arg("20")
        .arg("--schema-wait-timeout-ms")
        .arg("400")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(15)),
        "child never wrote the ready-file"
    );
    // SIGTERM immediately — nothing was ever published.
    send_sigterm(&guard.0);
    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound");
    assert!(status.success(), "child exited non-zero: {status:?}");

    // A FINALIZED bag exists with the placeholder channel and zero messages.
    assert!(
        out.exists(),
        "the bag file must exist even with zero messages"
    );
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "zero-message SIGTERM must still finalize, got {completeness:?}"
    );
    assert!(
        msgs.iter().all(|m| m.topic != topic),
        "no data messages were published"
    );
    let chan = reader
        .channels()
        .expect("channels")
        .into_iter()
        .find(|c| c.topic == topic)
        .expect("the silent tap still gets a channel");
    assert_eq!(chan.schema_name, "unknown", "placeholder name");
    let desc = chan.descriptor.expect("descriptor");
    assert_eq!(desc.schema_hash, 0, "placeholder hash 0");
    assert_eq!(desc.wire_fixed_size, 0, "placeholder size 0");

    drop(guard);
    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_file(&cfg_path);
    drop(mgr);
}

// ============================================================
// Frames published CONCURRENT with SIGTERM (no settle): exit 0,
// Finalized, and whatever was recorded is a monotonic FIFO sequence (no
// stronger lost-tail claim — the race window is the point).
// ============================================================

#[test]
#[serial]
fn sigterm_concurrent_with_publish_burst_finalizes_monotonic() {
    const K: usize = 32;
    let (mgr, ix_json) = make_manager_with_json(16);
    let topic = unique_topic("race");
    let out = unique_out("race");
    let ready = unique_out("race_ready");
    let _ = std::fs::remove_file(&ready);

    let mut pubr = publisher_with_provisioning(&mgr, &topic, 3, K + 16, 256);

    let cfg_path: PathBuf = unique_out("race_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--ready-file")
        .arg(&ready)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .arg("--flush-interval-ms")
        .arg("20")
        .arg("--schema-wait-timeout-ms")
        .arg("400")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    assert!(
        wait_for_file(&ready, Duration::from_secs(15)),
        "child never wrote the ready-file"
    );
    settle(); // tap armed; do NOT settle after the burst — the race is the test

    // Publish the burst and SIGTERM immediately (mid-drain in the child).
    for i in 0..K {
        let frame = build_frame(0xACE0, i as u32, i as u64, &[(i % 251) as u8; 16]);
        pubr.publish_raw(&frame).expect("publish");
    }
    send_sigterm(&guard.0);

    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound");
    assert!(status.success(), "child exited non-zero: {status:?}");

    // Finalized; the recorded stream is a monotonically increasing wire-seq
    // subsequence of the burst (delivery is FIFO; how far the tail got before
    // the final drain is the race — no exact-count claim).
    let reader = BagReader::open(&out).expect("open");
    let (msgs, completeness) = reader.recover_messages().expect("recover");
    assert!(
        completeness.is_finalized(),
        "concurrent SIGTERM must still finalize, got {completeness:?}"
    );
    let seqs: Vec<u32> = msgs
        .iter()
        .filter(|m| m.topic == topic)
        .map(|m| m.sequence)
        .collect();
    assert!(
        seqs.windows(2).all(|w| w[0] < w[1]),
        "recorded wire sequences must be strictly increasing (FIFO), got {seqs:?}"
    );
    assert!(
        seqs.len() <= K,
        "cannot record more than was published, got {}",
        seqs.len()
    );

    drop(guard);
    cleanup(&out);
    let _ = std::fs::remove_file(&ready);
    let _ = std::fs::remove_file(&cfg_path);
    drop(mgr);
}

// ============================================================
// A missing tapped topic makes the BINARY exit non-zero (the
// library error propagates through main's `?` to ExitCode::FAILURE).
// ============================================================

#[test]
#[serial]
fn missing_topic_makes_binary_exit_nonzero() {
    let (_mgr, ix_json) = make_manager_with_json(16);
    // NO publisher — the topic's services do not exist in this namespace.
    let topic = unique_topic("gone");
    let out = unique_out("gone");

    let cfg_path: PathBuf = unique_out("gone_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound");
    assert!(
        !status.success(),
        "a missing tapped topic must exit non-zero, got {status:?}"
    );
    assert!(!out.exists(), "no bag file on the missing-topic path");

    drop(guard);
    let _ = std::fs::remove_file(&cfg_path);
}

// ============================================================
// Binary twin: a garbage --test-iox2-config file exits
// non-zero (the InvalidTransportConfig propagates; the process never joins
// the default global namespace silently).
// ============================================================

#[test]
#[serial]
fn garbage_iox2_config_file_makes_binary_exit_nonzero() {
    let topic = unique_topic("badcfg");
    let out = unique_out("badcfg");
    let cfg_path: PathBuf = unique_out("badcfg_cfg");
    std::fs::write(&cfg_path, b"definitely not an iceoryx2 config {").expect("write cfg");

    let child = bagd_cmd()
        .arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn bagd");
    let mut guard = ChildGuard(child);

    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound");
    assert!(
        !status.success(),
        "a garbage iceoryx2 config must exit non-zero, got {status:?}"
    );
    assert!(!out.exists(), "no bag file on the bad-config path");

    drop(guard);
    let _ = std::fs::remove_file(&cfg_path);
}

// ============================================================
// A SET-but-invalid RUST_LOG eprintlns the
// degradation to stderr before falling back to the default filter (an unset
// RUST_LOG stays quiet). Uses the fast missing-topic exit path.
// ============================================================

/// Shared body: spawn bagd on a missing topic (fast non-zero exit) with the
/// given RUST_LOG treatment, under the harness's ChildGuard + bounded-wait
/// discipline (no unbounded `.output()`), and return stderr.
fn run_missing_topic_capture_stderr(rust_log: Option<&str>) -> String {
    let (_mgr, ix_json) = make_manager_with_json(16);
    let topic = unique_topic("badlog");
    let out = unique_out("badlog");
    let cfg_path: PathBuf = unique_out("badlog_cfg");
    std::fs::write(&cfg_path, ix_json.as_bytes()).expect("write cfg");

    let mut cmd = bagd_cmd();
    cmd.arg("--out")
        .arg(&out)
        .arg("--topic")
        .arg(&topic)
        .arg("--test-iox2-config")
        .arg(&cfg_path)
        .arg("--status-period-ms")
        .arg("0")
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    match rust_log {
        Some(v) => {
            cmd.env("RUST_LOG", v);
        }
        None => {
            cmd.env_remove("RUST_LOG");
        }
    }
    let child = cmd.spawn().expect("spawn bagd");
    let mut guard = ChildGuard(child);

    let status = wait_bounded(&mut guard.0, Duration::from_secs(15))
        .expect("child did not exit within the bound");
    assert!(
        !status.success(),
        "the missing topic still governs the exit code: {status:?}"
    );
    let mut stderr = String::new();
    use std::io::Read as _;
    guard
        .0
        .stderr
        .take()
        .expect("piped stderr")
        .read_to_string(&mut stderr)
        .expect("read stderr");
    assert!(!out.exists(), "no bag file on the missing-topic path");
    let _ = std::fs::remove_file(&cfg_path);
    stderr
}

#[test]
#[serial]
fn invalid_rust_log_eprintlns_to_stderr() {
    // Typo'd level → EnvFilter parse error → the eprintln fires.
    let stderr = run_missing_topic_capture_stderr(Some("cerulion=trce"));
    assert!(
        stderr.contains("invalid RUST_LOG"),
        "a set-but-invalid RUST_LOG must be eprintln'd, stderr was: {stderr}"
    );
}

/// Negative twin: an UNSET RUST_LOG errors the EnvFilter builder too, but must
/// stay quiet (the var_os guard) — a mutation warning on unset fails here.
#[test]
#[serial]
fn unset_rust_log_stays_quiet_on_stderr() {
    let stderr = run_missing_topic_capture_stderr(None);
    assert!(
        !stderr.contains("invalid RUST_LOG"),
        "an UNSET RUST_LOG must not eprintln the invalid-filter warning, stderr was: {stderr}"
    );
}
