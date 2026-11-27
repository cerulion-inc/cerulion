// SPDX-License-Identifier: AGPL-3.0-only
//! **A free run whose process hosts several executor units takes a Flashback
//! checkpoint in EVERY unit**, over the real binary.
//!
//! A checkpoint of a single-unit worker forks its process. A process that runs
//! several units on several threads cannot be forked safely (the child copies one
//! thread, and a lock a sibling unit holds stays held in the copy), so each unit of
//! such a process captures on its own thread instead. This file proves the whole
//! chain reaches a user: the supervisor arms the plane for a split deployment,
//! every unit creates its own state ring, and a capture taken with
//! `cerulion flashback` carries a complete checkpoint for every rank.
//!
//! # The fixture, and the hand-written oracle
//!
//! `mpdemo` is the 3-node chain `ticker -> relay -> sink` in two process groups,
//! `p0: [ticker, relay]` and `p1: [sink]`. `relay` declares `isolate: true`, so
//! with no cost profile `p0` runs as two executor units (`p0.ticker` and
//! `p0.relay`) in one process and `p1` runs as one unit in its own. Three units,
//! one node each, so three ranks:
//!
//! * the run log has two "state arm attached without fork" lines (the two units
//!   of `p0`) and three "checkpoint state ring created" lines (one per rank);
//! * the capture's `anchor.per_rank` has exactly the keys `0`, `1` and `2`, each
//!   with `nodes == 1` and `complete == 1`, and `anchor.missing_ranks` is empty;
//! * the run is younger than the 30 s back window, so the capture holds every
//!   rank's step 0 and reads `resimmable: true`, and `cerulion bag play --resim
//!   all` re-executes it from the start with exit 0.
//!
//! The positive control on the log is the supervisor's "capture plane ARMED for
//! this deployment" line, so an empty capture of stderr cannot pass the negative
//! ones: no process logs that it takes no checkpoints ("flashback checkpoints
//! are OFF", the kill switch's "Flashback is OFF for this run", or a rank's own
//! "Flashback STATE capture is OFF").
//!
//! A capture whose window begins mid run is not taken here: a mid-run free-run
//! recording of several worker ranks is refused, since per-rank resume is not
//! supported.
//!
//! Hermetic: `CERULION_NETWORK=off`, an owned `CERULION_HOME` and
//! `CERULION_FLASHBACK_DIR`, the execution mode REMOVED from the child's
//! environment (the free-run default). `#[serial]`: the data plane and the
//! `/__cerulion/flashback` trigger channel are on the default iceoryx2 namespace.
//!
//! Prerequisites (the helpers PANIC with the exact instruction if missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! ```bash
//! cargo test -p cerulion_cli --test flashback_split_units_e2e_test -- --test-threads=1
//! ```

#![cfg(unix)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_core::flashback::{FLASHBACK_CADENCE_MS_ENV, FLASHBACK_ENV};
use serial_test::serial;

mod mp_support;
use mp_support::*;

/// Liveness ceilings, in seconds. Load can only delay these.
const READY_DEADLINE: Duration = Duration::from_secs(90);
/// The capture verb waits out the shipped post window (15 s) and then the bag
/// write, so its ceiling is generous by a wide margin.
const CAPTURE_COMPLETES: Duration = Duration::from_secs(240);
const EXIT_DEADLINE: Duration = Duration::from_secs(90);
const RESIM_COMPLETES: Duration = Duration::from_secs(240);

/// The run anchors every 2 s, so the 15 s post window holds several checkpoints
/// of every rank.
const ANCHOR_CADENCE_MS: &str = "2000";

/// The window recorder is up and holding.
const WINDOW_HELD: &str = "flashback: holding a rolling window";

/// Strip ANSI SGR escapes: the default formatter wraps each field name and its
/// `=` in their own escapes, so `key=value` is not a substring of the raw line.
fn strip_ansi(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == 0x1b && i + 1 < bytes.len() && bytes[i + 1] == b'[' {
            i += 2;
            while i < bytes.len() && !(0x40..=0x7e).contains(&bytes[i]) {
                i += 1;
            }
            i += 1;
            continue;
        }
        out.push(bytes[i] as char);
        i += 1;
    }
    out
}

/// Bounded poll for `needle` in the run's log; a run that exits first fails here.
fn wait_for_line(guard: &mut ChildGuard, stdout: &Path, stderr: &Path, needle: &str) {
    let start = Instant::now();
    loop {
        if read_file(stdout).contains(needle) || read_file(stderr).contains(needle) {
            return;
        }
        if let Ok(Some(status)) = guard.try_wait_noting() {
            panic!(
                "graph run EXITED ({status:?}) before logging {needle:?}; stderr:\n{}",
                read_file(stderr)
            );
        }
        assert!(
            start.elapsed() < READY_DEADLINE,
            "log line {needle:?} not seen within {READY_DEADLINE:?}; stderr so far:\n{}",
            read_file(stderr)
        );
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Every `.mcap` directly under `dir`, sorted.
fn mcaps(dir: &Path) -> Vec<PathBuf> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok().map(|e| e.path()))
                .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("mcap"))
                .collect()
        })
        .unwrap_or_default();
    out.sort();
    out
}

/// The capture's own manifest, `__cerulion/flashback.json`.
fn capture_manifest(capture: &Path) -> serde_json::Value {
    let reader = cerulion_bag::BagReader::open(capture).expect("open the capture");
    serde_json::from_slice(
        &reader
            .attachment("__cerulion/flashback.json")
            .expect("read the capture manifest")
            .expect("a capture carries its own manifest")
            .data,
    )
    .expect("the capture manifest is valid JSON")
}

#[test]
#[serial]
fn a_process_that_hosts_several_units_takes_a_checkpoint_in_every_unit() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let root = tmp.path();
    let prefix = format!("fbsplit{}", std::process::id());
    build_mp_workspace(root, &prefix);
    isolate_nodes(root, "mpdemo", &["relay"]);
    let home = root.join("cerhome");
    let flashbacks = root.join("flashbacks");
    std::fs::create_dir_all(&home).unwrap();
    std::fs::create_dir_all(&flashbacks).unwrap();
    let home_str = home.to_str().expect("tempdir paths are UTF-8").to_owned();
    let flashbacks_str = flashbacks
        .to_str()
        .expect("tempdir paths are UTF-8")
        .to_owned();

    let (mut run, stdout_path, stderr_path) = spawn_graph_run_graph(
        root,
        "mpdemo",
        false,
        &[],
        &[
            ("CERULION_HOME", &home_str),
            ("CERULION_FLASHBACK_DIR", &flashbacks_str),
            // SET, never inherited: a developer's `CERULION_FLASHBACK=off` would
            // turn every arm of this file into a refusal.
            (FLASHBACK_ENV, "on"),
            (FLASHBACK_CADENCE_MS_ENV, ANCHOR_CADENCE_MS),
            (
                "RUST_LOG",
                "cerulion=info,cerulion_core=info,cerulion_cli_engine=info,cerulion_bagd=info",
            ),
        ],
        SpawnExecutionMode::Default,
    );
    let _bagd_guard = BagdGuard::arm();
    wait_for_line(
        &mut run,
        &stdout_path,
        &stderr_path,
        "GO signaled; deployment live",
    );
    wait_for_line(&mut run, &stdout_path, &stderr_path, WINDOW_HELD);

    // CAPTURE, through the operator's own verb. It waits out the post window
    // and prints the finished verdict, so its exit is the capture's.
    let mut flash = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["flashback", "--note", "split units probe"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_HOME", &home)
            .env("CERULION_FLASHBACK_DIR", &flashbacks)
            .stdout(Stdio::from(
                std::fs::File::create(root.join("flash.stdout")).unwrap(),
            ))
            .stderr(Stdio::from(
                std::fs::File::create(root.join("flash.stderr")).unwrap(),
            ))
            .spawn()
            .expect("spawn cerulion flashback"),
    );
    let flash_status = flash
        .wait_bounded(CAPTURE_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion flashback` never returned"));
    flash.finish().assert_clean();
    assert!(
        flash_status.success(),
        "`cerulion flashback` must capture: {flash_status:?}\nstdout:\n{}\nstderr:\n{}\n\
         run log:\n{}",
        read_file(&root.join("flash.stdout")),
        read_file(&root.join("flash.stderr")),
        read_file(&stderr_path)
    );

    // Reap first, then signal: a run that exited on its own must fail here, not
    // pass a signal that landed on a zombie.
    assert!(
        run.try_wait_noting()
            .expect("polling the graph run must not fail")
            .is_none(),
        "the graph run exited on its own during the capture\nstderr:\n{}",
        read_file(&stderr_path)
    );
    signal_live_process(run.id(), libc::SIGINT);
    let status = run
        .wait_bounded(EXIT_DEADLINE)
        .expect("graph run did not exit after SIGINT");
    assert!(
        status.success(),
        "graph run must exit 0 on SIGINT, got {status:?}\nstderr:\n{}",
        read_file(&stderr_path)
    );
    run.finish().assert_clean();
    let log = strip_ansi(&format!(
        "{}\n{}",
        read_file(&stdout_path),
        read_file(&stderr_path)
    ));

    // ------------------------------------------------------------ the run log
    assert!(
        log.contains("Flashback capture plane ARMED for this deployment"),
        "POSITIVE CONTROL: the supervisor must arm the plane for a split deployment; \
         log:\n{log}"
    );
    // Every sentence that says a process takes no checkpoints.
    for off in [
        "flashback checkpoints are OFF",
        "Flashback is OFF for this run",
        "Flashback STATE capture is OFF",
    ] {
        assert!(
            !log.contains(off),
            "no process of a split deployment may turn its checkpoints off ({off:?}); \
             log:\n{log}"
        );
    }
    assert_eq!(
        log.matches("state arm attached without fork").count(),
        2,
        "the two units of `p0` share a process, so each is armed without fork; log:\n{log}"
    );
    assert_eq!(
        log.matches("checkpoint state ring created").count(),
        3,
        "every one of the three units creates its own state ring; log:\n{log}"
    );

    // ------------------------------------------------------- the capture's claim
    let captures = mcaps(&flashbacks);
    assert_eq!(
        captures.len(),
        1,
        "exactly one capture in this run's own directory, got {captures:?}"
    );
    let manifest = capture_manifest(&captures[0]);
    let anchor = &manifest["anchor"];
    let per_rank: BTreeMap<String, (u64, u64)> = anchor["per_rank"]
        .as_object()
        .unwrap_or_else(|| panic!("the capture must embed a per-rank anchor block: {manifest}"))
        .iter()
        .map(|(rank, entry)| {
            let count = |key: &str| {
                entry[key]
                    .as_u64()
                    .unwrap_or_else(|| panic!("rank {rank} `{key}` is a count: {entry}"))
            };
            (rank.clone(), (count("nodes"), count("complete")))
        })
        .collect();
    let want: BTreeMap<String, (u64, u64)> = [
        ("0".to_string(), (1, 1)),
        ("1".to_string(), (1, 1)),
        ("2".to_string(), (1, 1)),
    ]
    .into_iter()
    .collect();
    assert_eq!(
        per_rank, want,
        "every unit (three ranks, one node each) must carry a complete checkpoint: {manifest}"
    );
    assert_eq!(
        anchor["missing_ranks"],
        serde_json::json!([]),
        "no rank may be missing from the checkpoint: {manifest}"
    );
    // The run is younger than the 30 s back window, so the capture still holds
    // every rank's step 0 and replays from the start, beside its checkpoints.
    assert_eq!(
        anchor["resimmable"],
        serde_json::json!(true),
        "a capture whose window reaches step 0 on every rank replays from the start: \
         {manifest}"
    );
    let reason = anchor["resimmable_reason"]
        .as_str()
        .unwrap_or_else(|| panic!("a verdict states its reason: {manifest}"));
    assert!(
        reason.contains("step 0"),
        "the verdict names the window: {reason}"
    );

    // ------------------------------------------------- the verdict, replayed
    // The replay the verdict predicts: from the start, reading no checkpoint.
    let resim_stderr = root.join("resim.stderr");
    let mut resim = ChildGuard::single_process(
        Command::new(env!("CARGO_BIN_EXE_cerulion"))
            .args(["bag", "play"])
            .arg(&captures[0])
            .args(["--resim", "all"])
            .current_dir(root)
            .env_remove("CARGO_TARGET_DIR")
            .env("CERULION_NETWORK", "off")
            .env("CERULION_HOME", &home)
            .stdout(Stdio::from(
                std::fs::File::create(root.join("resim.stdout")).unwrap(),
            ))
            .stderr(Stdio::from(std::fs::File::create(&resim_stderr).unwrap()))
            .spawn()
            .expect("spawn cerulion bag play --resim"),
    );
    let resim_status = resim
        .wait_bounded(RESIM_COMPLETES)
        .unwrap_or_else(|| panic!("`cerulion bag play --resim` never returned"));
    resim.finish().assert_clean();
    assert_eq!(
        resim_status.code(),
        Some(0),
        "`cerulion bag play --resim all` re-executes the capture from the start\nstderr:\n{}",
        read_file(&resim_stderr)
    );
}
