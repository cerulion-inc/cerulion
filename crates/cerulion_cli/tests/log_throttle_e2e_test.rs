// SPDX-License-Identifier: AGPL-3.0-only
//! The throttled and once-only log macros through the REAL `cerulion` binary,
//! in a cdylib node, over a hand-built workspace: `nodes/chatter/` (a copy of
//! the `test_node_macro_log_throttle_cdylib` fixture source and its prebuilt
//! cdylib) and `graphs/logthr.yaml`.
//!
//! 1. `graph run logthr --time-source virtual` prints exactly the lines the
//!    fixture's source says it prints (its module doc is the oracle), on
//!    stderr through the node's own subscriber, and two runs print the same
//!    line sequence, graph timestamps included.
//! 2. `graph run logthr --record` writes a bag of that same run, the recorded
//!    run prints the same lines, and `bag play --resim all --verify` on the bag
//!    exits 0.
//!
//! The fixture stops the graph itself on its 200th tick, so no run here needs
//! a signal. `CERULION_NETWORK=off` keeps every run local; `#[serial]` and a
//! unique graph prefix per run keep the default iceoryx2 namespace apart.
//!
//! Prerequisite: `cargo build -p test_node_macro_log_throttle_cdylib`.

#![cfg(unix)]

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant, SystemTime};

use serial_test::serial;

const MS: u64 = 1_000_000;

fn dylib_file(name: &str) -> String {
    if cfg!(target_os = "macos") {
        format!("lib{name}.dylib")
    } else {
        format!("lib{name}.so")
    }
}

fn unique_prefix(tag: &str) -> String {
    let nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .expect("a clock after the epoch")
        .as_nanos();
    format!("logthr{tag}{nanos}")
}

/// A workspace with one `chatter` node: the fixture's source for the
/// workspace walkers and its prebuilt cdylib under the node-type name.
fn build_workspace(root: &Path, prefix: &str) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("nodes/chatter/src")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    std::fs::write(
        root.join("graphs/logthr.yaml"),
        format!(
            "name: logthr\nprefix: {prefix}\nnodes:\n- id: chatter\n  type: chatter\n  inputs: []\n  \
             outputs:\n  - name: cmd\n    schema: geometry_msgs/Vector3\n"
        ),
    )
    .unwrap();
    let fixture_src = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures/test_node_macro_log_throttle_cdylib/src/lib.rs");
    std::fs::copy(&fixture_src, root.join("nodes/chatter/src/lib.rs")).expect("copy fixture src");
    std::fs::copy(
        cerulion_core::testing::find_fixture_cdylib("test_node_macro_log_throttle_cdylib"),
        root.join("target/debug").join(dylib_file("chatter")),
    )
    .expect("copy fixture cdylib");
}

/// What one `cerulion` invocation left behind.
struct Run {
    code: Option<i32>,
    stderr: String,
}

/// Run the `cerulion` binary in `root` to completion, bounded. Output goes to
/// files, so a chatty child can never block on a full pipe.
fn cerulion(root: &Path, args: &[&str]) -> Run {
    let err_path = root.join(format!(
        "cerulion-{}.stderr",
        args.join("_").replace('/', "_")
    ));
    let mut child = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args(args)
        .current_dir(root)
        .env_remove("CARGO_TARGET_DIR")
        .env_remove("RUST_LOG")
        .env("CERULION_NETWORK", "off")
        .env("NO_COLOR", "1")
        .stdout(Stdio::null())
        .stderr(Stdio::from(std::fs::File::create(&err_path).unwrap()))
        .spawn()
        .expect("spawn cerulion");
    let start = Instant::now();
    let status = loop {
        if let Some(status) = child.try_wait().expect("try_wait") {
            break Some(status);
        }
        if start.elapsed() > Duration::from_secs(120) {
            let _ = child.kill();
            let _ = child.wait();
            break None;
        }
        std::thread::sleep(Duration::from_millis(20));
    };
    let stderr = std::fs::read_to_string(&err_path).unwrap_or_default();
    let Some(status) = status else {
        panic!(
            "`cerulion {}` did not finish in 120 s\nstderr:\n{stderr}",
            args.join(" ")
        );
    };
    Run {
        code: status.code(),
        stderr,
    }
}

/// The fixture's lines on `stderr`, each from its level on (the wall-clock
/// timestamp before it differs between runs by design).
fn node_lines(stderr: &str) -> Vec<String> {
    stderr
        .lines()
        .filter(|l| l.contains("log fixture"))
        .map(|l| {
            let start = ["WARN", "INFO"]
                .iter()
                .filter_map(|lvl| l.find(lvl))
                .min()
                .unwrap_or_else(|| panic!("a node line carries its level: {l}"));
            l[start..].to_string()
        })
        .collect()
}

/// `(level, message, tick_ns)` of one normalized line.
fn parse(line: &str) -> (&str, &str, u64) {
    let level = line.split_whitespace().next().unwrap();
    let message = if line.contains("log fixture throttled") {
        "log fixture throttled"
    } else if line.contains("log fixture first tick") {
        "log fixture first tick"
    } else if line.contains("log fixture last tick") {
        "log fixture last tick"
    } else {
        panic!("unexpected node line: {line}")
    };
    let tick_ns = line
        .split_whitespace()
        .find_map(|w| w.strip_prefix("tick_ns="))
        .unwrap_or_else(|| panic!("a node line carries tick_ns: {line}"))
        .parse()
        .unwrap();
    (level, message, tick_ns)
}

/// The fixture's hand oracle (its module doc): on the first tick a throttled
/// warn then the once info; then a throttled warn every 250 ms, 8 in all;
/// then the last tick's info at 1990 ms.
fn assert_matches_fixture_oracle(lines: &[String], context: &str) {
    let parsed: Vec<(&str, &str, u64)> = lines.iter().map(|l| parse(l)).collect();
    let got: Vec<(&str, &str, u64)> = match parsed.first() {
        Some(&(_, _, t0)) => parsed.iter().map(|&(l, m, t)| (l, m, t - t0)).collect(),
        None => panic!("{context}: no node lines at all"),
    };
    let mut want = vec![
        ("WARN", "log fixture throttled", 0),
        ("INFO", "log fixture first tick", 0),
    ];
    for k in 1..8u64 {
        want.push(("WARN", "log fixture throttled", k * 250 * MS));
    }
    want.push(("INFO", "log fixture last tick", 1990 * MS));
    assert_eq!(got, want, "{context}: the node's lines on stderr");
}

/// The shape a live (wall-following) run must have: its graph times are where
/// the ticks landed, and a loaded host can slip or bunch them, so instead of
/// exact times this checks the throttle rule itself against the graph time
/// span the run actually covered (first tick to the last tick's line). The
/// first tick prints the throttled warn then the once info, the last tick
/// prints its info last, every gap between throttled lines is at least the
/// 250 ms period, and there are no more throttled lines than that span holds.
/// The lower bound comes from the last tick: it calls the throttle before its
/// own info line, so if the period had run out since the last throttled line,
/// that call emitted. The last throttled line is therefore under 250 ms of
/// graph time before the last tick, however the ticks before it landed.
fn assert_follows_the_throttle_rule(lines: &[String], context: &str) {
    let parsed: Vec<(&str, &str, u64)> = lines.iter().map(|l| parse(l)).collect();
    let shape: Vec<(&str, &str)> = parsed.iter().map(|&(l, m, _)| (l, m)).collect();
    assert!(
        shape.len() >= 3,
        "{context}: at least the first and last ticks' lines: {lines:?}"
    );
    let n = shape.len();
    assert_eq!(
        shape[..2],
        [
            ("WARN", "log fixture throttled"),
            ("INFO", "log fixture first tick"),
        ],
        "{context}: the first tick's lines: {lines:?}"
    );
    assert_eq!(
        shape[n - 1],
        ("INFO", "log fixture last tick"),
        "{context}: the last tick's line comes last: {lines:?}"
    );
    assert!(
        shape[2..n - 1]
            .iter()
            .all(|&s| s == ("WARN", "log fixture throttled")),
        "{context}: only throttled lines in between: {lines:?}"
    );
    let first_ns = parsed[0].2;
    let last_ns = parsed[n - 1].2;
    let throttled: Vec<u64> = parsed
        .iter()
        .filter(|&&(l, _, _)| l == "WARN")
        .map(|&(_, _, t)| t)
        .collect();
    let most = (last_ns - first_ns) / (250 * MS) + 1;
    assert!(
        throttled.len() as u64 <= most,
        "{context}: {} throttled lines over {} ns of graph time, which holds at most {most}: \
         {lines:?}",
        throttled.len(),
        last_ns - first_ns
    );
    for w in throttled.windows(2) {
        assert!(
            w[1] - w[0] >= 250 * MS,
            "{context}: throttled lines {} ns apart, under the period: {lines:?}",
            w[1] - w[0]
        );
    }
    let last_throttled = *throttled.last().expect("the first tick's throttled line");
    assert!(
        last_ns - last_throttled < 250 * MS,
        "{context}: the last throttled line is {} ns before the last tick, so the throttle \
         stopped emitting once its period ran out: {lines:?}",
        last_ns - last_throttled
    );
}

#[test]
#[serial]
fn a_virtual_run_prints_the_fixtures_throttled_and_once_lines_identically_twice() {
    let mut runs = Vec::new();
    for tag in ["a", "b"] {
        let dir = tempfile::tempdir().unwrap();
        build_workspace(dir.path(), &unique_prefix(tag));
        let out = cerulion(
            dir.path(),
            &[
                "graph",
                "run",
                "logthr",
                "--time-source",
                "virtual",
                "--single-process",
            ],
        );
        assert_eq!(out.code, Some(0), "run {tag}\nstderr:\n{}", out.stderr);
        let lines = node_lines(&out.stderr);
        assert_matches_fixture_oracle(&lines, &format!("virtual run {tag}"));
        runs.push(lines);
    }
    assert_eq!(runs[0], runs[1], "two runs print the same line sequence");
}

fn find_bag(recordings: &Path) -> PathBuf {
    std::fs::read_dir(recordings)
        .expect("the recordings directory exists")
        .flatten()
        .map(|e| e.path())
        .find(|p| p.extension().and_then(|x| x.to_str()) == Some("mcap"))
        .expect("the run wrote a bag")
}

/// On drop, SIGKILLs any recorder this run left behind. The recorder runs in
/// its own process group, so killing the run on a panic would orphan it. The
/// run records into an absolute path inside its own temporary directory, and
/// the recorder's `--out` carries that path, so the match finds this run's
/// recorder and never one another run started.
struct BagdGuard {
    pattern: String,
}

impl BagdGuard {
    fn for_recordings(recordings: &Path) -> Self {
        Self {
            pattern: format!("bagd --out {}/", recordings.display()),
        }
    }
}

impl Drop for BagdGuard {
    fn drop(&mut self) {
        let Ok(out) = Command::new("pgrep")
            .args(["-f", "--", &self.pattern])
            .output()
        else {
            return;
        };
        for pid in String::from_utf8_lossy(&out.stdout)
            .lines()
            .filter_map(|l| l.trim().parse::<i32>().ok())
        {
            // SAFETY: killpg(2) on the recorder's own process group; no memory
            // is touched. ESRCH after a clean exit is the expected no-op.
            unsafe {
                libc::killpg(pid, libc::SIGKILL);
            }
        }
    }
}

#[test]
#[serial]
fn a_recorded_run_prints_the_same_lines_and_its_bag_resims_and_verifies() {
    let dir = tempfile::tempdir().unwrap();
    // Canonical, so the path the recorder is given is the one matched here
    // whether or not the CLI resolves it again.
    let root = &dir.path().canonicalize().unwrap();
    build_workspace(root, &unique_prefix("rec"));
    let recordings = root.join("recordings");
    let _bagd = BagdGuard::for_recordings(&recordings);
    let record = format!("--record={}", recordings.display());
    let out = cerulion(
        root,
        &["graph", "run", "logthr", &record, "--single-process"],
    );
    assert_eq!(out.code, Some(0), "recorded run\nstderr:\n{}", out.stderr);
    let recorded = node_lines(&out.stderr);
    assert_follows_the_throttle_rule(&recorded, "recorded run");

    let bag = find_bag(&recordings);
    let bag_arg = bag.to_str().unwrap();
    let out = cerulion(
        root,
        &["bag", "play", "--resim", "all", "--verify", bag_arg],
    );
    let resim_err = out.stderr;
    assert_eq!(
        out.code,
        Some(0),
        "bag play --resim all --verify\nstderr:\n{resim_err}"
    );
    assert!(
        resim_err.contains("replay PASS"),
        "the verdict is printed:\n{resim_err}"
    );
    // The re-execution runs the node again on the recorded graph clock, so it
    // suppresses and emits the very same lines, graph timestamps included.
    assert_eq!(
        node_lines(&resim_err),
        recorded,
        "the resim prints the recorded run's lines\nresim stderr:\n{resim_err}"
    );
}
