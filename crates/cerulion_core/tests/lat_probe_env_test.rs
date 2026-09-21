// SPDX-License-Identifier: AGPL-3.0-only
//! The latency probe's ENV → BEHAVIOUR wiring, pinned over a SUBPROCESS.
//!
//! [`cerulion_core::lat_probe`]'s parse is oracle-tested in-module, but the parse is
//! not the risk. The risk is the wiring: the stride is resolved ONCE into a
//! process-global `OnceLock`, so an in-process test can observe exactly one value
//! for the whole binary and cannot tell "the env was read" from "the default
//! happened to match". A probe that silently ignored its switch would leave an
//! operator running the measurement runbook against an empty log and
//! concluding, wrongly, that the desk chain is fast.
//!
//! So each arm runs a CHILD process (`current_exe` + `--exact
//! subprocess_probe_report --ignored --nocapture`, the `iox2_log_level_test`
//! pattern) with the env var set, and asserts on what the child reports back against
//! a HAND-WRITTEN oracle — including WHICH sequences a stride selects, since the
//! cross-process join depends entirely on two daemons independently choosing the
//! same set.
//!
//! No transport, no iceoryx2 — the children are pure env + arithmetic, so the file
//! is parallel-safe and fast.

use std::io::Read;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_core::lat_probe::LAT_PROBE_ENV;

/// The guard env var that turns a re-exec into the child entry point (rather than a
/// nested parent that would fork forever).
const CHILD_ENV: &str = "CER_LAT_PROBE_CHILD";

/// Sequences the child evaluates, so the parent can pin the SELECTED SET exactly.
const PROBE_SEQS: u32 = 91;

/// The child's one-line report marker.
const REPORT: &str = "PROBE-REPORT";

/// Run the child with `value` for [`LAT_PROBE_ENV`] (`None` = leave it UNSET) and
/// return its `(stdout, stderr)`. Bounded: a child that hangs fails loudly instead
/// of wedging the suite.
fn run_child(value: Option<&str>) -> (String, String) {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args([
        "--exact",
        "subprocess_probe_report",
        "--ignored",
        "--nocapture",
    ])
    .env(CHILD_ENV, "1")
    .stdout(Stdio::piped())
    .stderr(Stdio::piped());
    match value {
        Some(v) => cmd.env(LAT_PROBE_ENV, v),
        // The parent's own environment must not leak into an "unset" arm.
        None => cmd.env_remove(LAT_PROBE_ENV),
    };

    let mut child = cmd.spawn().expect("spawn probe child");
    let mut out = child.stdout.take().expect("stdout");
    let mut err = child.stderr.take().expect("stderr");
    // Drain both pipes on threads so a full pipe buffer can never deadlock the wait.
    let out_h = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = out.read_to_string(&mut s);
        s
    });
    let err_h = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = err.read_to_string(&mut s);
        s
    });

    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        match child.try_wait().expect("try_wait") {
            Some(_) => break,
            None if Instant::now() >= deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("the probe child did not exit within 30s");
            }
            None => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    (
        out_h.join().expect("stdout thread"),
        err_h.join().expect("stderr thread"),
    )
}

/// Extract the child's report line body (everything after the marker).
fn report(stdout: &str) -> String {
    stdout
        .lines()
        .find_map(|l| {
            l.split_once(REPORT)
                .map(|(_, rest)| rest.trim().to_string())
        })
        .unwrap_or_else(|| panic!("no {REPORT} line in child stdout:\n{stdout}"))
}

#[test]
fn an_unset_probe_is_completely_inert() {
    // The shipping default. Anything else here would mean every desk daemon logs a
    // line per video frame by default — the disk-fill class.
    let (stdout, _) = run_child(None);
    assert_eq!(
        report(&stdout),
        "enabled=false selected=",
        "an unset probe must report disabled and select NOTHING"
    );
}

#[test]
fn an_explicit_off_or_zero_is_inert_too() {
    for value in ["off", "OFF", "0"] {
        let (stdout, _) = run_child(Some(value));
        assert_eq!(
            report(&stdout),
            "enabled=false selected=",
            "{value:?} must disable the probe"
        );
    }
}

#[test]
fn stride_one_selects_every_frame() {
    // The value the runbook tells an operator to use for a short capture.
    let (stdout, _) = run_child(Some("1"));
    let expected: Vec<String> = (0..PROBE_SEQS).map(|s| s.to_string()).collect();
    assert_eq!(
        report(&stdout),
        format!("enabled=true selected={}", expected.join(",")),
        "stride 1 must select every sequence"
    );
}

#[test]
fn a_stride_selects_exactly_the_multiples_and_nothing_else() {
    // THE wiring pin. A probe that read the env but ignored the VALUE would pass the
    // stride-1 arm above and fail here — and a stride that disagreed between two
    // daemons would join zero frames, which is the whole point of the mechanism.
    let (stdout, _) = run_child(Some("30"));
    assert_eq!(
        report(&stdout),
        "enabled=true selected=0,30,60,90",
        "stride 30 over sequences 0..{PROBE_SEQS} must select exactly the multiples"
    );
}

#[test]
fn an_unusable_value_disables_the_probe_loudly_never_silently() {
    // An operator who typo'd the switch must be TOLD, not left staring at an empty
    // log. The child installs a stderr subscriber so the warn is observable.
    let (stdout, stderr) = run_child(Some("yes"));
    assert_eq!(
        report(&stdout),
        "enabled=false selected=",
        "an unusable value must not half-enable the probe"
    );
    assert!(
        stderr.contains("latency probe DISABLED"),
        "the refusal must be LOUD; stderr was:\n{stderr}"
    );
    assert!(
        stderr.contains(LAT_PROBE_ENV),
        "the refusal must name the env var so the operator can find it; stderr:\n{stderr}"
    );
}

#[test]
fn an_enabled_probe_announces_itself_with_its_stride() {
    // The complement of the arm above: an operator who set it correctly gets a
    // confirmation carrying the stride actually in force, so a mismatch between what
    // they typed and what the daemon does is visible in the same log they are about
    // to read.
    let (_, stderr) = run_child(Some("30"));
    assert!(
        stderr.contains("latency probe ENABLED"),
        "an enabled probe must announce itself; stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("stride=30"),
        "the announcement must carry the stride in force; stderr:\n{stderr}"
    );
}

/// The CHILD entry point: resolve the probe from the env exactly as a daemon does,
/// then report what it decided. `#[ignore]` so it never runs in a normal sweep, and
/// it no-ops unless [`CHILD_ENV`] marks this process as a child.
#[test]
#[ignore = "subprocess child entry point — driven by the parent tests in this file"]
fn subprocess_probe_report() {
    if std::env::var(CHILD_ENV).is_err() {
        return;
    }
    // A stderr subscriber so the module's ENABLED / DISABLED lines are observable.
    let _ = tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        // ANSI OFF: the parent matches whole `key=value` tokens, and colour codes
        // would split `stride=30` into pieces no substring check can see.
        .with_ansi(false)
        .with_max_level(tracing::Level::DEBUG)
        .try_init();

    let enabled = cerulion_core::lat_probe::probe_enabled();
    let selected: Vec<String> = (0..PROBE_SEQS)
        .filter(|s| cerulion_core::lat_probe::should_sample(*s))
        .map(|s| s.to_string())
        .collect();
    println!("{REPORT} enabled={enabled} selected={}", selected.join(","));
}
