// SPDX-License-Identifier: AGPL-3.0-only
//! `IOX2_LOG_LEVEL` must actually filter iceoryx2's own logger.
//!
//! # The bug
//!
//! iceoryx2's log level is a `static AtomicU8` inside the `iceoryx2-log` crate,
//! consulted by `__internal_print_log_msg` before every emission. Nothing reads
//! `IOX2_LOG_LEVEL` unless somebody calls `set_log_level*`. The
//! documented knob was INERT — `IOX2_LOG_LEVEL=error` still produced 190 MB of
//! iceoryx2 warnings in 30 s, and `IOX2_LOG_LEVEL=FATAL` 125 MB in 25 s.
//!
//! The fix routes every entry point through
//! `cerulion_core::iceoryx_logger::init_iceoryx_log_level*`, so the knob is
//! real.
//!
//! # Why a subprocess
//!
//! The emission goes to the PROCESS's stderr via iceoryx2's built-in console
//! logger (`iceoryx2-bb-loggers`), which libtest cannot capture and which no
//! `tracing` subscriber sees. Asserting on it requires owning a child's stderr
//! — the same constraint (and the same self-re-exec shape) as
//! `cdylib_tracing_stopgap_test.rs`.
//!
//! # Why the probe is the REAL production line
//!
//! The child does not synthesize a log call. It reproduces the exact flood
//! condition — a publisher notifying its own never-drained listener until the
//! `AF_UNIX SOCK_DGRAM` event socket fills — so the string under test is the
//! genuine `iceoryx2-0.9.1/src/port/notifier.rs:525` warning
//! (`... due to FailedToDeliverSignal.`) that flooded the robot, not a stand-in.
//!
//! # Arms
//!
//! | `IOX2_LOG_LEVEL` | expectation |
//! |---|---|
//! | `warn` | warning PRESENT — the ANTI-TAUTOLOGY control: the child really does generate the condition, so every "absent" arm below is meaningful |
//! | (unset) | ABSENT — Cerulion's default is `error` (NOT iceoryx2's crate default `Info`, which would print) |
//! | `error` | ABSENT — the live-evidence value that did nothing |
//! | `FATAL` | ABSENT — the other live-evidence value; also pins case-insensitivity |
//! | `notalevel` | ABSENT + a LOUD one-line fallback message (a typo must never silently re-enable the flood) |

use std::io::Read;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;

/// Set by the parent so the ignored child test body actually runs its probe.
const CHILD_ENV: &str = "CER_IOX2_LOG_LEVEL_CHILD";

/// The genuine iceoryx2 warning the Go2 flood consisted of
/// (`iceoryx2-0.9.1/src/port/notifier.rs:525`).
const IOX2_WARN_MARKER: &str = "FailedToDeliverSignal";

/// Cerulion's loud fallback for an unparseable `IOX2_LOG_LEVEL`
/// (`iceoryx_logger::init_iceoryx_log_level`).
const FALLBACK_MARKER: &str = "is not a valid iceoryx2 log level";

/// Breadcrumb the child prints once its probe has run, so a child that died
/// before probing cannot be mistaken for "the level suppressed everything".
const CHILD_DONE_MARKER: &str = "CHILD_PROBE_DONE";

/// Notifies the child issues without draining. Must exceed the platform's
/// `AF_UNIX SOCK_DGRAM` capacity for 8-byte datagrams by a wide margin (macOS
/// saturates after ~32, Linux after a few hundred).
const CHILD_NOTIFIES: usize = 2_000;

/// Bounded wait for the child — a hang must fail loudly, never wedge CI.
const CHILD_TIMEOUT: Duration = Duration::from_secs(60);

/// The child body: reproduce the condition and let iceoryx2 log (or
/// not) according to `IOX2_LOG_LEVEL`.
///
/// `#[ignore]` so it never runs in a normal suite pass; the parent invokes it by
/// exact name with [`CHILD_ENV`] set.
#[test]
#[ignore = "child process entry point — driven by the parent tests in this file"]
fn subprocess_child_iox2_probe() {
    if std::env::var_os(CHILD_ENV).is_none() {
        // Someone ran `-- --ignored` directly: do nothing rather than emit
        // thousands of lines.
        return;
    }
    // `init_for_test` applies the level through the SAME entry point a
    // production binary uses; the explicit call below makes the dependency
    // legible and covers the (real) case of an iceoryx2 emission before any
    // transport exists.
    cerulion_core::iceoryx_logger::init_iceoryx_log_level_from_env();

    let ix = cerulion_core::testing::iceoryx_test_config();
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: "log_level_child".to_string(),
            ..Default::default()
        },
        ix,
    )
    .expect("init_for_test");
    let publisher = transport
        .create_publisher("/loglevel/probe", MaxSliceLen::const_new(256), 0)
        .expect("publisher");

    // Never drained ⇒ the publisher's own event socket fills ⇒ iceoryx2 emits
    // its `FailedToDeliverSignal` warning once per notify from then on.
    for _ in 0..CHILD_NOTIFIES {
        let _ = publisher.notify_sent_sample();
    }
    // Prove — independent of any log level — that the condition really occurred
    // in this child, so an "absent marker" assertion cannot pass vacuously.
    assert!(
        publisher.notify_undelivered_count() > 0,
        "child precondition: the notify path must have saturated its own listener"
    );
    eprintln!("{CHILD_DONE_MARKER}");
}

/// Run the child with an explicit `IOX2_LOG_LEVEL` (or unset) and return its
/// captured stderr.
fn run_child(level: Option<&str>) -> String {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args([
        "--exact",
        "subprocess_child_iox2_probe",
        "--ignored",
        "--nocapture",
        "--test-threads=1",
    ])
    .env(CHILD_ENV, "1")
    .stdout(Stdio::piped())
    .stderr(Stdio::piped());
    match level {
        Some(v) => {
            cmd.env("IOX2_LOG_LEVEL", v);
        }
        None => {
            // The repo's `.cargo/config.toml` exports IOX2_LOG_LEVEL=error for
            // `cargo test`, so "unset" must be constructed explicitly.
            cmd.env_remove("IOX2_LOG_LEVEL");
        }
    }
    let mut child = cmd.spawn().expect("spawn child");
    let mut err = child.stderr.take().expect("child stderr");
    // Drain on a helper thread so a large stderr can never deadlock the pipe.
    let drain = std::thread::spawn(move || {
        let mut buf = String::new();
        let _ = err.read_to_string(&mut buf);
        buf
    });

    let deadline = Instant::now() + CHILD_TIMEOUT;
    loop {
        match child.try_wait().expect("try_wait") {
            Some(_) => break,
            None if Instant::now() > deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!("log-level child did not exit within {CHILD_TIMEOUT:?}");
            }
            None => std::thread::sleep(Duration::from_millis(50)),
        }
    }
    let stderr = drain.join().expect("stderr drain thread");
    assert!(
        stderr.contains(CHILD_DONE_MARKER),
        "child did not reach its probe (level={level:?}); stderr:\n{stderr}"
    );
    stderr
}

/// ANTI-TAUTOLOGY CONTROL — run this first mentally: at `warn` the child's
/// stderr DOES carry the real iceoryx2 flood line. Every "absent" assertion in
/// this file is meaningful only because this one passes.
#[test]
fn warn_level_shows_the_real_iceoryx2_flood_line() {
    let stderr = run_child(Some("warn"));
    assert!(
        stderr.contains(IOX2_WARN_MARKER),
        "at IOX2_LOG_LEVEL=warn the genuine iceoryx2 notifier warning must be visible — \
         without it every suppression arm in this file is vacuous; stderr:\n{stderr}"
    );
}

/// The DEFAULT: no `IOX2_LOG_LEVEL` ⇒ Cerulion's `error`, not iceoryx2's crate
/// default `Info` (which would print the flood).
#[test]
fn absent_level_defaults_to_error_and_suppresses_the_flood() {
    let stderr = run_child(None);
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "with IOX2_LOG_LEVEL unset the default must be `error`, suppressing iceoryx2's \
         per-notify warning; stderr:\n{stderr}"
    );
}

/// The exact value the Go2 operator set that did nothing. It must work now.
#[test]
fn error_level_suppresses_the_flood() {
    let stderr = run_child(Some("error"));
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "IOX2_LOG_LEVEL=error must suppress iceoryx2's warn-level flood (this is the \
         live-evidence value that was inert); stderr:\n{stderr}"
    );
}

/// The other live-evidence value — UPPERCASE, so this also pins that the
/// parser is case-insensitive rather than silently rejecting and defaulting.
#[test]
fn uppercase_fatal_level_suppresses_the_flood_without_a_parse_complaint() {
    let stderr = run_child(Some("FATAL"));
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "IOX2_LOG_LEVEL=FATAL must suppress the flood; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains(FALLBACK_MARKER),
        "`FATAL` is a VALID level — it must be accepted case-insensitively, not \
         reported as unparseable; stderr:\n{stderr}"
    );
}

/// A typo must fall back LOUDLY to `error` — never silently, and never to a
/// level that re-enables the flood.
#[test]
fn unparseable_level_falls_back_loudly_to_error() {
    let stderr = run_child(Some("notalevel"));
    assert!(
        stderr.contains(FALLBACK_MARKER),
        "an unparseable IOX2_LOG_LEVEL must be reported LOUDLY (a silently-ignored typo \
         is exactly the inert-knob class of bug); stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("notalevel"),
        "the loud fallback must name the offending value; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "the fallback level must be `error` (suppressing the flood), not iceoryx2's \
         crate default; stderr:\n{stderr}"
    );
}
