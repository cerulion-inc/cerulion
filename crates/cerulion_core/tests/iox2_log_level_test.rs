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
//! # Why the probe is a REAL iceoryx2 line
//!
//! The child does not synthesize a log call: the string under test is emitted
//! by iceoryx2 itself, from inside its own service builder, so what is being
//! filtered is genuinely iceoryx2's logger and not a Cerulion stand-in.
//!
//! The line it drives CHANGED with iceoryx2 0.10, and the reason is the point.
//! The original probe reproduced the flood itself: a publisher notifying its
//! own never-drained listener until the `AF_UNIX SOCK_DGRAM` event socket
//! filled, after which every notify failed and iceoryx2 logged
//! `... due to FailedToDeliverSignal.` once per publish. 0.10 made that state
//! unreachable (the doorbell carries one byte, a full doorbell is swallowed,
//! and a notify into an already-notified listener skips the send), so the
//! marker string does not occur and the condition cannot be built. Keeping that
//! probe would have left five arms asserting the ABSENCE of a line nothing
//! emits, which passes for the wrong reason.
//!
//! The probe is now iceoryx2's own `warn!` for an event service built with a
//! zero port ceiling, which it clamps to one and complains about. It is
//! deterministic, needs no timing and no saturation, and it is still a
//! `warn`-level line from inside iceoryx2, which is what every arm below is
//! about.
//!
//! # Arms
//!
//! | `IOX2_LOG_LEVEL` | expectation |
//! |---|---|
//! | `warn` | warning PRESENT — the ANTI-TAUTOLOGY control: the child really does generate the condition, so every "absent" arm below is meaningful |
//! | (unset) | ABSENT — Cerulion's default is `error` (NOT iceoryx2's crate default `Info`, which would print) |
//! | `error` | ABSENT — the live-evidence value that did nothing |
//! | `FATAL` | ABSENT — the other live-evidence value; also pins case-insensitivity |
//! | `notalevel` | ABSENT + a LOUD one-line fallback message (a typo must never silently re-enable iceoryx2's own logging) |

use std::io::Read;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_core::transport::{TransportConfig, TransportManager};

/// Set by the parent so the ignored child test body actually runs its probe.
const CHILD_ENV: &str = "CER_IOX2_LOG_LEVEL_CHILD";

/// A genuine `warn`-level line from inside iceoryx2's own service builder,
/// emitted when an event service is built with a zero port ceiling
/// (`adjust_attributes_to_meaningful_values`, which clamps it to one).
///
/// Deterministic and free of timing: it is a builder complaint, not a runtime
/// failure, so the child can produce it in one call on any platform.
const IOX2_WARN_MARKER: &str = "Setting the maximum amount of notifiers to 0 is not supported";

/// Cerulion's loud fallback for an unparseable `IOX2_LOG_LEVEL`
/// (`iceoryx_logger::init_iceoryx_log_level`).
const FALLBACK_MARKER: &str = "is not a valid iceoryx2 log level";

/// Breadcrumb the child prints once its probe has run, so a child that died
/// before probing cannot be mistaken for "the level suppressed everything".
const CHILD_DONE_MARKER: &str = "CHILD_PROBE_DONE";

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
    // A transport first, so the child exercises the same startup path a real
    // binary does (this is what applies the level) before anything is logged.
    let transport = TransportManager::init_for_test(
        TransportConfig {
            node_name: "log_level_child".to_string(),
            ..Default::default()
        },
        ix.clone(),
    )
    .expect("init_for_test");
    drop(transport);

    // The probe: an event service asked for zero notifiers. iceoryx2 clamps the
    // value to one and warns about it, from inside its own builder.
    let node = iceoryx2::node::NodeBuilder::new()
        .config(&ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("child node");
    let name: iceoryx2::service::service_name::ServiceName =
        "/loglevel/probe/event".try_into().expect("service name");
    let service = node
        .service_builder(&name)
        .event()
        .max_notifiers(0)
        .create()
        .expect("the zero ceiling is clamped, not refused");
    // Prove — independent of any log level — that the clamp really happened in
    // this child, so an "absent marker" assertion cannot pass vacuously.
    use iceoryx2::service::port_factory::PortFactory as _;
    assert_eq!(
        service.static_config().max_notifiers(),
        1,
        "child precondition: iceoryx2 must have clamped the zero notifier ceiling to \
         one, which is the act it warns about"
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

/// ANTI-TAUTOLOGY CONTROL — read this one first: at `warn` the child's stderr
/// DOES carry the real iceoryx2 line. Every "absent" assertion in this file is
/// meaningful only because this one passes.
#[test]
fn warn_level_shows_a_real_iceoryx2_warning() {
    let stderr = run_child(Some("warn"));
    assert!(
        stderr.contains(IOX2_WARN_MARKER),
        "at IOX2_LOG_LEVEL=warn the genuine iceoryx2 builder warning must be visible — \
         without it every suppression arm in this file is vacuous; stderr:\n{stderr}"
    );
}

/// The DEFAULT: no `IOX2_LOG_LEVEL` ⇒ Cerulion's `error`, not iceoryx2's crate
/// default `Info` (which would print every warning).
#[test]
fn absent_level_defaults_to_error_and_suppresses_iceoryx2_warnings() {
    let stderr = run_child(None);
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "with IOX2_LOG_LEVEL unset the default must be `error`, suppressing iceoryx2's \
         warn-level lines; stderr:\n{stderr}"
    );
}

/// The exact value the Go2 operator set that did nothing. It must work now.
#[test]
fn error_level_suppresses_iceoryx2_warnings() {
    let stderr = run_child(Some("error"));
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "IOX2_LOG_LEVEL=error must suppress iceoryx2's warn-level lines (this is the \
         live-evidence value that was inert); stderr:\n{stderr}"
    );
}

/// The other live-evidence value — UPPERCASE, so this also pins that the
/// parser is case-insensitive rather than silently rejecting and defaulting.
#[test]
fn uppercase_fatal_level_suppresses_warnings_without_a_parse_complaint() {
    let stderr = run_child(Some("FATAL"));
    assert!(
        !stderr.contains(IOX2_WARN_MARKER),
        "IOX2_LOG_LEVEL=FATAL must suppress iceoryx2's warn-level lines; stderr:\n{stderr}"
    );
    assert!(
        !stderr.contains(FALLBACK_MARKER),
        "`FATAL` is a VALID level — it must be accepted case-insensitively, not \
         reported as unparseable; stderr:\n{stderr}"
    );
}

/// A typo must fall back LOUDLY to `error` — never silently, and never to a
/// level that re-enables iceoryx2's own logging.
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
        "the fallback level must be `error` (suppressing warn-level lines), not \
         iceoryx2's crate default; stderr:\n{stderr}"
    );
}
