// SPDX-License-Identifier: AGPL-3.0-only
//! The logcat subscriber, installed over a fake sink as the process's global
//! subscriber: the runtime's own lines reach it, and the start config's
//! `log_level` filters them.
//!
//! The subscriber and its level are process-wide, so every test holds
//! `SERIAL`, and this file is its own test binary.

use std::ffi::CStr;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use cerulion_android::{codes, install_log_sink_for_test, EmbeddedRuntime, LogPriority, LogSink};
use serde_json::json;
use tempfile::tempdir;

/// The robot name of every runtime here: the first start of the process
/// fixes it, with the listen endpoints.
const ROBOT: &str = "logcat-sink";

/// The runtime's line for a gateway with no LAN-dialable port to advertise.
const BEACON_SKIPPED: &str = "cerulion_android::runtime: mDNS beacon not raised: no LAN-dialable nonzero TCP listen port; port 0 and loopback hosts are not advertised";

#[derive(Clone, Default)]
struct FakeSink(Arc<Mutex<Vec<(LogPriority, String)>>>);

impl FakeSink {
    fn take(&self) -> Vec<(LogPriority, String)> {
        std::mem::take(&mut *self.0.lock().unwrap())
    }
}

impl LogSink for FakeSink {
    fn write(&self, priority: LogPriority, message: &CStr) {
        let message = message.to_str().expect("utf-8 line").to_owned();
        self.0.lock().unwrap().push((priority, message));
    }
}

static SERIAL: Mutex<()> = Mutex::new(());

/// The installed fake sink, emptied, with the serial lock held.
fn sink() -> (MutexGuard<'static, ()>, &'static FakeSink) {
    static SINK: OnceLock<FakeSink> = OnceLock::new();
    let guard = SERIAL.lock().unwrap_or_else(|error| error.into_inner());
    let sink = SINK.get_or_init(|| {
        let sink = FakeSink::default();
        assert!(
            install_log_sink_for_test(sink.clone()),
            "no other global subscriber is set in this binary"
        );
        sink
    });
    sink.take();
    (guard, sink)
}

/// Start a runtime with a loopback gateway plus `extra` config keys, then
/// stop it.
fn run_once(extra: serde_json::Value) -> Result<(), cerulion_android::EmbedError> {
    let temp = tempdir().unwrap();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into())?;
    let mut config = json!({ "listen": ["tcp/127.0.0.1:0"] });
    config
        .as_object_mut()
        .unwrap()
        .extend(extra.as_object().unwrap().clone());
    let started = runtime.start(&config.to_string());
    if started.is_ok() {
        runtime.stop()?;
    }
    started
}

fn texts(lines: &[(LogPriority, String)]) -> Vec<&str> {
    lines.iter().map(|(_, line)| line.as_str()).collect()
}

#[test]
fn the_runtime_line_reaches_the_sink_at_info_priority_by_default() {
    let (_serial, sink) = sink();
    run_once(json!({})).expect("start and stop");
    let lines = sink.take();
    assert!(
        lines.contains(&(LogPriority::Info, BEACON_SKIPPED.to_owned())),
        "{:?}",
        texts(&lines)
    );
}

#[test]
fn log_level_warn_drops_info_lines_until_a_start_without_the_key() {
    let (_serial, sink) = sink();
    run_once(json!({ "log_level": "warn" })).expect("start at warn");
    tracing::info!(target: "app", "info under warn");
    tracing::warn!(target: "app", "warn under warn");
    let lines = sink.take();
    assert!(
        !texts(&lines).contains(&BEACON_SKIPPED),
        "{:?}",
        texts(&lines)
    );
    assert!(!texts(&lines).contains(&"app: info under warn"));
    assert!(lines.contains(&(LogPriority::Warn, "app: warn under warn".to_owned())));

    run_once(json!({})).expect("start without log_level");
    tracing::info!(target: "app", "info after reset");
    let lines = sink.take();
    assert!(
        texts(&lines).contains(&BEACON_SKIPPED),
        "{:?}",
        texts(&lines)
    );
    assert!(texts(&lines).contains(&"app: info after reset"));
}

#[test]
fn a_config_that_fails_to_parse_or_validate_leaves_the_level_unchanged() {
    let (_serial, sink) = sink();
    run_once(json!({ "log_level": "warn" })).expect("start at warn");
    let refused = run_once(json!({ "log_level": "debug", "record": { "file": "../x.mcap" } }))
        .expect_err("a record file outside the files directory is refused");
    assert_eq!(refused.code(), codes::INVALID_ARGUMENT);
    let refused = run_once(json!({ "log_level": "verbose" })).expect_err("unknown level");
    assert_eq!(refused.code(), codes::INVALID_ARGUMENT);
    sink.take();
    tracing::info!(target: "app", "info after refusals");
    tracing::warn!(target: "app", "warn after refusals");
    assert_eq!(texts(&sink.take()), ["app: warn after refusals"]);
    run_once(json!({})).expect("reset to info for the other tests");
}

#[test]
fn a_start_refused_while_running_leaves_the_level_unchanged() {
    let (_serial, sink) = sink();
    let temp = tempdir().unwrap();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), ROBOT.into()).unwrap();
    // The same listen endpoints as `run_once` (see `ROBOT`).
    runtime
        .start(&json!({ "listen": ["tcp/127.0.0.1:0"], "log_level": "warn" }).to_string())
        .expect("start at warn");
    let refused = runtime
        .start(&json!({ "listen": ["tcp/127.0.0.1:0"], "log_level": "error" }).to_string())
        .expect_err("a second start while running is refused");
    assert_eq!(refused.code(), codes::BAD_STATE);
    sink.take();
    tracing::info!(target: "app", "info while running");
    tracing::warn!(target: "app", "warn while running");
    assert_eq!(texts(&sink.take()), ["app: warn while running"]);
    runtime.stop().expect("stop");
    run_once(json!({})).expect("reset to info for the other tests");
}

#[test]
fn a_start_refused_after_its_path_checks_puts_the_previous_level_back() {
    let (_serial, sink) = sink();
    run_once(json!({ "log_level": "warn" })).expect("start at warn");
    let refused = run_once(json!({ "log_level": "error", "graph_yaml": "nodes: [" }))
        .expect_err("a graph that does not parse is refused");
    assert_eq!(refused.code(), codes::GRAPH);
    sink.take();
    tracing::info!(target: "app", "info after a graph refusal");
    tracing::warn!(target: "app", "warn after a graph refusal");
    assert_eq!(texts(&sink.take()), ["app: warn after a graph refusal"]);
    run_once(json!({})).expect("reset to info for the other tests");
}

#[test]
fn a_refused_second_runtime_leaves_the_running_owner_level() {
    let (_serial, sink) = sink();
    let config =
        |level: &str| json!({ "listen": ["tcp/127.0.0.1:0"], "log_level": level }).to_string();
    let owner_dir = tempdir().unwrap();
    let mut owner = EmbeddedRuntime::new(owner_dir.path().to_path_buf(), ROBOT.into()).unwrap();
    owner
        .start(&config("info"))
        .expect("the owner starts at info");
    let second_dir = tempdir().unwrap();
    let mut second = EmbeddedRuntime::new(second_dir.path().to_path_buf(), ROBOT.into()).unwrap();
    let refused = second
        .start(&config("error"))
        .expect_err("a second gateway runtime cannot share the owner's gateway");
    assert_eq!(refused.code(), codes::BAD_STATE);
    sink.take();
    tracing::info!(target: "app", "owner info after the refusal");
    assert_eq!(texts(&sink.take()), ["app: owner info after the refusal"]);
    owner.stop().expect("stop the owner");
}

/// A value whose `Debug` logs a line of its own.
struct Chatty;

impl std::fmt::Debug for Chatty {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        tracing::info!(target: "inner", "from debug");
        f.write_str("chatty")
    }
}

#[test]
fn a_line_logged_while_a_line_is_formatted_still_reaches_the_sink() {
    let (_serial, sink) = sink();
    tracing::info!(target: "outer", value = ?Chatty, "x");
    // The global subscriber has no reentrancy guard, so the inner line is
    // written while the outer one is still being formatted.
    assert_eq!(
        texts(&sink.take()),
        ["inner: from debug", "outer: x value=chatty"]
    );
}

/// Logs a line when its thread's locals are destroyed.
struct LogsOnDrop;

impl Drop for LogsOnDrop {
    fn drop(&mut self) {
        tracing::warn!(target: "teardown", "from a thread-local destructor");
    }
}

thread_local! {
    static LOGS_ON_DROP: LogsOnDrop = const { LogsOnDrop };
}

#[test]
fn a_line_logged_while_a_thread_exits_still_reaches_the_sink() {
    let (_serial, sink) = sink();
    std::thread::spawn(|| {
        // The line buffer is made after this local, so it is destroyed first.
        LOGS_ON_DROP.with(|_| ());
        tracing::info!(target: "worker", "running");
    })
    .join()
    .expect("a line logged during thread exit must not panic");
    assert_eq!(
        texts(&sink.take()),
        [
            "worker: running",
            "teardown: from a thread-local destructor"
        ]
    );
}
