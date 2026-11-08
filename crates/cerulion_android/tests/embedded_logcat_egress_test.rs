// SPDX-License-Identifier: AGPL-3.0-only
//! The warning for an `egress` list that no gateway reads, written to the
//! logcat subscriber at the level of the start that sets it, and only once
//! that start goes ahead.
//!
//! The subscriber is process-wide and the first start fixes an empty
//! `listen` for the process, so this file is its own test binary.

use std::ffi::CStr;
use std::fs;
use std::sync::{Arc, Mutex};

use cerulion_android::{codes, install_log_sink_for_test, EmbeddedRuntime, LogPriority, LogSink};
use serde_json::json;
use tempfile::tempdir;

const TOPIC: &str = "/logcat_egress/state";

const INERT_EGRESS: &str = "cerulion_android::runtime: egress is set but listen and connect are empty: this start builds no gateway, so the list has no effect";

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

#[test]
fn an_egress_list_without_listen_warns_at_its_own_level_once_the_start_goes_ahead() {
    let sink = FakeSink::default();
    assert!(
        install_log_sink_for_test(sink.clone()),
        "no other global subscriber is set in this binary"
    );
    let temp = tempdir().unwrap();
    let mut runtime =
        EmbeddedRuntime::new(temp.path().to_path_buf(), "logcat-egress".into()).unwrap();
    let publish = json!([{"topic": TOPIC, "schema": "std_msgs/String"}]);

    // A start that keeps `error` drops warnings until a start sets another level.
    runtime
        .start(&json!({"log_level": "error", "publish": publish}).to_string())
        .expect("start at error");
    runtime.stop().expect("stop");
    sink.take();

    // A start refused after it was planned never ran: nothing names its list.
    fs::create_dir_all(temp.path().join("bags")).unwrap();
    fs::write(temp.path().join("bags/taken.mcap"), b"").unwrap();
    let error = runtime
        .start(
            &json!({
                "log_level": "warn",
                "egress": [TOPIC],
                "publish": publish,
                "record": {"file": "taken.mcap"}
            })
            .to_string(),
        )
        .expect_err("an existing recording file is refused");
    assert_eq!(error.code(), codes::RECORDER);
    let lines = sink.take();
    assert!(
        !lines.iter().any(|(_, line)| line == INERT_EGRESS),
        "a refused start must not warn about its list: {lines:?}"
    );

    // The start that goes ahead warns once, at its own `warn` level.
    runtime
        .start(&json!({"log_level": "warn", "egress": [TOPIC], "publish": publish}).to_string())
        .expect("start with an egress list and no listen");
    let lines = sink.take();
    assert_eq!(
        lines
            .iter()
            .filter(|line| **line == (LogPriority::Warn, INERT_EGRESS.to_owned()))
            .count(),
        1,
        "{lines:?}"
    );
    runtime.stop().expect("stop");
}
