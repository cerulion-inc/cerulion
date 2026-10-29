// SPDX-License-Identifier: AGPL-3.0-only
//! An app that installed its own global `tracing` subscriber keeps it: the
//! logcat subscriber is not installed over it, and `log_level` does not
//! filter it.
//!
//! The app's subscriber is process-wide, so this file is its own test binary.

use std::ffi::CStr;
use std::sync::{Arc, Mutex};

use cerulion_android::{install_log_sink_for_test, EmbeddedRuntime, LogPriority, LogSink};
use serde_json::json;
use tempfile::tempdir;
use tracing_subscriber::layer::{Context, SubscriberExt};
use tracing_subscriber::Layer;

const BEACON_SKIPPED: &str = "mDNS beacon not raised: no LAN-dialable nonzero TCP listen port; port 0 and loopback hosts are not advertised";

type Lines = Arc<Mutex<Vec<String>>>;

/// The app's own layer: keeps each event's message.
struct AppLayer(Lines);

impl<S: tracing::Subscriber> Layer<S> for AppLayer {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, S>) {
        struct Message<'a>(&'a mut String);
        impl tracing::field::Visit for Message<'_> {
            fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
                if field.name() == "message" {
                    *self.0 = format!("{value:?}");
                }
            }
        }
        let mut message = String::new();
        event.record(&mut Message(&mut message));
        self.0.lock().unwrap().push(message);
    }
}

#[derive(Clone, Default)]
struct FakeSink(Arc<Mutex<Vec<String>>>);

impl LogSink for FakeSink {
    fn write(&self, _priority: LogPriority, message: &CStr) {
        let message = message.to_str().expect("utf-8 line").to_owned();
        self.0.lock().unwrap().push(message);
    }
}

#[test]
fn an_app_subscriber_set_first_is_kept_and_log_level_does_not_filter_it() {
    let app_lines = Lines::default();
    tracing::subscriber::set_global_default(
        tracing_subscriber::registry().with(AppLayer(app_lines.clone())),
    )
    .expect("the app's subscriber is the first in this binary");

    let sink = FakeSink::default();
    assert!(
        !install_log_sink_for_test(sink.clone()),
        "the logcat subscriber must not replace the app's"
    );

    let temp = tempdir().unwrap();
    let mut runtime = EmbeddedRuntime::new(temp.path().to_path_buf(), "logcat-app".into()).unwrap();
    runtime
        .start(&json!({ "listen": ["tcp/127.0.0.1:0"], "log_level": "error" }).to_string())
        .expect("start");
    runtime.stop().expect("stop");

    let app_lines = app_lines.lock().unwrap().clone();
    assert!(
        app_lines.iter().any(|line| line == BEACON_SKIPPED),
        "the app's subscriber must get the runtime's info line under log_level error: {app_lines:?}"
    );
    assert!(
        sink.0.lock().unwrap().is_empty(),
        "nothing reaches the logcat sink"
    );
}
