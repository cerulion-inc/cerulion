// SPDX-License-Identifier: AGPL-3.0-only
//! `tracing` output to logcat.
//!
//! On Android the runtime installs one global subscriber whose layer writes
//! every enabled event to logcat under one tag, unless the app installed its
//! own subscriber first. The start config's `log_level` sets its filter.

use std::sync::OnceLock;

use tracing_subscriber::filter::LevelFilter;
use tracing_subscriber::{reload, Registry};
#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
use {
    std::ffi::CStr,
    tracing::{Level, Subscriber},
    tracing_subscriber::layer::{Context, SubscriberExt},
    tracing_subscriber::Layer,
};

use crate::config::LogLevelConfig;

/// The filter handle of the installed subscriber; unset while none is.
static LEVEL: OnceLock<LevelHandle> = OnceLock::new();

type LevelHandle = reload::Handle<LevelFilter, Registry>;

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
/// The Android log priority of a line (`android/log.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum LogPriority {
    /// `ANDROID_LOG_VERBOSE`, for `trace` lines.
    Verbose = 2,
    /// `ANDROID_LOG_DEBUG`.
    Debug = 3,
    /// `ANDROID_LOG_INFO`.
    Info = 4,
    /// `ANDROID_LOG_WARN`.
    Warn = 5,
    /// `ANDROID_LOG_ERROR`.
    Error = 6,
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
impl LogPriority {
    fn of(level: &Level) -> Self {
        match *level {
            Level::ERROR => Self::Error,
            Level::WARN => Self::Warn,
            Level::INFO => Self::Info,
            Level::DEBUG => Self::Debug,
            Level::TRACE => Self::Verbose,
        }
    }
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
/// Where a formatted line goes: logcat on a device, a fake in tests.
pub trait LogSink: Send + Sync + 'static {
    /// Write one line. `message` is `<target>: <message> <field>=<value>...`.
    fn write(&self, priority: LogPriority, message: &CStr);
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
struct LogcatLayer<S> {
    sink: S,
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
impl<S: LogSink, Sub: Subscriber> Layer<Sub> for LogcatLayer<S> {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, Sub>) {
        let priority = LogPriority::of(event.metadata().level());
        let written = LINE.try_with(|line| match line.try_borrow_mut() {
            Ok(mut line) => {
                self.sink.write(priority, format_line(&mut line, event));
                // Keep a line-sized buffer, not the largest line ever seen.
                line.shrink_to(LINE_CAPACITY);
                true
            }
            // A field's `Debug` logged an event of its own while this
            // thread's line was being built.
            Err(_) => false,
        });
        // Also when the thread is exiting and its buffer is already gone.
        if written != Ok(true) {
            self.sink
                .write(priority, format_line(&mut Vec::new(), event));
        }
    }
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
thread_local! {
    /// The buffer a thread formats its lines in, reused so an enabled line
    /// allocates only while the buffer grows.
    static LINE: std::cell::RefCell<Vec<u8>> =
        std::cell::RefCell::new(Vec::with_capacity(LINE_CAPACITY));
}

/// Bytes kept for a thread's line buffer between lines (logcat truncates a
/// line near 4 KiB).
#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
const LINE_CAPACITY: usize = 1024;

/// Write `<target>: <message> <field>=<value>...` into `line`, NUL-terminated.
///
/// A NUL byte inside the text becomes a space, so logcat prints the whole
/// line.
#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
fn format_line<'a>(line: &'a mut Vec<u8>, event: &tracing::Event<'_>) -> &'a CStr {
    use std::io::Write;
    line.clear();
    // Writes to a `Vec` cannot fail.
    let _ = write!(line, "{}:", event.metadata().target());
    event.record(&mut FieldWriter {
        line: &mut *line,
        message: true,
    });
    event.record(&mut FieldWriter {
        line: &mut *line,
        message: false,
    });
    for byte in line.iter_mut() {
        if *byte == 0 {
            *byte = b' ';
        }
    }
    line.push(0);
    CStr::from_bytes_with_nul(line).expect("one NUL, at the end")
}

/// Writes either the `message` field or every other field of an event.
#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
struct FieldWriter<'a> {
    line: &'a mut Vec<u8>,
    message: bool,
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
impl tracing::field::Visit for FieldWriter<'_> {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        use std::io::Write;
        let is_message = field.name() == "message";
        // Writes to a `Vec` cannot fail.
        let _ = match (self.message, is_message) {
            (true, true) => write!(self.line, " {value:?}"),
            (false, false) => write!(self.line, " {}={value:?}", field.name()),
            _ => Ok(()),
        };
    }
}

#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
/// A subscriber that writes to `sink`, with its level filter's handle.
fn subscriber(sink: impl LogSink) -> (impl Subscriber + Send + Sync, LevelHandle) {
    let (filter, handle) = reload::Layer::new(LevelFilter::INFO);
    (
        Registry::default().with(filter).with(LogcatLayer { sink }),
        handle,
    )
}

/// Install the subscriber over `sink` as the global default.
///
/// Returns `false`, and changes nothing, when a global subscriber is already
/// set: an app that installed its own keeps it, and `log_level` then has no
/// effect.
#[cfg(any(target_os = "android", test, feature = "test-helpers"))]
pub fn install_with_sink(sink: impl LogSink) -> bool {
    let (subscriber, handle) = subscriber(sink);
    if tracing::subscriber::set_global_default(subscriber).is_err() {
        return false;
    }
    // Only the one successful global install reaches this line.
    let _ = LEVEL.set(handle);
    true
}

/// Install the logcat subscriber, unless a global subscriber is already set.
///
/// The library load and `init` both call it; the first call installs.
#[cfg(target_os = "android")]
pub(crate) fn install_logcat() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        install_with_sink(logcat::Logcat);
    });
}

#[cfg(target_os = "android")]
mod logcat {
    use std::ffi::{c_char, c_int, CStr};

    use super::{LogPriority, LogSink};

    /// The logcat tag of every runtime and node line.
    const TAG: &CStr = c"Cerulion";

    #[link(name = "log")]
    unsafe extern "C" {
        /// `liblog`'s write call (`android/log.h`), in every NDK API level.
        fn __android_log_write(prio: c_int, tag: *const c_char, text: *const c_char) -> c_int;
    }

    /// Writes each line to logcat.
    pub(super) struct Logcat;

    impl LogSink for Logcat {
        fn write(&self, priority: LogPriority, message: &CStr) {
            // SAFETY: `TAG` and `message` are NUL-terminated and live for
            // the whole call; `liblog` copies both and keeps neither pointer.
            // Its return code is not checked: a failed write has nowhere
            // else to go.
            unsafe {
                __android_log_write(priority as c_int, TAG.as_ptr(), message.as_ptr());
            }
        }
    }
}

/// Set the filter of the installed subscriber; a no-op while none is.
pub(crate) fn set_level(level: LogLevelConfig) {
    if let Some(handle) = LEVEL.get() {
        apply(handle, level);
    }
}

fn apply(handle: &LevelHandle, level: LogLevelConfig) {
    let filter = match level {
        LogLevelConfig::Error => LevelFilter::ERROR,
        LogLevelConfig::Warn => LevelFilter::WARN,
        LogLevelConfig::Info => LevelFilter::INFO,
        LogLevelConfig::Debug => LevelFilter::DEBUG,
        LogLevelConfig::Trace => LevelFilter::TRACE,
    };
    // `reload` fails only once the subscriber is dropped; the global one
    // never is, and a scoped test subscriber outlives every call.
    let _ = handle.reload(filter);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Clone, Default)]
    struct FakeSink(Arc<Mutex<Vec<(LogPriority, String)>>>);

    impl FakeSink {
        fn lines(&self) -> Vec<(LogPriority, String)> {
            self.0.lock().unwrap().clone()
        }
    }

    impl LogSink for FakeSink {
        fn write(&self, priority: LogPriority, message: &CStr) {
            let message = message.to_str().expect("utf-8 line").to_owned();
            self.0.lock().unwrap().push((priority, message));
        }
    }

    /// Run `emit` under a fresh subscriber over a fake sink, filtered at
    /// `level`, and return what reached the sink.
    fn capture(level: LogLevelConfig, emit: impl FnOnce()) -> Vec<(LogPriority, String)> {
        let sink = FakeSink::default();
        let (subscriber, handle) = subscriber(sink.clone());
        tracing::subscriber::with_default(subscriber, || {
            apply(&handle, level);
            emit();
        });
        sink.lines()
    }

    #[test]
    fn each_level_reaches_the_sink_with_its_android_priority() {
        let lines = capture(LogLevelConfig::Trace, || {
            tracing::trace!(target: "app", "t");
            tracing::debug!(target: "app", "d");
            tracing::info!(target: "app", "i");
            tracing::warn!(target: "app", "w");
            tracing::error!(target: "app", "e");
        });
        let priorities: Vec<i32> = lines.iter().map(|(p, _)| *p as i32).collect();
        // ANDROID_LOG_VERBOSE..ANDROID_LOG_ERROR from `android/log.h`.
        assert_eq!(priorities, [2, 3, 4, 5, 6]);
    }

    #[test]
    fn a_line_carries_target_message_and_fields() {
        let error = std::io::Error::other("bind refused");
        let lines = capture(LogLevelConfig::Info, || {
            tracing::warn!(target: "cerulion_android::runtime", port = 7683u16, error = %error, "mDNS beacon not raised");
            tracing::info!(target: "timer", "tick");
            tracing::info!(target: "timer", seq = 4u64);
        });
        let text: Vec<&str> = lines.iter().map(|(_, line)| line.as_str()).collect();
        assert_eq!(
            text,
            [
                "cerulion_android::runtime: mDNS beacon not raised port=7683 error=bind refused",
                "timer: tick",
                "timer: seq=4",
            ]
        );
    }

    #[test]
    fn a_nul_byte_in_a_line_does_not_cut_it_short() {
        let lines = capture(LogLevelConfig::Info, || {
            tracing::info!(target: "app", name = "a\0b", "x\0y");
        });
        assert_eq!(lines[0].1, "app: x y name=\"a\\0b\"");
    }

    #[test]
    fn the_level_drops_every_lower_line_and_a_later_level_restores_them() {
        let sink = FakeSink::default();
        let (subscriber, handle) = subscriber(sink.clone());
        tracing::subscriber::with_default(subscriber, || {
            apply(&handle, LogLevelConfig::Warn);
            tracing::debug!(target: "app", "dropped debug");
            tracing::info!(target: "app", "dropped info");
            tracing::warn!(target: "app", "kept warn");
            tracing::error!(target: "app", "kept error");
            apply(&handle, LogLevelConfig::Error);
            tracing::warn!(target: "app", "dropped warn");
            apply(&handle, LogLevelConfig::Info);
            tracing::debug!(target: "app", "dropped debug again");
            tracing::info!(target: "app", "kept info");
        });
        let text: Vec<String> = sink.lines().into_iter().map(|(_, line)| line).collect();
        assert_eq!(
            text,
            ["app: kept warn", "app: kept error", "app: kept info"]
        );
    }
}
