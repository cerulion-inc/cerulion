// SPDX-License-Identifier: AGPL-3.0-only
//! Usage telemetry for the daemon: one `vizd_started` event and a
//! `vizd_heartbeat` every [`HEARTBEAT_INTERVAL`] while it runs.
//!
//! Only the production entry (`main.rs`) starts this, so the in-process
//! daemons the tests drive never send anything. Nothing is sent unless the
//! binary was built with a PostHog key and consent resolves enabled (see
//! `cerulion_telemetry`); events carry no topic, host, path or payload data,
//! only the platform and whole minutes of uptime.

use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use cerulion_telemetry::{consent, Client, Common, EventSpec, Props, DEFAULT_SHUTDOWN_BUDGET};

/// How often a running daemon reports that it is still up.
pub const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(15 * 60);

/// Sent once, after the control socket is bound.
pub const VIZD_STARTED: EventSpec = EventSpec {
    name: "vizd_started",
    allowlist: &["os", "arch"],
};

/// Sent every [`HEARTBEAT_INTERVAL`] of uptime.
pub const VIZD_HEARTBEAT: EventSpec = EventSpec {
    name: "vizd_heartbeat",
    allowlist: &["uptime_minutes"],
};

/// The properties every vizd event carries.
pub fn common() -> Common {
    Common {
        surface: "vizd".into(),
        env: if cfg!(debug_assertions) {
            "dev"
        } else {
            "prod"
        }
        .into(),
        app_version: env!("CARGO_PKG_VERSION").into(),
        channel: None,
    }
}

/// `vizd_started` properties.
pub fn started_props() -> Props {
    vec![
        ("os".into(), std::env::consts::OS.into()),
        ("arch".into(), std::env::consts::ARCH.into()),
    ]
}

/// `vizd_heartbeat` properties for a daemon that has been up for `uptime`.
pub fn heartbeat_props(uptime: Duration) -> Props {
    let minutes = uptime.as_secs() / 60;
    vec![(
        "uptime_minutes".into(),
        i64::try_from(minutes).unwrap_or(i64::MAX).into(),
    )]
}

/// A thread that calls `tick(n)` every `interval` (n = 1, 2, ...) until
/// dropped. Dropping it wakes the thread at once and joins it.
pub struct Heartbeat {
    stop: Option<mpsc::Sender<()>>,
    thread: Option<JoinHandle<()>>,
}

impl Heartbeat {
    /// Start the ticking thread.
    pub fn spawn(interval: Duration, mut tick: impl FnMut(u64) + Send + 'static) -> Heartbeat {
        let (stop, stopped) = mpsc::channel::<()>();
        let thread = thread::Builder::new()
            .name("vizd-telemetry-heartbeat".into())
            .spawn(move || {
                let mut ticks = 0u64;
                while let Err(RecvTimeoutError::Timeout) = stopped.recv_timeout(interval) {
                    ticks += 1;
                    tick(ticks);
                }
            })
            .ok();
        Heartbeat {
            stop: Some(stop),
            thread,
        }
    }
}

impl Drop for Heartbeat {
    fn drop(&mut self) {
        drop(self.stop.take());
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

/// The daemon's telemetry: the client plus its heartbeat. `None` when
/// telemetry is off, unkeyed, or has no anonymous id, in which case no
/// thread is started.
pub struct Telemetry {
    heartbeat: Option<Heartbeat>,
    client: Arc<Mutex<Option<Client>>>,
}

impl Telemetry {
    /// Send `vizd_started` and start the heartbeat.
    pub fn start() -> Option<Telemetry> {
        let client = Client::from_env(common())?;
        let anon_id = consent::anon_id().ok().flatten()?;
        client.capture_anonymous(VIZD_STARTED, &anon_id, started_props());
        let client = Arc::new(Mutex::new(Some(client)));
        let beat = Arc::clone(&client);
        let started = Instant::now();
        let heartbeat = Heartbeat::spawn(HEARTBEAT_INTERVAL, move |_| {
            // Consent is re-read on every beat, so `cerulion telemetry off` or
            // `DO_NOT_TRACK` stops a running daemon's heartbeats.
            if !consent::status().enabled {
                return;
            }
            if let Ok(guard) = beat.lock() {
                if let Some(client) = guard.as_ref() {
                    client.capture_anonymous(
                        VIZD_HEARTBEAT,
                        &anon_id,
                        heartbeat_props(started.elapsed()),
                    );
                }
            }
        });
        Some(Telemetry {
            heartbeat: Some(heartbeat),
            client,
        })
    }

    /// Stop the heartbeat and flush within [`DEFAULT_SHUTDOWN_BUDGET`].
    pub fn shutdown(mut self) {
        drop(self.heartbeat.take());
        let client = self.client.lock().ok().and_then(|mut guard| guard.take());
        if let Some(mut client) = client {
            client.shutdown(DEFAULT_SHUTDOWN_BUDGET);
        }
    }
}
