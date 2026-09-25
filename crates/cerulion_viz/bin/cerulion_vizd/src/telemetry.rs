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
/// dropped. Dropping it wakes the thread at once and waits at most
/// [`DEFAULT_SHUTDOWN_BUDGET`] for it to finish; a tick stuck past that (a
/// consent read on a hung filesystem) is left to die with the process.
pub struct Heartbeat {
    stop: Option<mpsc::Sender<()>>,
    finished: mpsc::Receiver<()>,
    thread: Option<JoinHandle<()>>,
}

impl Heartbeat {
    /// Start the ticking thread, or `None` if the OS refuses one.
    pub fn spawn(
        interval: Duration,
        mut tick: impl FnMut(u64) + Send + 'static,
    ) -> Option<Heartbeat> {
        let (stop, stopped) = mpsc::channel::<()>();
        let (finish, finished) = mpsc::channel::<()>();
        let thread = thread::Builder::new()
            .name("vizd-telemetry-heartbeat".into())
            .spawn(move || {
                let mut ticks = 0u64;
                while let Err(RecvTimeoutError::Timeout) = stopped.recv_timeout(interval) {
                    ticks += 1;
                    tick(ticks);
                }
                drop(finish);
            })
            .ok()?;
        Some(Heartbeat {
            stop: Some(stop),
            finished,
            thread: Some(thread),
        })
    }
}

impl Heartbeat {
    /// Wake the thread and wait at most `budget` for it to finish. Only the
    /// first call waits; later calls and the drop return at once.
    pub fn stop_within(&mut self, budget: Duration) {
        let Some(stop) = self.stop.take() else {
            return;
        };
        drop(stop);
        let finished = matches!(
            self.finished.recv_timeout(budget),
            Err(RecvTimeoutError::Disconnected)
        );
        if let (true, Some(thread)) = (finished, self.thread.take()) {
            let _ = thread.join();
        }
    }
}

impl Drop for Heartbeat {
    fn drop(&mut self) {
        self.stop_within(DEFAULT_SHUTDOWN_BUDGET);
    }
}

/// The daemon's telemetry: the client plus its heartbeat. `None` when
/// telemetry is off, unkeyed, has no anonymous id, or cannot start its
/// heartbeat thread; nothing is sent then.
pub struct Telemetry {
    heartbeat: Option<Heartbeat>,
    client: Arc<Mutex<Option<Client>>>,
}

impl Telemetry {
    /// Send `vizd_started` and start the heartbeat.
    pub fn start() -> Option<Telemetry> {
        let client = Client::from_env(common())?;
        let anon_id = consent::anon_id().ok().flatten()?;
        let client = Arc::new(Mutex::new(Some(client)));
        let beat = Arc::clone(&client);
        let started = Instant::now();
        let heartbeat = Heartbeat::spawn(HEARTBEAT_INTERVAL, move |_| {
            // Consent and the anonymous id are re-read on every beat, so
            // `cerulion telemetry off` or `DO_NOT_TRACK` stops a running
            // daemon's heartbeats, and an id rotated by an account switch
            // is used from the next beat on.
            if !consent::status().enabled {
                return;
            }
            let Some(anon_id) = consent::anon_id().ok().flatten() else {
                return;
            };
            if let Ok(guard) = beat.lock() {
                if let Some(client) = guard.as_ref() {
                    client.capture_anonymous(
                        VIZD_HEARTBEAT,
                        &anon_id,
                        heartbeat_props(started.elapsed()),
                    );
                }
            }
        })?;
        if let Ok(guard) = client.lock() {
            if let Some(client) = guard.as_ref() {
                client.capture_anonymous(VIZD_STARTED, &anon_id, started_props());
            }
        }
        Some(Telemetry {
            heartbeat: Some(heartbeat),
            client,
        })
    }

    /// Run [`Telemetry::start`] on its own thread, so a consent file held
    /// locked by another process never delays the daemon or its shutdown.
    pub fn start_in_background() -> Starting {
        let (ready, receiver) = mpsc::channel();
        let _ = thread::Builder::new()
            .name("vizd-telemetry-start".into())
            .spawn(move || {
                let _ = ready.send(Telemetry::start());
            });
        Starting(receiver)
    }

    /// Stop the heartbeat and flush, both within one
    /// [`DEFAULT_SHUTDOWN_BUDGET`].
    pub fn shutdown(self) {
        self.shutdown_by(Instant::now() + DEFAULT_SHUTDOWN_BUDGET);
    }

    fn shutdown_by(mut self, deadline: Instant) {
        if let Some(mut heartbeat) = self.heartbeat.take() {
            heartbeat.stop_within(deadline.saturating_duration_since(Instant::now()));
        }
        let client = self
            .client
            .try_lock()
            .ok()
            .and_then(|mut guard| guard.take());
        if let Some(mut client) = client {
            client.shutdown(deadline.saturating_duration_since(Instant::now()));
        }
    }
}

/// Telemetry that may still be starting; see [`Telemetry::start_in_background`].
pub struct Starting(mpsc::Receiver<Option<Telemetry>>);

impl Starting {
    /// Wait for the start and shut the telemetry down, all within one
    /// [`DEFAULT_SHUTDOWN_BUDGET`]. A start still blocked at the deadline is
    /// abandoned and sends nothing more than it already queued.
    pub fn shutdown(self) {
        let deadline = Instant::now() + DEFAULT_SHUTDOWN_BUDGET;
        if let Ok(Some(telemetry)) = self.0.recv_timeout(DEFAULT_SHUTDOWN_BUDGET) {
            telemetry.shutdown_by(deadline);
        }
    }
}
