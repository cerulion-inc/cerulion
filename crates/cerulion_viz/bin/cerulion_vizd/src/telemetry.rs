// SPDX-License-Identifier: AGPL-3.0-only
//! Usage telemetry for the daemon: one `vizd_started` event and a
//! `vizd_heartbeat` every [`HEARTBEAT_INTERVAL`] while it runs.
//!
//! Only the production entry (`main.rs`) starts this, so the in-process
//! daemons the tests drive never send anything. Nothing is sent unless a
//! PostHog key is set (`POSTHOG_API_KEY`) and consent resolves enabled (see
//! `cerulion_telemetry`); events carry no topic, host, path or payload data,
//! only the platform and whole minutes of uptime.

use std::sync::atomic::{AtomicBool, Ordering};
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
        Telemetry::begin(client, &anon_id, HEARTBEAT_INTERVAL, || true)
    }

    /// [`Telemetry::start`] with the client and heartbeat interval supplied;
    /// the anonymous id and consent still come from the environment (tests).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn start_with(client: Client, interval: Duration) -> Option<Telemetry> {
        let anon_id = consent::anon_id().ok().flatten()?;
        Telemetry::begin(client, &anon_id, interval, || true)
    }

    /// Queue `vizd_started` for `anon_id` and start a heartbeat every
    /// `interval`. The started event and every beat are queued under the
    /// consent lock, so an opt-out that has returned is always seen. The
    /// first beat waits for the started event to be queued, so it is never
    /// first on the wire, however long the consent lock holds this thread.
    /// `wanted` is asked once more under that lock, right before the started
    /// event is queued: a start whose caller gave up while this thread waited
    /// for the lock (see [`Starting::shutdown`]) queues nothing.
    fn begin(
        client: Client,
        anon_id: &str,
        interval: Duration,
        wanted: impl Fn() -> bool,
    ) -> Option<Telemetry> {
        let client = Arc::new(Mutex::new(Some(client)));
        let beat = Arc::clone(&client);
        let started = Instant::now();
        let (started_queued, after_started) = mpsc::channel::<()>();
        let mut after_started = Some(after_started);
        let heartbeat = Heartbeat::spawn(interval, move |_| {
            // The first beat waits here until `vizd_started` is queued (or
            // this function has returned without queuing it).
            if let Some(gate) = after_started.take() {
                let _ = gate.recv();
            }
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
            // Queued under the consent lock: an opt-out that has returned is
            // always seen, including one that completed while `anon_id`
            // waited on that lock.
            consent::while_enabled(|| {
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
        })?;
        consent::while_enabled(|| {
            if !wanted() {
                return;
            }
            if let Ok(guard) = client.lock() {
                if let Some(client) = guard.as_ref() {
                    client.capture_anonymous(VIZD_STARTED, anon_id, started_props());
                }
            }
        });
        // Queued (or skipped by an opt-out): the beats may follow.
        drop(started_queued);
        Some(Telemetry {
            heartbeat: Some(heartbeat),
            client,
        })
    }

    /// Run [`Telemetry::start`] on its own thread, so a consent file held
    /// locked by another process never delays the daemon or its shutdown.
    pub fn start_in_background() -> Starting {
        Starting::spawn(|| Client::from_env(common()), HEARTBEAT_INTERVAL)
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
pub struct Starting {
    ready: mpsc::Receiver<Option<Telemetry>>,
    abandoned: Arc<AtomicBool>,
}

impl Starting {
    /// [`Telemetry::start_in_background`] with the client and heartbeat
    /// interval supplied; `client` runs on the starter thread (tests).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn spawn_with(
        client: impl FnOnce() -> Option<Client> + Send + 'static,
        interval: Duration,
    ) -> Starting {
        Starting::spawn(client, interval)
    }

    fn spawn(
        client: impl FnOnce() -> Option<Client> + Send + 'static,
        interval: Duration,
    ) -> Starting {
        let (ready, receiver) = mpsc::channel();
        let abandoned = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&abandoned);
        let _ = thread::Builder::new()
            .name("vizd-telemetry-start".into())
            .spawn(move || {
                // The client and the anonymous id come first: a consent file
                // held locked by another process blocks here, before anything
                // is queued.
                let prepared = client().and_then(|c| Some((c, consent::anon_id().ok().flatten()?)));
                let wanted = || !flag.load(Ordering::Acquire);
                if !wanted() {
                    return;
                }
                // `begin` asks again under the consent lock, so a shutdown
                // that gave up while that lock was held by another process
                // still queues nothing.
                let telemetry = prepared
                    .and_then(|(c, anon_id)| Telemetry::begin(c, &anon_id, interval, wanted));
                // A handover nobody waits for (shutdown gave up) stops at
                // once, with no budget: its queue is dropped, and only a POST
                // the worker had already begun may complete.
                if let Err(mpsc::SendError(Some(late))) = ready.send(telemetry) {
                    late.shutdown_by(Instant::now());
                }
            });
        Starting {
            ready: receiver,
            abandoned,
        }
    }

    /// Wait for the start and shut the telemetry down, all within one
    /// [`DEFAULT_SHUTDOWN_BUDGET`]. A start still blocked at the deadline is
    /// abandoned and sends nothing: the starter reads the flag before it
    /// begins and again under the consent lock before it queues the started
    /// event, and a start already past both checks shuts itself down when
    /// it cannot hand over.
    pub fn shutdown(self) {
        let deadline = Instant::now() + DEFAULT_SHUTDOWN_BUDGET;
        match self.ready.recv_timeout(DEFAULT_SHUTDOWN_BUDGET) {
            Ok(Some(telemetry)) => telemetry.shutdown_by(deadline),
            Ok(None) | Err(RecvTimeoutError::Disconnected) => {}
            Err(RecvTimeoutError::Timeout) => {
                self.abandoned.store(true, Ordering::Release);
                // A start that handed over since the timeout is stopped with
                // no budget; dropped unseen it would flush for a full budget
                // past the deadline instead.
                if let Ok(Some(late)) = self.ready.try_recv() {
                    late.shutdown_by(Instant::now());
                }
            }
        }
    }
}
