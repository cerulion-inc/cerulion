// SPDX-License-Identifier: AGPL-3.0-only
//! The PostHog client. Feature `posthog` off: [`Client::from_env`] is `None`
//! and every method on the (unconstructible) type is a no-op.

use std::time::Duration;

/// How a [`Client::shutdown`] ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShutdownOutcome {
    /// The worker drained its queue and exited within the budget.
    Flushed,
    /// The budget expired first. The worker was told to abort: it cancels
    /// the POST in flight (if any) and drops the rest of the queue, all
    /// counted in `queue_dropped`. `in_flight` is `0` when the worker
    /// acknowledged the abort inside the budget, nothing is on the wire
    /// after `shutdown` returns. It is the event count of the one batch
    /// whose POST was in progress only if the worker did not acknowledge
    /// in time (a starved thread); that batch cannot be reported on and up
    /// to `in_flight` events may still reach the server.
    TimedOut {
        /// Events in a POST the worker had not confirmed cancelled at the deadline.
        in_flight: usize,
    },
    /// Feature off, or already shut down: nothing to do.
    Noop,
}

/// Default [`Client::shutdown`] budget.
pub const DEFAULT_SHUTDOWN_BUDGET: Duration = Duration::from_millis(300);
/// Bounded queue depth; overflow drops the OLDEST queued event.
pub const QUEUE_CAPACITY: usize = 64;
/// Per-request HTTP timeout (connect + transfer).
pub const HTTP_TIMEOUT: Duration = Duration::from_secs(3);
/// Longest wait [`Client::shutdown`] accepts; a larger budget is clamped.
pub const MAX_SHUTDOWN_BUDGET: Duration = Duration::from_secs(60);
/// Default PostHog ingestion host.
pub const DEFAULT_HOST: &str = "https://us.i.posthog.com";

#[cfg(feature = "posthog")]
pub use enabled::Client;

#[cfg(feature = "posthog")]
mod enabled {
    use super::{ShutdownOutcome, DEFAULT_HOST, HTTP_TIMEOUT, MAX_SHUTDOWN_BUDGET, QUEUE_CAPACITY};
    use crate::consent;
    use crate::guard;
    use crate::payload::{self, Event};
    use crate::rfc3339;
    use crate::Common;
    use crate::{EventSpec, Props};
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
    use std::sync::mpsc;
    use std::sync::{Arc, Condvar, Mutex};
    use std::thread;
    use std::time::{Duration, Instant, SystemTime};
    use tokio::sync::Notify;

    struct State {
        events: VecDeque<Event>,
        closed: bool,
    }

    struct Shared {
        state: Mutex<State>,
        wake: Condvar,
        /// Raised by a timed-out `shutdown`; the worker's POST future is
        /// raced against `abort_wake` and dropped, the connection with it,
        /// the moment this is set. Checked again before every new POST.
        abort: AtomicBool,
        abort_wake: Notify,
        /// [`GATE_IDLE`], `n` (a POST of `n` events is in progress) or
        /// [`GATE_ABANDONED`]. The worker stores `n` before the POST and
        /// `compare_exchange(n, GATE_IDLE)` after it; a shutdown whose abort
        /// went unacknowledged does one `swap(GATE_ABANDONED)`. Exactly one
        /// side wins per batch, so a cancelled batch is counted as dropped by
        /// the worker or reported as `in_flight` by shutdown, never both.
        gate: AtomicUsize,
        api_key: String,
        batch_url: String,
        common: Common,
        http: reqwest::Client,
        queue_dropped: AtomicU64,
        post_failed: AtomicU64,
    }

    const GATE_IDLE: usize = 0;
    const GATE_ABANDONED: usize = usize::MAX;

    /// A live client: one bounded queue, one worker thread.
    pub struct Client {
        shared: Arc<Shared>,
        /// Closed by the worker when it exits; `shutdown` waits on it with a budget.
        done: mpsc::Receiver<()>,
        shut: bool,
    }

    impl Client {
        /// `None` (complete no-op) unless `POSTHOG_API_KEY` is set AND consent
        /// resolves enabled. Host from `POSTHOG_HOST`, default [`DEFAULT_HOST`].
        pub fn from_env(common: Common) -> Option<Client> {
            Client::from_env_or_key(None, common)
        }

        /// [`Client::from_env`], with `fallback_key` (a key baked into a
        /// release binary) used when `POSTHOG_API_KEY` is unset or blank.
        pub fn from_env_or_key(fallback_key: Option<&str>, common: Common) -> Option<Client> {
            let api_key = std::env::var("POSTHOG_API_KEY")
                .ok()
                .filter(|k| !k.trim().is_empty())
                .or_else(|| {
                    fallback_key
                        .filter(|k| !k.trim().is_empty())
                        .map(str::to_owned)
                })?;
            if !consent::status().enabled {
                return None;
            }
            let host = std::env::var("POSTHOG_HOST")
                .ok()
                .filter(|h| !h.trim().is_empty())
                .unwrap_or_else(|| DEFAULT_HOST.to_owned());
            Client::new(api_key, &host, common)
        }

        /// Construct against an explicit host (tests, or a surface that already
        /// resolved config). `None` if `host` is not an `https` URL (plain
        /// `http` is accepted for a loopback host only), a [`Common`] string
        /// fails the guard, or the HTTP client cannot be built.
        pub fn new(api_key: String, host: &str, common: Common) -> Option<Client> {
            if !host_is_allowed(host) {
                return None;
            }
            let strings = [
                Some(&common.surface),
                Some(&common.env),
                Some(&common.app_version),
                common.channel.as_ref(),
            ];
            if strings
                .into_iter()
                .flatten()
                .any(|s| guard::check_str(s).is_err())
            {
                return None;
            }
            // `rustls-no-provider` names no crypto provider; `ring` is the one
            // the rest of the release package set resolves to. A second
            // install is refused and harmless.
            let _ = rustls::crypto::ring::default_provider().install_default();
            let http = reqwest::Client::builder()
                .timeout(HTTP_TIMEOUT)
                .connect_timeout(HTTP_TIMEOUT)
                .no_gzip()
                .user_agent(format!("{}/{}", crate::LIB_NAME, crate::LIB_VERSION))
                .build()
                .ok()?;
            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .ok()?;
            let shared = Arc::new(Shared {
                state: Mutex::new(State {
                    events: VecDeque::with_capacity(QUEUE_CAPACITY),
                    closed: false,
                }),
                wake: Condvar::new(),
                abort: AtomicBool::new(false),
                abort_wake: Notify::new(),
                gate: AtomicUsize::new(GATE_IDLE),
                api_key,
                batch_url: format!("{}/batch", host.trim_end_matches('/')),
                common,
                http,
                queue_dropped: AtomicU64::new(0),
                post_failed: AtomicU64::new(0),
            });
            let (done_tx, done) = mpsc::channel();
            let worker_shared = Arc::clone(&shared);
            thread::Builder::new()
                .name("cerulion-telemetry".into())
                .spawn(move || {
                    worker(&worker_shared, &runtime);
                    drop(done_tx);
                })
                .ok()?;
            Some(Client {
                shared,
                done,
                shut: false,
            })
        }

        /// Queue an event for a known person (`distinct_id` = Supabase sub).
        /// Every id passes the guard's value rules or the event is dropped.
        pub fn capture(&self, spec: EventSpec, distinct_id: &str, props: Props) {
            let (uuid, ts) = stamp();
            self.enqueue(Event::capture(spec, distinct_id, uuid, ts, props));
        }

        /// Queue an event for an `anon:` id with `$process_person_profile: false`.
        pub fn capture_anonymous(&self, spec: EventSpec, anon_id: &str, props: Props) {
            let (uuid, ts) = stamp();
            self.enqueue(Event::capture_anonymous(spec, anon_id, uuid, ts, props));
        }

        /// `$create_alias`: merge `anon_id` into `sub`. Call exactly once, at login.
        pub fn alias(&self, sub: &str, anon_id: &str) {
            let (uuid, ts) = stamp();
            self.enqueue(Event::alias(sub, anon_id, uuid, ts));
        }

        /// `$set` with `$set_once` person properties (allowlist:
        /// [`payload::SET_ONCE_ALLOWLIST`]).
        pub fn set_once(&self, sub: &str, props: Props) {
            let (uuid, ts) = stamp();
            self.enqueue(Event::set_once(sub, uuid, ts, props));
        }

        /// Events dropped because the queue was full (oldest-first).
        pub fn queue_dropped(&self) -> u64 {
            self.shared.queue_dropped.load(Ordering::Relaxed)
        }

        /// Batches that failed to POST (network error or non-2xx).
        pub fn post_failed(&self) -> u64 {
            self.shared.post_failed.load(Ordering::Relaxed)
        }

        /// Close the queue and wait up to `budget` for the worker to drain it.
        ///
        /// The worker gets the first nine tenths of the budget to flush. If
        /// it has not exited by then it is told to abort: the POST in
        /// flight is cancelled (its future, and with it the connection, is
        /// dropped, nothing more is written) and the rest of the queue is
        /// discarded, all counted in [`Client::queue_dropped`]; the last
        /// tenth of the budget waits for the worker to acknowledge. In the
        /// normal case it does, and [`ShutdownOutcome::TimedOut`] carries
        /// `in_flight: 0`: no POST is on the wire once `shutdown` returns.
        /// Only if the worker thread is starved past the deadline does
        /// `shutdown` return without its acknowledgement, reporting the
        /// batch it was sending as `in_flight` (that batch is then the
        /// worker's to finish or cancel, uncounted). Nothing is spooled.
        /// Idempotent.
        pub fn shutdown(&mut self, budget: Duration) -> ShutdownOutcome {
            if self.shut {
                return ShutdownOutcome::Noop;
            }
            let budget = budget.min(MAX_SHUTDOWN_BUDGET);
            let deadline = Instant::now() + budget;
            self.shut = true;
            {
                let mut state = lock(&self.shared.state);
                state.closed = true;
            }
            self.shared.wake.notify_all();
            if self.wait_done(deadline - budget / 10) {
                return ShutdownOutcome::Flushed;
            }
            self.shared.abort.store(true, Ordering::Release);
            self.shared.abort_wake.notify_one();
            if self.wait_done(deadline) {
                return ShutdownOutcome::TimedOut { in_flight: 0 };
            }
            let in_flight = match self.shared.gate.swap(GATE_ABANDONED, Ordering::AcqRel) {
                GATE_IDLE | GATE_ABANDONED => 0,
                n => n,
            };
            // Past the deadline nothing may block: if the worker holds the
            // queue it sees `abort` before its next POST and counts the queue.
            if let Ok(mut state) = self.shared.state.try_lock() {
                let abandoned = state.events.len() as u64;
                state.events.clear();
                drop(state);
                self.shared
                    .queue_dropped
                    .fetch_add(abandoned, Ordering::Relaxed);
            }
            ShutdownOutcome::TimedOut { in_flight }
        }

        /// `true` once the worker has exited; `false` if `until` passes first.
        fn wait_done(&self, until: Instant) -> bool {
            let wait = until.saturating_duration_since(Instant::now());
            !matches!(
                self.done.recv_timeout(wait),
                Err(mpsc::RecvTimeoutError::Timeout)
            )
        }

        fn enqueue(&self, event: Option<Event>) {
            let Some(event) = event else {
                return;
            };
            let mut state = lock(&self.shared.state);
            if state.closed {
                return;
            }
            if state.events.len() >= QUEUE_CAPACITY {
                state.events.pop_front();
                self.shared.queue_dropped.fetch_add(1, Ordering::Relaxed);
            }
            state.events.push_back(event);
            drop(state);
            self.shared.wake.notify_one();
        }
    }

    impl Drop for Client {
        fn drop(&mut self) {
            self.shutdown(super::DEFAULT_SHUTDOWN_BUDGET);
        }
    }

    /// `https`, or `http` to a loopback address (a local test server).
    fn host_is_allowed(host: &str) -> bool {
        let Ok(url) = reqwest::Url::parse(host) else {
            return false;
        };
        match url.scheme() {
            "https" => url.host().is_some(),
            "http" => url.host_str().is_some_and(|h| {
                h == "localhost"
                    || h.trim_start_matches('[')
                        .trim_end_matches(']')
                        .parse::<std::net::IpAddr>()
                        .is_ok_and(|ip| ip.is_loopback())
            }),
            _ => false,
        }
    }

    fn stamp() -> (String, String) {
        (
            uuid::Uuid::now_v7().to_string(),
            rfc3339::format(SystemTime::now()),
        )
    }

    fn lock(m: &Mutex<State>) -> std::sync::MutexGuard<'_, State> {
        m.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Worker exit on abort: count the drained batch and whatever was queued
    /// behind it, then close the queue so nothing is stranded.
    fn abandon(shared: &Shared, drained: usize) {
        let mut state = lock(&shared.state);
        state.closed = true;
        let queued = state.events.len();
        state.events.clear();
        drop(state);
        shared
            .queue_dropped
            .fetch_add((drained + queued) as u64, Ordering::Relaxed);
    }

    fn worker(shared: &Shared, runtime: &tokio::runtime::Runtime) {
        loop {
            let batch: Vec<Event> = {
                let mut state = lock(&shared.state);
                while state.events.is_empty() && !state.closed {
                    state = shared
                        .wake
                        .wait(state)
                        .unwrap_or_else(|poisoned| poisoned.into_inner());
                }
                if state.events.is_empty() {
                    return;
                }
                state.events.drain(..).collect()
            };
            let n = batch.len();
            if shared.abort.load(Ordering::Acquire) {
                return abandon(shared, n);
            }
            let body = payload::batch_json(&shared.api_key, &batch, &shared.common);
            let request = shared.http.post(&shared.batch_url).json(&body);
            shared.gate.store(n, Ordering::Release);
            let outcome = runtime.block_on(async {
                tokio::select! {
                    biased;
                    () = shared.abort_wake.notified() => None,
                    sent = request.send() => Some(sent.map(|r| r.status().is_success()).unwrap_or(false)),
                }
            });
            let owned = shared
                .gate
                .compare_exchange(n, GATE_IDLE, Ordering::AcqRel, Ordering::Acquire)
                .is_ok();
            match outcome {
                None => return abandon(shared, if owned { n } else { 0 }),
                Some(false) => {
                    shared.post_failed.fetch_add(1, Ordering::Relaxed);
                }
                Some(true) => {}
            }
        }
    }
}

#[cfg(not(feature = "posthog"))]
pub use disabled::Client;

#[cfg(not(feature = "posthog"))]
mod disabled {
    use super::ShutdownOutcome;
    use crate::Common;
    use crate::{EventSpec, Props};
    use std::time::Duration;

    /// Feature off: cannot be constructed, so every method body is unreachable
    /// and the whole surface compiles to nothing.
    pub struct Client {
        _never: Never,
    }

    enum Never {}

    impl Client {
        /// Feature off: always `None`.
        pub fn from_env(_common: Common) -> Option<Client> {
            None
        }

        /// Feature off: always `None`.
        pub fn from_env_or_key(_fallback_key: Option<&str>, _common: Common) -> Option<Client> {
            None
        }

        /// Feature off: always `None`.
        pub fn new(_api_key: String, _host: &str, _common: Common) -> Option<Client> {
            None
        }

        pub fn capture(&self, _spec: EventSpec, _distinct_id: &str, _props: Props) {}

        pub fn capture_anonymous(&self, _spec: EventSpec, _anon_id: &str, _props: Props) {}

        pub fn alias(&self, _sub: &str, _anon_id: &str) {}

        pub fn set_once(&self, _sub: &str, _props: Props) {}

        pub fn queue_dropped(&self) -> u64 {
            0
        }

        pub fn post_failed(&self) -> u64 {
            0
        }

        pub fn shutdown(&mut self, _budget: Duration) -> ShutdownOutcome {
            ShutdownOutcome::Noop
        }
    }
}
