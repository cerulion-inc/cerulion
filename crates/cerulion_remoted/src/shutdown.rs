// SPDX-License-Identifier: AGPL-3.0-only
//! The serving loop's shutdown stage, and the hard-exit deadline of the
//! `cerulion-remoted` binary.
//!
//! ## The shutdown stage
//!
//! The accept loop of [`serve_planes`](crate::daemon::serve_planes) stops on a
//! stop request (the shutdown future resolved) or because the endpoint closed
//! on its own. Either way it drops the shutdown future before it logs anything,
//! then runs the
//! [`ShutdownStage`] of its [`OpsServing`](crate::OpsServing) before it
//! returns. The daemon's serving core then ends the rcpd e-stop delivery in
//! flight and stops the epoch poller, the entry closes the endpoint, and the
//! binary's runtime teardown waits for the work left on its blocking pool.
//!
//! While the stage runs, the ops server and its receipt sink are alive. After
//! a stop request the endpoint and its open connections are alive too, and
//! those connections keep serving while the stage runs; when the endpoint
//! closed on its own, they are already gone.
//!
//! The stage drives each [`ShutdownParticipant`] to settlement on a task of its
//! own, one after another in registration order, with no timeout and no abort.
//! A participant that fails, panics or is cancelled is reported in a
//! [`ShutdownStageError`], and never stops the ones after it from running. The
//! participants are the ones the host registered on the ops serving's stage.
//! Outside the stage, an ops connection ends when the endpoint closes, and a
//! verb still running on the blocking pool then holds the runtime teardown
//! until the verb returns or the hard-exit deadline ends the process.
//!
//! ## The hard-exit deadline
//!
//! The binary arms [`HARD_EXIT_ENV`]'s deadline at the first Ctrl-C or SIGTERM,
//! or, without one, when serving drops its shutdown future (the accept loop
//! stopped on its own, or serving never started), so the stage is bounded on
//! both paths. Past the deadline the process ends with exit status 4. It is a
//! separate failure policy from the graceful stop: it ends the process and
//! proves nothing finished. The daemon's own work ends with the process, except
//! a thread in an uninterruptible kernel wait, such as an fsync on a stalled
//! disk, which holds the exit until that wait returns; a child process keeps
//! running.

use std::fmt;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

/// Work that must finish before the daemon tears its planes down.
pub trait ShutdownParticipant: Send + Sync + 'static {
    /// A fixed name for logs and the failure report, e.g. "shell".
    fn name(&self) -> &'static str;

    /// Called once, after the accept loop stopped and before the rcpd e-stop
    /// delivery and the epoch poller are stopped and the entry closes the
    /// endpoint, which may already be closed (see the module doc). Ok means its
    /// cleanup completed; Err means it did not.
    fn shut_down(&self) -> Pin<Box<dyn Future<Output = std::io::Result<()>> + Send + '_>>;
}

/// The participants the serving loop awaits at shutdown, in registration
/// order. Clones share one list.
#[derive(Clone, Default)]
pub struct ShutdownStage {
    participants: Arc<Mutex<Vec<Arc<dyn ShutdownParticipant>>>>,
    /// Set under the `participants` lock when the stage runs.
    ran: Arc<AtomicBool>,
}

impl ShutdownStage {
    /// Adds `participant`; a participant added after the stage ran is never
    /// called, and that is logged at warn.
    pub fn register(&self, participant: Arc<dyn ShutdownParticipant>) {
        let mut participants = self
            .participants
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        if self.ran.load(Ordering::Acquire) {
            tracing::warn!(
                participant = participant.name(),
                "cerulion_remoted: a shutdown participant registered after the shutdown stage \
                 ran is never called"
            );
            return;
        }
        participants.push(participant);
    }

    /// Drives every participant's `shut_down` to settlement, in registration
    /// order, each on a task of its own, with no timeout and no abort. The list
    /// is taken once, under the lock, as the run starts, a poisoned lock
    /// included. Returns every participant that did not complete.
    pub(crate) async fn run(&self) -> Result<(), ShutdownStageError> {
        let participants = {
            let mut participants = self
                .participants
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            self.ran.store(true, Ordering::Release);
            std::mem::take(&mut *participants)
        };
        let mut failures = Vec::new();
        for participant in participants {
            let name = participant.name();
            let settled = tokio::spawn(async move { participant.shut_down().await }).await;
            let kind = match settled {
                Ok(Ok(())) => continue,
                Ok(Err(error)) => ShutdownFailureKind::Failed(error),
                Err(join) if join.is_panic() => ShutdownFailureKind::Panicked,
                Err(_) => ShutdownFailureKind::Cancelled,
            };
            failures.push(ShutdownFailure {
                participant: name,
                kind,
            });
        }
        if failures.is_empty() {
            Ok(())
        } else {
            Err(ShutdownStageError { failures })
        }
    }
}

/// Every participant that did not complete, in registration order.
#[derive(Debug)]
pub struct ShutdownStageError {
    /// One entry per participant that did not complete.
    pub failures: Vec<ShutdownFailure>,
}

/// One participant that did not complete, and how.
#[derive(Debug)]
pub struct ShutdownFailure {
    /// The participant's [`ShutdownParticipant::name`].
    pub participant: &'static str,
    /// How it did not complete.
    pub kind: ShutdownFailureKind,
}

/// How a participant did not complete. None of these is completed cleanup.
#[derive(Debug)]
pub enum ShutdownFailureKind {
    /// Its `shut_down` returned this error.
    Failed(std::io::Error),
    /// Its `shut_down` panicked.
    Panicked,
    /// Its task was cancelled before it settled.
    Cancelled,
}

impl fmt::Display for ShutdownFailureKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShutdownFailureKind::Failed(error) => write!(f, "failed: {error}"),
            ShutdownFailureKind::Panicked => f.write_str("panicked"),
            ShutdownFailureKind::Cancelled => f.write_str("cancelled"),
        }
    }
}

impl fmt::Display for ShutdownStageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("shutdown participants did not complete:")?;
        for failure in &self.failures {
            write!(f, " {} {};", failure.participant, failure.kind)?;
        }
        Ok(())
    }
}

impl std::error::Error for ShutdownStageError {}

/// The environment variable setting the hard-exit deadline of the
/// `cerulion-remoted` binary, in whole milliseconds (a positive integer).
pub const HARD_EXIT_ENV: &str = "CERULION_REMOTED_HARD_EXIT_MS";

/// The nominal share of the default deadline reserved for the participants the
/// host registers. It is a term of the arithmetic below, not a bound on a
/// participant.
const STAGE_ALLOWANCE: Duration = Duration::from_secs(18);

/// The nominal share for the endpoint close: iroh's bound on waiting for its
/// close notifications to be acknowledged, about 3 s with the default
/// transport parameters.
const ENDPOINT_CLOSE_ALLOWANCE: Duration = Duration::from_secs(3);

/// The nominal share for thread scheduling and the runtime teardown of the
/// tasks left when serving stops.
const SCHEDULING_ALLOWANCE: Duration = Duration::from_secs(4);

/// The default hard-exit deadline, 30 s, chosen as the sum of nominal shares
/// one after another: the stage's participants (18 s), the wait for the task
/// of an rcpd e-stop delivery that the stop has already ended, which gives the
/// delivery no more time (5 s), the endpoint close (about 3 s) and
/// scheduling (4 s). The shares bound no real work and their sum proves no
/// cleanup completed: a participant may keep awaiting work past any of them,
/// and that work stays pending until the process exits. Past the deadline, or
/// on a second stop signal, the binary ends the process with exit status 4,
/// the explicit failure exit. A participant that needs more than its share
/// needs this deadline raised with it. An ops request that runs to its full
/// [`OPS_SESSION_DEADLINE`](crate::ops::OPS_SESSION_DEADLINE) is cut the same
/// way: the runtime teardown waits for it on the blocking pool, and the
/// deadline ends the process first.
pub const DEFAULT_HARD_EXIT: Duration = Duration::from_secs(
    STAGE_ALLOWANCE.as_secs()
        + crate::rcp::CONNECT_TIMEOUT.as_secs()
        + ENDPOINT_CLOSE_ALLOWANCE.as_secs()
        + SCHEDULING_ALLOWANCE.as_secs(),
);

/// The hard-exit deadline from a [`HARD_EXIT_ENV`] value: a positive whole
/// number of milliseconds, else [`DEFAULT_HARD_EXIT`]. A set, non-blank value
/// that is not one is logged at warn and the default is used, as
/// `cerulion-netd` does with its own deadline; a blank one takes the default
/// silently.
pub fn resolve_hard_exit(env_value: Option<&str>) -> Duration {
    let Some(value) = env_value.map(str::trim).filter(|v| !v.is_empty()) else {
        return DEFAULT_HARD_EXIT;
    };
    parse_hard_exit(value).unwrap_or_else(|| {
        tracing::warn!(
            variable = HARD_EXIT_ENV,
            value = ?value,
            default_ms = DEFAULT_HARD_EXIT.as_millis() as u64,
            "cerulion_remoted: the hard-exit deadline is not a positive whole number of \
             milliseconds; using the default"
        );
        DEFAULT_HARD_EXIT
    })
}

/// The deadline a trimmed [`HARD_EXIT_ENV`] value names: a positive whole
/// number of milliseconds, else `None`. Pure: no logging.
fn parse_hard_exit(value: &str) -> Option<Duration> {
    match value.parse::<u64>() {
        Ok(ms) if ms > 0 => Some(Duration::from_millis(ms)),
        _ => None,
    }
}

/// The hard-exit deadline this process's environment sets ([`HARD_EXIT_ENV`]).
pub fn hard_exit_from_env() -> Duration {
    resolve_hard_exit(std::env::var(HARD_EXIT_ENV).ok().as_deref())
}

/// The environment variable that adds a probe participant to the daemon's
/// shutdown stage: `ok`, `fail` or `stall`, for a probe that completes, fails
/// or never finishes. The probe logs whether the rcpd e-stop sender had
/// already been stopped when it ran.
#[cfg(feature = "test-seam")]
#[doc(hidden)]
pub const PROBE_FOR_TEST_ENV: &str = "CERULION_REMOTED_SHUTDOWN_PROBE_FOR_TEST";

/// The participant `PROBE_FOR_TEST_ENV` adds.
#[cfg(feature = "test-seam")]
struct ProbeForTest {
    outcome: String,
    rcpd: Option<Arc<crate::rcp::RcpdEstop>>,
}

#[cfg(feature = "test-seam")]
impl ShutdownParticipant for ProbeForTest {
    fn name(&self) -> &'static str {
        "shutdown-probe"
    }

    fn shut_down(&self) -> Pin<Box<dyn Future<Output = std::io::Result<()>> + Send + '_>> {
        Box::pin(async move {
            let rcpd_stopped = self.rcpd.as_ref().map(|rcpd| rcpd.is_stopping());
            tracing::warn!(
                outcome = %self.outcome,
                rcpd_stopped = ?rcpd_stopped,
                "cerulion_remoted: the shutdown probe runs (test seam)"
            );
            match self.outcome.as_str() {
                "stall" => std::future::pending::<std::io::Result<()>>().await,
                "fail" => Err(std::io::Error::other("the shutdown probe failed")),
                _ => Ok(()),
            }
        })
    }
}

/// Register a [`ProbeForTest`] on `stage` when [`PROBE_FOR_TEST_ENV`] is set.
#[cfg(feature = "test-seam")]
pub(crate) fn register_probe_for_test(
    stage: &ShutdownStage,
    rcpd: Option<Arc<crate::rcp::RcpdEstop>>,
) {
    if let Ok(outcome) = std::env::var(PROBE_FOR_TEST_ENV) {
        stage.register(Arc::new(ProbeForTest { outcome, rcpd }));
    }
}

/// The environment variable that makes the daemon close its own endpoint
/// once it listens, so its accept loop stops with no stop request.
#[cfg(feature = "test-seam")]
#[doc(hidden)]
pub const CLOSE_ENDPOINT_FOR_TEST_ENV: &str = "CERULION_REMOTED_CLOSE_ENDPOINT_FOR_TEST";

/// Close a clone of `endpoint` on a task of its own when
/// [`CLOSE_ENDPOINT_FOR_TEST_ENV`] is set.
#[cfg(feature = "test-seam")]
pub(crate) fn close_endpoint_for_test(endpoint: &cerulion_link::Endpoint) {
    if std::env::var_os(CLOSE_ENDPOINT_FOR_TEST_ENV).is_some() {
        let endpoint = endpoint.clone();
        tokio::spawn(async move { endpoint.close().await });
    }
}

/// The environment variable that sets, in milliseconds, how long the binary's
/// exit waits for its log line in place of its 100 ms; a value that is not a
/// positive whole number keeps the 100 ms.
#[cfg(feature = "test-seam")]
#[doc(hidden)]
pub const EXIT_LOG_GRACE_FOR_TEST_ENV: &str = "CERULION_REMOTED_EXIT_LOG_GRACE_MS_FOR_TEST";

#[cfg(test)]
mod tests {
    use super::*;

    type Log = Arc<Mutex<Vec<&'static str>>>;

    /// Records its name after yielding `yields` times, then settles as
    /// `outcome` says: `ok`, `fail` or `panic`.
    struct Recorder {
        name: &'static str,
        yields: usize,
        outcome: &'static str,
        log: Log,
    }

    impl ShutdownParticipant for Recorder {
        fn name(&self) -> &'static str {
            self.name
        }

        fn shut_down(&self) -> Pin<Box<dyn Future<Output = std::io::Result<()>> + Send + '_>> {
            Box::pin(async move {
                for _ in 0..self.yields {
                    tokio::task::yield_now().await;
                }
                self.log.lock().unwrap().push(self.name);
                match self.outcome {
                    "fail" => Err(std::io::Error::other("recorder failed")),
                    "panic" => panic!("recorder panicked"),
                    _ => Ok(()),
                }
            })
        }
    }

    fn register(
        stage: &ShutdownStage,
        log: &Log,
        name: &'static str,
        yields: usize,
        outcome: &'static str,
    ) {
        let log = log.clone();
        stage.register(Arc::new(Recorder {
            name,
            yields,
            outcome,
            log,
        }));
    }

    fn stage_of(participants: &[(&'static str, usize, &'static str)]) -> (ShutdownStage, Log) {
        let log = Log::default();
        let stage = ShutdownStage::default();
        for &(name, yields, outcome) in participants {
            register(&stage, &log, name, yields, outcome);
        }
        (stage, log)
    }

    #[tokio::test]
    async fn the_stage_awaits_each_participant_in_registration_order() {
        // The first participant yields longest: a stage that ran them at once
        // would record it last.
        let (stage, log) = stage_of(&[("a", 8, "ok"), ("b", 0, "ok"), ("c", 2, "ok")]);
        assert!(stage.run().await.is_ok());
        assert_eq!(*log.lock().unwrap(), ["a", "b", "c"]);
    }

    #[tokio::test]
    async fn a_failure_or_a_panic_skips_no_later_participant_and_each_is_reported() {
        let (stage, log) = stage_of(&[
            ("a", 0, "fail"),
            ("b", 0, "panic"),
            ("c", 0, "ok"),
            ("d", 0, "fail"),
        ]);
        let error = stage.run().await.unwrap_err();
        assert_eq!(*log.lock().unwrap(), ["a", "b", "c", "d"]);
        let reported: Vec<_> = error
            .failures
            .iter()
            .map(|f| (f.participant, f.kind.to_string()))
            .collect();
        assert_eq!(
            reported,
            [
                ("a", "failed: recorder failed".to_string()),
                ("b", "panicked".to_string()),
                ("d", "failed: recorder failed".to_string()),
            ]
        );
    }

    #[tokio::test]
    async fn a_participant_registered_after_the_stage_ran_is_never_called() {
        let (stage, log) = stage_of(&[("a", 0, "ok")]);
        assert!(stage.run().await.is_ok());
        register(&stage, &log, "late", 0, "ok");
        assert!(stage.run().await.is_ok());
        assert_eq!(*log.lock().unwrap(), ["a"]);
    }

    #[tokio::test]
    async fn a_poisoned_stage_still_runs_its_participants() {
        let (stage, log) = stage_of(&[("a", 0, "ok")]);
        let holder = stage.clone();
        let _ = std::thread::spawn(move || {
            let _held = holder.participants.lock().unwrap();
            panic!("poisons the stage's lock");
        })
        .join();
        assert!(stage.participants.is_poisoned());
        assert!(stage.run().await.is_ok());
        assert_eq!(*log.lock().unwrap(), ["a"]);
    }

    #[test]
    fn the_deadline_takes_positive_milliseconds_and_falls_back_otherwise() {
        assert_eq!(resolve_hard_exit(None), DEFAULT_HARD_EXIT);
        assert_eq!(resolve_hard_exit(Some(" ")), DEFAULT_HARD_EXIT);
        assert_eq!(resolve_hard_exit(Some("250")), Duration::from_millis(250));
        assert_eq!(
            resolve_hard_exit(Some(" 1500 ")),
            Duration::from_millis(1500)
        );
        for refused in ["0", "-5", "1.5", "2s", "off", "18446744073709551616"] {
            assert_eq!(
                resolve_hard_exit(Some(refused)),
                DEFAULT_HARD_EXIT,
                "{refused}"
            );
        }
    }

    #[test]
    fn only_a_positive_whole_number_of_milliseconds_names_a_deadline() {
        assert_eq!(parse_hard_exit("1"), Some(Duration::from_millis(1)));
        for refused in ["0", "-1", "1.0", "1ms", ""] {
            assert_eq!(parse_hard_exit(refused), None, "{refused}");
        }
    }

    #[test]
    fn the_default_deadline_is_the_stop_steps_one_after_another() {
        assert_eq!(DEFAULT_HARD_EXIT, Duration::from_secs(30));
        assert_eq!(crate::rcp::CONNECT_TIMEOUT, Duration::from_secs(5));
    }
}
