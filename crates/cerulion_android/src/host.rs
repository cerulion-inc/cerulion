// SPDX-License-Identifier: AGPL-3.0-only
//! Process-global asynchronous facade for app and JNI callers.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, TryLockError};
use std::thread;

use crate::error::{codes, EmbedError};
use crate::runtime::{EmbeddedRuntime, LastError, RuntimeState, Status};

static HOST: OnceLock<Host> = OnceLock::new();

struct Host {
    files_dir: PathBuf,
    robot_name: String,
    runtime: Mutex<EmbeddedRuntime>,
    status: Arc<Mutex<Status>>,
    lifecycle: Mutex<Lifecycle>,
    #[cfg(test)]
    worker_hold: Mutex<Option<Box<dyn FnOnce() + Send>>>,
}

/// Async-operation lifecycle. The lock is held only to make and record a
/// transition decision or to read it with the status, never across runtime
/// work.
enum Lifecycle {
    /// No start or stop worker is in flight.
    Idle,
    /// A start worker is running; `stop_requested` records a stop that
    /// arrived while startup was in flight, applied when startup settles.
    Starting { stop_requested: bool },
    /// A stop worker is running.
    Stopping,
}

/// The transition `Host::stop` takes, decided under the
/// lifecycle lock.
#[derive(Debug, PartialEq, Eq)]
enum StopDecision {
    /// Start is in flight: record the request and return OK; the start
    /// worker performs the stop once startup settles.
    Defer,
    /// A stop worker is already in flight.
    Busy,
    /// Spawn a stop worker now.
    Spawn,
    /// Nothing started that could be stopped.
    BadState,
}

fn stop_decision(lifecycle: &Lifecycle, state: RuntimeState) -> StopDecision {
    match lifecycle {
        Lifecycle::Starting { .. } => StopDecision::Defer,
        Lifecycle::Stopping => StopDecision::Busy,
        Lifecycle::Idle => match state {
            RuntimeState::Running | RuntimeState::Failed | RuntimeState::Finished => {
                StopDecision::Spawn
            }
            _ => StopDecision::BadState,
        },
    }
}

impl Host {
    fn new(files_dir: PathBuf, robot_name: String, runtime: EmbeddedRuntime) -> Self {
        let status = runtime.status_handle();
        Self {
            files_dir,
            robot_name,
            runtime: Mutex::new(runtime),
            status,
            lifecycle: Mutex::new(Lifecycle::Idle),
            #[cfg(test)]
            worker_hold: Mutex::new(None),
        }
    }

    /// The status an app reads. While the lifecycle is Starting or Stopping
    /// the state reads as that transition, whatever the runtime recorded;
    /// the other fields read as recorded. A worker records Running, Stopped
    /// or Failed while it still holds the runtime lock, and sets the
    /// lifecycle to Idle only after releasing it.
    /// The lifecycle and the status are read under both locks, in the order
    /// `start` and `stop` take them, so a reported Running, Stopped or Failed
    /// means the worker has released the runtime lock and the lifecycle is
    /// Idle: a publish or a restart issued next is not answered BUSY by it.
    fn observed_status(&self) -> Status {
        let lifecycle = lock_lifecycle(&self.lifecycle);
        let mut status = lock_status(&self.status).clone();
        match *lifecycle {
            Lifecycle::Idle => {}
            Lifecycle::Starting { .. } => status.state = RuntimeState::Starting,
            Lifecycle::Stopping => status.state = RuntimeState::Stopping,
        }
        status
    }

    /// TEST SEAM: run the hook a test installed, once, in the next start or
    /// stop worker after its runtime call returns and before the worker
    /// releases the runtime lock.
    fn hold_worker_for_test(&self) {
        #[cfg(test)]
        if let Some(hold) = self
            .worker_hold
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .take()
        {
            hold();
        }
    }

    fn same_identity(&self, files_dir: &Path, robot_name: &str) -> bool {
        self.files_dir == files_dir && self.robot_name == robot_name
    }

    fn start(&'static self, config_json: &str) -> i32 {
        {
            let mut lifecycle = lock_lifecycle(&self.lifecycle);
            if !matches!(*lifecycle, Lifecycle::Idle) {
                return codes::BUSY;
            }
            if matches!(
                lock_status(&self.status).state,
                RuntimeState::Starting | RuntimeState::Running | RuntimeState::Stopping
            ) {
                return codes::BAD_STATE;
            }
            {
                let mut status = lock_status(&self.status);
                status.state = RuntimeState::Starting;
                status.last_error = None;
            }
            *lifecycle = Lifecycle::Starting {
                stop_requested: false,
            };
        }
        let config_json = config_json.to_owned();
        let result = thread::Builder::new()
            .name("cerulion-android-start".to_string())
            .spawn(move || {
                let result = catch_unwind(AssertUnwindSafe(|| {
                    let mut runtime = lock_runtime(&self.runtime);
                    let result = runtime.start_accepted(&config_json);
                    self.hold_worker_for_test();
                    result
                }));
                match result {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => set_status_error(&self.status, &error),
                    Err(payload) => {
                        // A panic may have left a partially-started runtime
                        // behind: tear it down before reporting the failure.
                        // `stop` rejects a runtime that is still Starting, so
                        // mark the panic first; teardown then runs and the
                        // panic status is re-recorded over stop's Stopped.
                        set_panic_status_with(&self.status, &*payload);
                        let teardown = catch_unwind(AssertUnwindSafe(|| {
                            let mut runtime = lock_runtime(&self.runtime);
                            runtime.stop().map(|_| ())
                        }));
                        if let Ok(Err(error)) = teardown {
                            tracing::warn!(
                                error = %error,
                                "teardown after a start panic reported an error"
                            );
                        }
                        set_panic_status_with(&self.status, &*payload);
                    }
                }
                // A stop that arrived during startup is honored here, in
                // the same worker. The flag read and the lifecycle write are
                // ONE critical section: splitting them lets a stop land
                // between the read and `Idle` and be silently lost.
                let deferred = {
                    let mut lifecycle = lock_lifecycle(&self.lifecycle);
                    match *lifecycle {
                        Lifecycle::Starting {
                            stop_requested: true,
                        } => {
                            *lifecycle = Lifecycle::Stopping;
                            true
                        }
                        _ => {
                            *lifecycle = Lifecycle::Idle;
                            false
                        }
                    }
                };
                if deferred {
                    if matches!(
                        lock_status(&self.status).state,
                        RuntimeState::Running | RuntimeState::Failed | RuntimeState::Finished
                    ) {
                        let result = catch_unwind(AssertUnwindSafe(|| {
                            let mut runtime = lock_runtime(&self.runtime);
                            runtime.stop().map(|_| ())
                        }));
                        match result {
                            Ok(Ok(())) => {}
                            Ok(Err(error)) => set_status_error(&self.status, &error),
                            Err(payload) => set_panic_status_with(&self.status, &*payload),
                        }
                    }
                    *lock_lifecycle(&self.lifecycle) = Lifecycle::Idle;
                }
            });
        if let Err(error) = result {
            let error = EmbedError::Spawn(format!("failed to spawn start worker: {error}"));
            set_status_error(&self.status, &error);
            // A stop accepted while the lifecycle was Starting must not be
            // dropped with the flag: consume `stop_requested` and land in the
            // stopped state, tearing down any surviving component first.
            // As in the start worker, the flag read and the lifecycle write
            // are one critical section, and a deferred stop holds the
            // lifecycle at Stopping until its teardown finishes, so a
            // concurrent retry cannot start a run this stop then tears down.
            let deferred = {
                let mut lifecycle = lock_lifecycle(&self.lifecycle);
                match *lifecycle {
                    Lifecycle::Starting {
                        stop_requested: true,
                    } => {
                        *lifecycle = Lifecycle::Stopping;
                        true
                    }
                    _ => {
                        *lifecycle = Lifecycle::Idle;
                        false
                    }
                }
            };
            if deferred
                && matches!(
                    lock_status(&self.status).state,
                    RuntimeState::Running | RuntimeState::Failed | RuntimeState::Finished
                )
            {
                let result = catch_unwind(AssertUnwindSafe(|| {
                    let mut runtime = lock_runtime(&self.runtime);
                    runtime.stop().map(|_| ())
                }));
                match result {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => set_status_error(&self.status, &error),
                    Err(payload) => set_panic_status_with(&self.status, &*payload),
                }
            }
            if deferred {
                *lock_lifecycle(&self.lifecycle) = Lifecycle::Idle;
            }
            return error.code();
        }
        codes::OK
    }

    fn stop(&'static self) -> i32 {
        {
            let mut lifecycle = lock_lifecycle(&self.lifecycle);
            match stop_decision(&lifecycle, lock_status(&self.status).state) {
                StopDecision::Defer => {
                    if let Lifecycle::Starting { stop_requested } = &mut *lifecycle {
                        *stop_requested = true;
                    }
                    return codes::OK;
                }
                StopDecision::Busy => return codes::BUSY,
                StopDecision::BadState => return codes::BAD_STATE,
                StopDecision::Spawn => *lifecycle = Lifecycle::Stopping,
            }
        }
        let result = thread::Builder::new()
            .name("cerulion-android-stop".to_string())
            .spawn(move || {
                let result = catch_unwind(AssertUnwindSafe(|| {
                    let mut runtime = lock_runtime(&self.runtime);
                    let result = runtime.stop().map(|_| ());
                    self.hold_worker_for_test();
                    result
                }));
                match result {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => set_status_error(&self.status, &error),
                    Err(_) => set_panic_status(&self.status),
                }
                *lock_lifecycle(&self.lifecycle) = Lifecycle::Idle;
            });
        if let Err(error) = result {
            let error = EmbedError::Spawn(format!("failed to spawn stop worker: {error}"));
            set_status_error(&self.status, &error);
            *lock_lifecycle(&self.lifecycle) = Lifecycle::Idle;
            return error.code();
        }
        codes::OK
    }

    /// Take the runtime lock for an app publish. The lifecycle check and the
    /// runtime try_lock run under the lifecycle guard so a stop cannot be
    /// accepted in between; then the guard drops before the publish itself.
    /// The lifecycle lock is taken with a wait (its holders only decide a
    /// transition or read the status); the runtime lock with try_lock, so a
    /// Java stop() is never made to wait out a publish. Once the caller
    /// holds the returned runtime lock, a stop accepted afterwards just waits
    /// on that lock in its worker: the publish is ordered before the stop and
    /// teardown cannot interleave.
    fn publish_runtime(&self) -> Result<MutexGuard<'_, EmbeddedRuntime>, i32> {
        let lifecycle = lock_lifecycle(&self.lifecycle);
        if !matches!(*lifecycle, Lifecycle::Idle) {
            return Err(codes::BUSY);
        }
        match self.runtime.try_lock() {
            Ok(runtime) => Ok(runtime),
            Err(TryLockError::WouldBlock) => Err(codes::BUSY),
            Err(TryLockError::Poisoned(error)) => Ok(error.into_inner()),
        }
    }

    /// Publish a message that `prepare` produces from the topic's JSON length
    /// limit. The limit lookup, `prepare` and the publish all run under ONE
    /// runtime lock, so the limit `prepare` checks against is the limit of
    /// the run that publishes: a stop or restart cannot land between the
    /// check and the publish. `prepare` runs only when the publish itself
    /// could proceed (BUSY, BAD_STATE and the unknown-topic code are
    /// returned first), and its `Err(code)` is returned as is.
    fn publish_prepared(
        &self,
        topic: &str,
        prepare: impl FnOnce(usize) -> Result<String, i32>,
    ) -> i32 {
        let mut runtime = match self.publish_runtime() {
            Ok(runtime) => runtime,
            Err(code) => return code,
        };
        let limit = match runtime.publish_json_limit(topic) {
            Ok(limit) => limit,
            Err(error) => return error.code(),
        };
        let json = match prepare(limit) {
            Ok(json) => json,
            Err(code) => return code,
        };
        runtime
            .publish(topic, &json)
            .map(|_| codes::OK)
            .unwrap_or_else(|error| error.code())
    }
}

/// Serializes `init`: a concurrent loser must observe the winner's host as
/// fully installed, never see `configure_shm_dir` repoint the resolved
/// shared-memory directory out from under an in-flight `HOST.set`.
static INIT_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

pub(crate) fn init(files_dir: &str, robot_name: &str) -> i32 {
    guarded_code(
        || {
            let _init_guard = INIT_LOCK
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            let files_dir = PathBuf::from(files_dir);
            if let Some(host) = HOST.get() {
                return if host.same_identity(&files_dir, robot_name) {
                    codes::OK
                } else {
                    codes::ALREADY_INITIALIZED
                };
            }
            let runtime = match EmbeddedRuntime::new(files_dir.clone(), robot_name.to_owned()) {
                Ok(runtime) => runtime,
                Err(error) => return error.code(),
            };
            if let Err(error) = configure_shm_dir(&files_dir) {
                return error.code();
            }
            let identity_dir = files_dir.clone();
            let host = Host::new(files_dir, robot_name.to_owned(), runtime);
            if HOST.set(host).is_ok() {
                codes::OK
            } else {
                let host = HOST.get().expect("host set after initialization race");
                if host.same_identity(&identity_dir, robot_name) {
                    codes::OK
                } else {
                    codes::ALREADY_INITIALIZED
                }
            }
        },
        panic_code,
    )
}

/// Point Cerulion's named shared-memory directory at a path the app owns.
///
/// An app uid cannot write `/data/local/tmp` and receives no `TMPDIR`, so
/// without this every named-object operation fails `EACCES`. The directory
/// is created mode 0700: named objects are files, and no other uid may
/// reach them. It is created on every platform so a host-side run lays out
/// the same files; only Android resolves named objects against it.
///
/// Repeating `init` with the same directory is a no-op inside
/// `set_shm_dir`; a different directory after resolution is refused, which
/// surfaces here as `ALREADY_INITIALIZED`.
fn configure_shm_dir(files_dir: &Path) -> Result<(), EmbedError> {
    let shm_dir = files_dir.join("cerulion-shm");
    let mut builder = std::fs::DirBuilder::new();
    builder.recursive(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        builder.mode(0o700);
    }
    builder.create(&shm_dir).map_err(|error| {
        EmbedError::InvalidArgument(format!(
            "cannot create shared-memory directory '{}': {error}",
            shm_dir.display()
        ))
    })?;
    #[cfg(target_os = "android")]
    cerulion_core::posix_shm::set_shm_dir(&shm_dir).map_err(|error| {
        EmbedError::AlreadyInitialized(format!(
            "named shared-memory directory is already resolved elsewhere: {error}"
        ))
    })?;
    Ok(())
}

pub(crate) fn start(config_json: &str) -> i32 {
    guarded_code(
        || match HOST.get() {
            Some(host) => host.start(config_json),
            None => codes::NOT_INITIALIZED,
        },
        panic_code,
    )
}

/// Publish the message `prepare` builds from the topic's JSON length limit,
/// with the limit and the publish bound to the same runtime (see
/// `Host::publish_prepared`). The JNI layer uses it to refuse an oversized
/// Java string before copying it.
pub(crate) fn publish_prepared(
    topic: &str,
    prepare: impl FnOnce(usize) -> Result<String, i32>,
) -> i32 {
    guarded_code(
        || match HOST.get() {
            Some(host) => host.publish_prepared(topic, prepare),
            None => codes::NOT_INITIALIZED,
        },
        panic_code,
    )
}

pub(crate) fn status() -> String {
    match catch_unwind(AssertUnwindSafe(status_inner)) {
        Ok(status) => status,
        Err(_) => {
            panic_code();
            failed_status_json("panic at embedding boundary")
        }
    }
}

pub(crate) fn stop() -> i32 {
    guarded_code(
        || match HOST.get() {
            Some(host) => host.stop(),
            None => codes::NOT_INITIALIZED,
        },
        panic_code,
    )
}

pub(crate) fn panic_code() -> i32 {
    if let Some(host) = HOST.get() {
        set_panic_status(&host.status);
    }
    codes::PANIC
}

fn status_inner() -> String {
    let Some(host) = HOST.get() else {
        return "{\"state\":\"uninitialized\"}".to_string();
    };
    serde_json::to_string(&host.observed_status())
        .unwrap_or_else(|_| failed_status_json("status serialization failed"))
}

fn failed_status_json(message: &str) -> String {
    serde_json::json!({
        "state": "failed",
        "last_error": {
            "code": codes::PANIC,
            "message": message
        }
    })
    .to_string()
}

fn guarded_code(operation: impl FnOnce() -> i32, on_panic: impl FnOnce() -> i32) -> i32 {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(code) => code,
        Err(_) => on_panic(),
    }
}

fn lock_lifecycle(lifecycle: &Mutex<Lifecycle>) -> MutexGuard<'_, Lifecycle> {
    lifecycle.lock().unwrap_or_else(|error| error.into_inner())
}

fn lock_runtime(runtime: &Mutex<EmbeddedRuntime>) -> std::sync::MutexGuard<'_, EmbeddedRuntime> {
    runtime.lock().unwrap_or_else(|error| error.into_inner())
}

fn lock_status(status: &Arc<Mutex<Status>>) -> std::sync::MutexGuard<'_, Status> {
    status.lock().unwrap_or_else(|error| error.into_inner())
}

fn set_status_error(status: &Arc<Mutex<Status>>, error: &EmbedError) {
    set_status_code(status, error.code(), &error.to_string());
}

fn set_status_code(status: &Arc<Mutex<Status>>, code: i32, message: &str) {
    let mut status = lock_status(status);
    status.state = RuntimeState::Failed;
    status.last_error = Some(LastError {
        code,
        message: message.to_string(),
    });
}

fn set_panic_status(status: &Arc<Mutex<Status>>) {
    set_status_code(status, codes::PANIC, "panic at embedding boundary");
}

/// Record a worker panic as `Failed`, carrying the panic payload's own
/// message when one is extractable (the `&'static str`/`String` forms
/// `panic!` produces); falls back to a fixed text otherwise.
fn set_panic_status_with(status: &Arc<Mutex<Status>>, payload: &dyn std::any::Any) {
    let message = if let Some(text) = payload.downcast_ref::<&'static str>() {
        format!("panic at embedding boundary: {text}")
    } else if let Some(text) = payload.downcast_ref::<String>() {
        format!("panic at embedding boundary: {text}")
    } else {
        "panic at embedding boundary".to_string()
    };
    set_status_code(status, codes::PANIC, &message);
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    fn test_host() -> Host {
        let directory = tempdir().expect("temporary files directory");
        let files_dir = directory.path().to_path_buf();
        let runtime =
            EmbeddedRuntime::new(files_dir.clone(), "host_test".to_string()).expect("runtime");
        // A dropped TempDir deletes the directory, and a real start() then
        // fails validate_paths; keep it for the rest of the process.
        std::mem::forget(directory);
        Host::new(files_dir, "host_test".to_string(), runtime)
    }

    fn static_test_host() -> &'static Host {
        Box::leak(Box::new(test_host()))
    }

    #[test]
    fn init_creates_an_app_owned_shm_directory() {
        let directory = tempdir().expect("temporary files directory");
        let files_dir = directory.path().to_path_buf();

        configure_shm_dir(&files_dir).expect("first configuration");
        // A same-directory repeat is OK, which is what an init retried
        // with the same files directory hits.
        configure_shm_dir(&files_dir).expect("repeated configuration");

        let shm_dir = files_dir.join("cerulion-shm");
        let metadata = std::fs::metadata(&shm_dir).expect("shm directory exists");
        assert!(metadata.is_dir());
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(
                metadata.permissions().mode() & 0o777,
                0o700,
                "other uids must not reach the named-object files"
            );
        }
    }

    #[test]
    fn concurrent_inits_install_exactly_one_host() {
        // `INIT_LOCK` serializes the pair, so one caller installs its host
        // and the loser observes it and reports ALREADY_INITIALIZED; without
        // serialization the loser could also repoint the shared-memory
        // directory mid-race.
        let dir_a = tempdir().expect("files dir A");
        let dir_b = tempdir().expect("files dir B");
        let path_a = dir_a.path().to_str().expect("utf8 path").to_string();
        let path_b = dir_b.path().to_str().expect("utf8 path").to_string();
        std::mem::forget(dir_a);
        std::mem::forget(dir_b);
        let barrier = std::sync::Barrier::new(2);
        let (code_a, code_b) = std::thread::scope(|scope| {
            let a = scope.spawn(|| {
                barrier.wait();
                init(&path_a, "racer")
            });
            let b = scope.spawn(|| {
                barrier.wait();
                init(&path_b, "racer")
            });
            (a.join().expect("thread A"), b.join().expect("thread B"))
        });
        let codes = [code_a, code_b];
        assert_eq!(codes.iter().filter(|code| **code == codes::OK).count(), 1);
        assert_eq!(
            codes
                .iter()
                .filter(|code| **code == codes::ALREADY_INITIALIZED)
                .count(),
            1
        );
        let winner = if code_a == codes::OK { path_a } else { path_b };
        let host = HOST.get().expect("a host is installed");
        assert!(host.same_identity(std::path::Path::new(&winner), "racer"));
    }

    #[test]
    fn publish_reports_busy_without_waiting() {
        let host = test_host();
        let _runtime = host.runtime.lock().expect("runtime lock");
        assert_eq!(publish_on(&host, "/topic", "{}"), codes::BUSY);
    }

    #[test]
    fn publish_reports_busy_through_any_pending_transition() {
        let host = test_host();
        lock_status(&host.status).state = RuntimeState::Running;
        for lifecycle in [
            Lifecycle::Starting {
                stop_requested: false,
            },
            Lifecycle::Starting {
                stop_requested: true,
            },
            Lifecycle::Stopping,
        ] {
            *lock_lifecycle(&host.lifecycle) = lifecycle;
            assert_eq!(publish_on(&host, "/topic", "{}"), codes::BUSY);
        }
        *lock_lifecycle(&host.lifecycle) = Lifecycle::Idle;
    }

    /// A plain publish through the JNI entry point's path: the message
    /// needs no length check.
    fn publish_on(host: &Host, topic: &str, json: &str) -> i32 {
        host.publish_prepared(topic, |_| Ok(json.to_string()))
    }

    #[test]
    fn publish_prepared_preserves_the_non_running_error() {
        // The JNI pre-check must surface the same refusal the publish would:
        // on a non-running runtime that is BAD_STATE, never ENCODE or an
        // OK that skips the check.
        let host = test_host();
        let error = host
            .runtime
            .lock()
            .expect("runtime lock")
            .publish_json_limit("/topic")
            .expect_err("a stopped runtime refuses the limit");
        assert_eq!(error.code(), codes::BAD_STATE);
        assert_eq!(publish_on(&host, "/topic", "{}"), codes::BAD_STATE);
    }

    #[test]
    fn publish_prepared_refuses_before_preparing_the_message() {
        // NOT_INITIALIZED before init. (Only checked when no earlier test
        // installed the singleton.)
        if HOST.get().is_none() {
            assert_eq!(
                publish_prepared("/topic", |_| unreachable!("no host")),
                codes::NOT_INITIALIZED,
                "uninitialized"
            );
        }
        let host = test_host();
        let refuse = |_: usize| -> Result<String, i32> {
            unreachable!("a refused publish must not prepare (copy) the message")
        };

        // BAD_STATE on a stopped runtime.
        assert_eq!(host.publish_prepared("/topic", refuse), codes::BAD_STATE);

        // BUSY while a lifecycle transition is in flight.
        *host.lifecycle.lock().expect("lifecycle") = Lifecycle::Starting {
            stop_requested: false,
        };
        assert_eq!(host.publish_prepared("/topic", refuse), codes::BUSY);
        *host.lifecycle.lock().expect("lifecycle") = Lifecycle::Idle;

        // BUSY while a thread holds the runtime lock.
        {
            let _runtime = host.runtime.lock().expect("runtime lock");
            assert_eq!(host.publish_prepared("/topic", refuse), codes::BUSY);
        }

        // UNKNOWN_TOPIC on a running runtime for a topic with no publisher.
        host.runtime
            .lock()
            .expect("runtime lock")
            .mark_running_for_test();
        assert_eq!(
            host.publish_prepared("/missing", refuse),
            codes::UNKNOWN_TOPIC,
            "unknown topic"
        );
    }

    #[test]
    fn publish_does_not_leak_the_lifecycle_guard() {
        let host = test_host();
        let _code = publish_on(&host, "/topic", "{}");
        assert!(host.lifecycle.try_lock().is_ok());
    }

    #[test]
    fn publish_reports_busy_while_start_in_flight() {
        let host = test_host();
        *lock_lifecycle(&host.lifecycle) = Lifecycle::Starting {
            stop_requested: false,
        };
        assert_eq!(publish_on(&host, "/topic", "{}"), codes::BUSY);
    }

    #[test]
    fn stop_decision_covers_every_lifecycle_and_state_arm() {
        assert_eq!(
            stop_decision(
                &Lifecycle::Starting {
                    stop_requested: false
                },
                RuntimeState::Starting
            ),
            StopDecision::Defer
        );
        assert_eq!(
            stop_decision(
                &Lifecycle::Starting {
                    stop_requested: true
                },
                RuntimeState::Starting
            ),
            StopDecision::Defer
        );
        assert_eq!(
            stop_decision(&Lifecycle::Stopping, RuntimeState::Running),
            StopDecision::Busy
        );
        assert_eq!(
            stop_decision(&Lifecycle::Idle, RuntimeState::Running),
            StopDecision::Spawn
        );
        assert_eq!(
            stop_decision(&Lifecycle::Idle, RuntimeState::Failed),
            StopDecision::Spawn
        );
        assert_eq!(
            stop_decision(&Lifecycle::Idle, RuntimeState::Finished),
            StopDecision::Spawn
        );
        for state in [
            RuntimeState::Idle,
            RuntimeState::Starting,
            RuntimeState::Stopping,
            RuntimeState::Stopped,
        ] {
            assert_eq!(
                stop_decision(&Lifecycle::Idle, state),
                StopDecision::BadState,
                "{state:?}"
            );
        }
    }

    #[test]
    fn start_reports_starting_before_returning_ok() {
        let host = static_test_host();
        {
            let mut status = lock_status(&host.status);
            status.state = RuntimeState::Failed;
            status.last_error = Some(LastError {
                code: codes::NETWORK,
                message: "old".to_string(),
            });
        }
        let config = r#"{"publish":[{"topic":"/host_test/heartbeat","schema":"std_msgs/String"}]}"#;
        assert_eq!(host.start(config), codes::OK);

        {
            let status = lock_status(&host.status);
            assert!(
                matches!(status.state, RuntimeState::Starting | RuntimeState::Running),
                "start returned OK with status state {:?}",
                status.state
            );
            assert_ne!(
                status
                    .last_error
                    .as_ref()
                    .map(|error| error.message.as_str()),
                Some("old")
            );
        }

        wait_for_observed(host, RuntimeState::Running);

        // The JNI publish path: the limit check and the publish must see the
        // same run, so no stop or restart may take the runtime while the
        // message is prepared against the limit. Pinned as: the runtime lock
        // is held inside `prepare`, which sees the active publisher's limit.
        let mut seen_limit = None;
        let code = host.publish_prepared("/host_test/heartbeat", |limit| {
            assert!(
                matches!(host.runtime.try_lock(), Err(TryLockError::WouldBlock)),
                "the runtime must stay locked between the limit check and the publish"
            );
            seen_limit = Some(limit);
            Ok(r#"{"data":"x"}"#.to_string())
        });
        assert_eq!(code, codes::OK);
        assert!(seen_limit.is_some_and(|limit| limit > 0));
        // An Err from `prepare` is returned as is, without publishing.
        assert_eq!(
            host.publish_prepared("/host_test/heartbeat", |_| Err(codes::ENCODE)),
            codes::ENCODE
        );
        assert_eq!(
            lock_status(&host.status)
                .published
                .get("/host_test/heartbeat"),
            Some(&1)
        );
        assert_eq!(host.stop(), codes::OK);
        wait_for_observed(host, RuntimeState::Stopped);
    }

    /// Poll the app-observed status until it reads `want`; panic on any
    /// other settled state (Failed, or Stopped while waiting for Running).
    fn wait_for_observed(host: &Host, want: RuntimeState) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let status = host.observed_status();
            if status.state == want {
                return;
            }
            assert!(
                matches!(
                    status.state,
                    RuntimeState::Starting | RuntimeState::Running | RuntimeState::Stopping
                ),
                "waiting for {want:?}, the runtime settled in {:?}: {:?}",
                status.state,
                status.last_error
            );
            assert!(
                std::time::Instant::now() < deadline,
                "runtime did not reach {want:?} within 10 seconds"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    /// Hold the next start or stop worker after its runtime call, with the
    /// runtime lock still held. The first receiver fires once the worker is
    /// held; the worker proceeds when the returned sender is dropped or sent.
    fn hold_next_worker(
        host: &Host,
    ) -> (std::sync::mpsc::Receiver<()>, std::sync::mpsc::Sender<()>) {
        let (held_tx, held_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
        *host.worker_hold.lock().expect("worker hold") = Some(Box::new(move || {
            held_tx.send(()).expect("test waits for the hold");
            let _ = release_rx.recv();
        }));
        (held_rx, release_tx)
    }

    /// Wait up to 10 seconds for the `worker` (start or stop) worker to
    /// reach the test hold. A worker that panics before the hold leaves the
    /// installed hook, and so its sender, alive: a blocking `recv` would
    /// then hang the test until the job timeout instead of failing it.
    fn wait_for_hold(held: &std::sync::mpsc::Receiver<()>, worker: &str) {
        wait_for_hold_within(held, worker, std::time::Duration::from_secs(10));
    }

    fn wait_for_hold_within(
        held: &std::sync::mpsc::Receiver<()>,
        worker: &str,
        timeout: std::time::Duration,
    ) {
        if let Err(error) = held.recv_timeout(timeout) {
            panic!("{worker} worker did not reach the hold within {timeout:?}: {error}");
        }
    }

    #[test]
    #[should_panic(expected = "start worker did not reach the hold within")]
    fn wait_for_hold_fails_when_the_worker_never_reaches_the_hold() {
        // The installed hook keeps the sender alive, as it does when a
        // worker panics before it runs the hook.
        let (_held_tx, held_rx) = std::sync::mpsc::channel::<()>();
        wait_for_hold_within(&held_rx, "start", std::time::Duration::from_millis(50));
    }

    #[test]
    fn running_is_reported_only_after_the_start_path_releases_the_runtime() {
        let host = static_test_host();
        let config = r#"{"publish":[{"topic":"/host_test/window","schema":"std_msgs/String"}]}"#;
        let (held, release) = hold_next_worker(host);
        assert_eq!(host.start(config), codes::OK);
        wait_for_hold(&held, "start");

        // The runtime has recorded Running and the start worker still holds
        // the runtime lock: a publish here is BUSY, so the app must not be
        // told Running yet.
        assert_eq!(lock_status(&host.status).state, RuntimeState::Running);
        assert_eq!(
            publish_on(host, "/host_test/window", r#"{"data":"x"}"#),
            codes::BUSY
        );
        assert_eq!(host.observed_status().state, RuntimeState::Starting);
        let reported: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&host.observed_status()).expect("json"))
                .expect("status json");
        assert_eq!(reported["state"], "starting");

        drop(release);
        wait_for_observed(host, RuntimeState::Running);
        // No retry: the first publish after the app reads Running succeeds.
        assert_eq!(
            publish_on(host, "/host_test/window", r#"{"data":"x"}"#),
            codes::OK
        );
        assert_eq!(host.stop(), codes::OK);
        wait_for_observed(host, RuntimeState::Stopped);
    }

    #[test]
    fn stopped_is_reported_only_after_the_stop_path_releases_the_runtime() {
        let host = static_test_host();
        let config =
            r#"{"publish":[{"topic":"/host_test/stop_window","schema":"std_msgs/String"}]}"#;
        assert_eq!(host.start(config), codes::OK);
        wait_for_observed(host, RuntimeState::Running);

        let (held, release) = hold_next_worker(host);
        assert_eq!(host.stop(), codes::OK);
        wait_for_hold(&held, "stop");
        assert_eq!(lock_status(&host.status).state, RuntimeState::Stopped);
        assert_eq!(host.start(config), codes::BUSY);
        assert_eq!(host.observed_status().state, RuntimeState::Stopping);

        drop(release);
        wait_for_observed(host, RuntimeState::Stopped);
        // The first restart after the app reads Stopped is accepted.
        assert_eq!(host.start(config), codes::OK);
        wait_for_observed(host, RuntimeState::Running);
        assert_eq!(host.stop(), codes::OK);
        wait_for_observed(host, RuntimeState::Stopped);
    }

    #[test]
    fn failed_is_reported_only_after_the_start_path_releases_the_runtime() {
        let host = static_test_host();
        let (held, release) = hold_next_worker(host);
        assert_eq!(host.start("{not json"), codes::OK);
        wait_for_hold(&held, "start");
        assert_eq!(lock_status(&host.status).state, RuntimeState::Failed);
        assert_eq!(host.observed_status().state, RuntimeState::Starting);

        drop(release);
        wait_for_observed(host, RuntimeState::Failed);
        assert_eq!(
            host.observed_status().last_error.map(|error| error.code),
            Some(codes::INVALID_ARGUMENT)
        );
        // The first restart after the app reads Failed is accepted.
        let config =
            r#"{"publish":[{"topic":"/host_test/fail_window","schema":"std_msgs/String"}]}"#;
        assert_eq!(host.start(config), codes::OK);
        wait_for_observed(host, RuntimeState::Running);
        assert_eq!(host.stop(), codes::OK);
        wait_for_observed(host, RuntimeState::Stopped);
    }

    #[test]
    fn stop_during_start_is_accepted_and_runtime_reaches_stopped() {
        let host = static_test_host();
        let config = r#"{"publish":[{"topic":"/host_test/heartbeat","schema":"std_msgs/String"}]}"#;
        assert_eq!(host.start(config), codes::OK);
        assert_eq!(host.stop(), codes::OK);
        wait_for_observed(host, RuntimeState::Stopped);
    }

    #[test]
    fn stop_while_idle_returns_bad_state_without_changing_status() {
        let host = static_test_host();
        let before = lock_status(&host.status).clone();
        assert_eq!(before.state, RuntimeState::Idle);

        assert_eq!(host.stop(), codes::BAD_STATE);

        assert_eq!(*lock_status(&host.status), before);
        assert!(matches!(*lock_lifecycle(&host.lifecycle), Lifecycle::Idle));
    }

    #[test]
    fn stop_while_stopping_returns_busy() {
        let host = static_test_host();
        lock_status(&host.status).state = RuntimeState::Running;
        *lock_lifecycle(&host.lifecycle) = Lifecycle::Stopping;

        assert_eq!(host.stop(), codes::BUSY);

        *lock_lifecycle(&host.lifecycle) = Lifecycle::Idle;
    }

    #[test]
    fn start_while_running_returns_bad_state_without_changing_status() {
        let host = static_test_host();
        lock_status(&host.status).state = RuntimeState::Running;
        let before = lock_status(&host.status).clone();

        assert_eq!(host.start("{}"), codes::BAD_STATE);

        assert_eq!(*lock_status(&host.status), before);
        assert!(matches!(*lock_lifecycle(&host.lifecycle), Lifecycle::Idle));
    }

    #[test]
    fn fallback_status_uses_panic_code() {
        let status: serde_json::Value =
            serde_json::from_str(&failed_status_json("panic at embedding boundary"))
                .expect("valid fallback JSON");
        assert_eq!(status["last_error"]["code"], codes::PANIC);
    }

    #[test]
    fn panic_guard_sets_status_and_returns_panic() {
        let host = test_host();
        let status = Arc::clone(&host.status);
        let code = guarded_code(
            || panic!("test panic"),
            || {
                set_panic_status(&status);
                codes::PANIC
            },
        );
        assert_eq!(code, codes::PANIC);
        let status = lock_status(&host.status);
        assert_eq!(status.state, RuntimeState::Failed);
        assert_eq!(
            status.last_error.as_ref().map(|error| error.code),
            Some(codes::PANIC)
        );
    }
}
