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
}

/// Async-operation lifecycle. The lock is held only to make
/// and record a transition decision, never across runtime work.
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
                    runtime.start_accepted(&config_json)
                }));
                match result {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => set_status_error(&self.status, &error),
                    Err(_) => set_panic_status(&self.status),
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
                            Err(_) => set_panic_status(&self.status),
                        }
                    }
                    *lock_lifecycle(&self.lifecycle) = Lifecycle::Idle;
                }
            });
        if let Err(error) = result {
            let error = EmbedError::Spawn(format!("failed to spawn start worker: {error}"));
            set_status_error(&self.status, &error);
            *lock_lifecycle(&self.lifecycle) = Lifecycle::Idle;
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
                    runtime.stop().map(|_| ())
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

    fn publish(&self, topic: &str, json: &str) -> i32 {
        // The check and the runtime try_lock run under the lifecycle guard so
        // a stop cannot be accepted in between; then the guard drops before
        // the publish itself, so a Java stop() is never made to wait out a
        // publish. Once this call holds the runtime lock, a stop accepted
        // afterwards just waits on that lock in its worker: the publish is
        // ordered before the stop and teardown cannot interleave.
        let mut runtime = {
            let lifecycle = match self.lifecycle.try_lock() {
                Ok(lifecycle) => lifecycle,
                Err(TryLockError::WouldBlock) => return codes::BUSY,
                Err(TryLockError::Poisoned(error)) => error.into_inner(),
            };
            if !matches!(*lifecycle, Lifecycle::Idle) {
                return codes::BUSY;
            }
            match self.runtime.try_lock() {
                Ok(runtime) => runtime,
                Err(TryLockError::WouldBlock) => return codes::BUSY,
                Err(TryLockError::Poisoned(error)) => error.into_inner(),
            }
        };
        runtime
            .publish(topic, json)
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

/// The publish-JSON length limit for `topic`, for the JNI pre-conversion
/// check. `None` when uninitialized, when the runtime is busy (the publish
/// itself then reports `BUSY`), or when the topic is unknown (the publish
/// then reports `UNKNOWN_TOPIC`): the pre-check only ever refuses a call the
/// real publish would refuse harder.
pub(crate) fn publish_json_limit(topic: &str) -> Option<usize> {
    let host = HOST.get()?;
    let runtime = match host.runtime.try_lock() {
        Ok(runtime) => runtime,
        Err(TryLockError::WouldBlock) => return None,
        Err(TryLockError::Poisoned(error)) => error.into_inner(),
    };
    runtime.publish_json_limit(topic).ok()
}

pub(crate) fn publish(topic: &str, json: &str) -> i32 {
    guarded_code(
        || match HOST.get() {
            Some(host) => host.publish(topic, json),
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
    let status = lock_status(&host.status);
    serde_json::to_string(&*status)
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
        assert_eq!(host.publish("/topic", "{}"), codes::BUSY);
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
            assert_eq!(host.publish("/topic", "{}"), codes::BUSY);
        }
        *lock_lifecycle(&host.lifecycle) = Lifecycle::Idle;
    }

    #[test]
    fn publish_does_not_leak_the_lifecycle_guard() {
        let host = test_host();
        let _code = host.publish("/topic", "{}");
        assert!(host.lifecycle.try_lock().is_ok());
    }

    #[test]
    fn publish_reports_busy_while_start_in_flight() {
        let host = test_host();
        *lock_lifecycle(&host.lifecycle) = Lifecycle::Starting {
            stop_requested: false,
        };
        assert_eq!(host.publish("/topic", "{}"), codes::BUSY);
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

        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            if lock_status(&host.status).state == RuntimeState::Running {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "runtime did not reach running within 10 seconds"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        assert_eq!(host.stop(), codes::OK);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let (state, last_error) = {
                let status = lock_status(&host.status);
                (status.state, status.last_error.clone())
            };
            let idle = matches!(*lock_lifecycle(&host.lifecycle), Lifecycle::Idle);
            if state == RuntimeState::Stopped && idle {
                break;
            }
            if state == RuntimeState::Failed {
                panic!("runtime failed during stop: {last_error:?}");
            }
            assert!(
                std::time::Instant::now() < deadline,
                "runtime did not reach stopped within 10 seconds"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    #[test]
    fn stop_during_start_is_accepted_and_runtime_reaches_stopped() {
        let host = static_test_host();
        let config = r#"{"publish":[{"topic":"/host_test/heartbeat","schema":"std_msgs/String"}]}"#;
        assert_eq!(host.start(config), codes::OK);
        assert_eq!(host.stop(), codes::OK);

        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            let status = lock_status(&host.status);
            let idle = matches!(*lock_lifecycle(&host.lifecycle), Lifecycle::Idle);
            if status.state == RuntimeState::Stopped && idle {
                break;
            }
            if matches!(status.state, RuntimeState::Failed) {
                panic!(
                    "runtime failed during deferred stop: {:?}",
                    status.last_error
                );
            }
            drop(status);
            assert!(
                std::time::Instant::now() < deadline,
                "runtime did not reach stopped within 10 seconds"
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
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
