// SPDX-License-Identifier: AGPL-3.0-only
//! Process-global asynchronous facade for app and JNI callers.

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock, TryLockError};
use std::thread;

use crate::error::{codes, EmbedError};
use crate::runtime::{EmbeddedRuntime, LastError, RuntimeState, Status};

static HOST: OnceLock<Host> = OnceLock::new();

struct Host {
    files_dir: PathBuf,
    robot_name: String,
    runtime: Mutex<EmbeddedRuntime>,
    status: Arc<Mutex<Status>>,
    busy: AtomicBool,
}

impl Host {
    fn new(files_dir: PathBuf, robot_name: String, runtime: EmbeddedRuntime) -> Self {
        let status = runtime.status_handle();
        Self {
            files_dir,
            robot_name,
            runtime: Mutex::new(runtime),
            status,
            busy: AtomicBool::new(false),
        }
    }

    fn same_identity(&self, files_dir: &Path, robot_name: &str) -> bool {
        self.files_dir == files_dir && self.robot_name == robot_name
    }

    fn start(&'static self, config_json: &str) -> i32 {
        if self
            .busy
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return codes::BUSY;
        }
        let config_json = config_json.to_owned();
        let result = thread::Builder::new()
            .name("cerulion-android-start".to_string())
            .spawn(move || {
                let result = catch_unwind(AssertUnwindSafe(|| {
                    let mut runtime = lock_runtime(&self.runtime);
                    runtime.start(&config_json)
                }));
                match result {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => set_status_error(&self.status, &error),
                    Err(_) => set_panic_status(&self.status),
                }
                self.busy.store(false, Ordering::Release);
            });
        if let Err(error) = result {
            self.busy.store(false, Ordering::Release);
            set_status_code(
                &self.status,
                codes::PANIC,
                &format!("failed to spawn start worker: {error}"),
            );
            return codes::PANIC;
        }
        codes::OK
    }

    fn stop(&'static self) -> i32 {
        if self
            .busy
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return codes::BUSY;
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
                self.busy.store(false, Ordering::Release);
            });
        if let Err(error) = result {
            self.busy.store(false, Ordering::Release);
            set_status_code(
                &self.status,
                codes::PANIC,
                &format!("failed to spawn stop worker: {error}"),
            );
            return codes::PANIC;
        }
        codes::OK
    }

    fn publish(&self, topic: &str, json: &str) -> i32 {
        if self.busy.load(Ordering::Acquire) {
            return codes::BUSY;
        }
        let mut runtime = match self.runtime.try_lock() {
            Ok(runtime) => runtime,
            Err(TryLockError::WouldBlock) => return codes::BUSY,
            Err(TryLockError::Poisoned(error)) => error.into_inner(),
        };
        if self.busy.load(Ordering::Acquire) {
            return codes::BUSY;
        }
        runtime
            .publish(topic, json)
            .map(|_| codes::OK)
            .unwrap_or_else(|error| error.code())
    }
}

pub(crate) fn init(files_dir: &str, robot_name: &str) -> i32 {
    guarded_code(
        || {
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

pub(crate) fn start(config_json: &str) -> i32 {
    guarded_code(
        || match HOST.get() {
            Some(host) => host.start(config_json),
            None => codes::NOT_INITIALIZED,
        },
        panic_code,
    )
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
            "{\"state\":\"failed\",\"last_error\":{\"code\":12,\"message\":\"panic at embedding boundary\"}}"
                .to_string()
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
    serde_json::to_string(&*status).unwrap_or_else(|_| {
        "{\"state\":\"failed\",\"last_error\":{\"code\":12,\"message\":\"status serialization failed\"}}"
            .to_string()
    })
}

fn guarded_code(operation: impl FnOnce() -> i32, on_panic: impl FnOnce() -> i32) -> i32 {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(code) => code,
        Err(_) => on_panic(),
    }
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
        drop(directory);
        Host::new(files_dir, "host_test".to_string(), runtime)
    }

    #[test]
    fn publish_reports_busy_without_waiting() {
        let host = test_host();
        let _runtime = host.runtime.lock().expect("runtime lock");
        assert_eq!(host.publish("/topic", "{}"), codes::BUSY);
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
