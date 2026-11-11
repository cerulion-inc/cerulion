// SPDX-License-Identifier: AGPL-3.0-only
//! Native admission fence for an owned server's idle teardown.

use std::fs::{File, OpenOptions, TryLockError};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};

/// The owned server's stable fence, outside its provisioned robot-state directory.
pub fn fence_path(state_root: &Path) -> PathBuf {
    state_root.join("serving/serving.hold")
}

/// Work which must remain reachable until it settles.
#[derive(Clone, Copy, Debug)]
pub enum HoldReason {
    /// An accepted ops or RCP session.
    Session,
    /// A remote shell or exec, including its final drain and witness.
    Shell,
}

type Source = Arc<dyn Fn() -> bool + Send + Sync>;

struct State {
    file: File,
    count: usize,
    sources: Vec<Source>,
    locked: bool,
    drop_failures: FailureRegimeLatch,
}

impl State {
    fn refresh(&mut self) -> io::Result<()> {
        let held = self.count != 0 || self.sources.iter().any(|source| source());
        if held && !self.locked {
            self.file.try_lock_shared().map_err(lock_error)?;
            self.locked = true;
        } else if !held && self.locked {
            self.file.unlock()?;
            self.locked = false;
        }
        Ok(())
    }
}

/// One shared native lock for all work in a server process. The parent takes an
/// exclusive lock before idle stop; admission then refuses until it has reaped.
/// Never unlink the lock file: its inode is the cross-process fencing identity.
#[derive(Clone)]
pub struct ServingHolds(Arc<Mutex<State>>);

impl ServingHolds {
    /// Open the fence created before the parent starts the server.
    pub fn open(path: &Path) -> io::Result<Self> {
        Ok(Self(Arc::new(Mutex::new(State {
            file: open_fence(path)?,
            count: 0,
            sources: Vec::new(),
            locked: false,
            drop_failures: FailureRegimeLatch::default(),
        }))))
    }

    /// Admit work or refuse while the parent has committed idle teardown.
    pub fn acquire(&self, _reason: HoldReason) -> io::Result<ServingHold> {
        let mut state = self.0.lock().unwrap_or_else(|e| e.into_inner());
        state.count += 1;
        if let Err(error) = state.refresh() {
            state.count -= 1;
            return Err(error);
        }
        Ok(ServingHold(self.clone()))
    }

    /// Retain the fence while observable state needs serving. Every mutation
    /// which can make a source true must occur under an acquired work guard.
    /// Sources must not acquire this manager; they run under its mutex.
    pub fn register_source(&self, source: Source) -> io::Result<()> {
        let mut state = self.0.lock().unwrap_or_else(|e| e.into_inner());
        state.sources.push(source);
        state.refresh()
    }

    /// Refresh completed work even when no session has since disconnected.
    pub fn refresh(&self) -> io::Result<()> {
        self.0.lock().unwrap_or_else(|e| e.into_inner()).refresh()
    }
}

/// Keeps admitted work alive; dropping it refreshes the latch/owed-work sources
/// before releasing the last shared lock, closing the disconnect/idle race.
pub struct ServingHold(ServingHolds);

impl Drop for ServingHold {
    fn drop(&mut self) {
        let mut state = self.0 .0.lock().unwrap_or_else(|e| e.into_inner());
        state.count -= 1;
        match state.refresh() {
            Ok(()) => {
                if let Some(suppressed) = state.drop_failures.on_success() {
                    tracing::info!(suppressed, "remote serving hold release recovered");
                }
            }
            Err(error) => match state.drop_failures.on_failure() {
                RegimeDecision::Loud | RegimeDecision::StillFailing { .. } => {
                    tracing::error!(error = %error, total_failures = state.drop_failures.total_failures(), "remote serving hold release failed");
                }
                RegimeDecision::Suppressed { .. } => {
                    tracing::debug!(error = %error, total_failures = state.drop_failures.total_failures(), "remote serving hold release failed");
                }
            },
        }
    }
}

fn lock_error(error: TryLockError) -> io::Error {
    match error {
        TryLockError::WouldBlock => io::ErrorKind::WouldBlock.into(),
        TryLockError::Error(error) => error,
    }
}

fn open_fence(path: &Path) -> io::Result<File> {
    let parent = path
        .parent()
        .ok_or_else(|| io::Error::other("remote serving fence has no parent directory"))?;
    let mut directory = std::fs::DirBuilder::new();
    directory.recursive(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        directory.mode(0o700);
    }
    directory.create(parent)?;
    let mut options = OpenOptions::new();
    options.read(true).write(true).create(true).truncate(false);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
    }
    let file = options.open(path)?;
    if !file.metadata()?.is_file() {
        return Err(io::Error::other(
            "remote serving fence is not a regular file",
        ));
    }
    Ok(file)
}

/// Parent-owned exclusive admission fence, held through SIGTERM and child reap.
pub struct IdleFence(File);

impl IdleFence {
    /// Refuse idle teardown while the server holds admitted or owed work.
    pub fn try_acquire(path: &Path) -> io::Result<Option<Self>> {
        let file = open_fence(path)?;
        match file.try_lock() {
            Ok(()) => Ok(Some(Self(file))),
            Err(TryLockError::WouldBlock) => Ok(None),
            Err(TryLockError::Error(error)) => Err(error),
        }
    }
}

impl Drop for IdleFence {
    fn drop(&mut self) {
        let _ = self.0.unlock();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn work_guards_block_idle_and_exclusive_idle_refuses_new_admission() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("hold");
        for _ in 0..2 {
            let holds = ServingHolds::open(&path).unwrap();
            let a = holds.acquire(HoldReason::Session).unwrap();
            let b = holds.acquire(HoldReason::Shell).unwrap();
            assert!(IdleFence::try_acquire(&path).unwrap().is_none());
            drop(a);
            assert!(IdleFence::try_acquire(&path).unwrap().is_none());
            drop(b);
            let idle = IdleFence::try_acquire(&path).unwrap().unwrap();
            assert_eq!(
                holds.acquire(HoldReason::Session).err().unwrap().kind(),
                io::ErrorKind::WouldBlock
            );
            drop(idle);
            assert!(holds.acquire(HoldReason::Session).is_ok());
        }
    }

    #[test]
    fn owed_state_retains_the_fence_after_disconnect_until_observable_completion() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("hold");
        let holds = ServingHolds::open(&path).unwrap();
        let owing = Arc::new(AtomicBool::new(false));
        let source = Arc::clone(&owing);
        holds
            .register_source(Arc::new(move || source.load(Ordering::Acquire)))
            .unwrap();
        let session = holds.acquire(HoldReason::Session).unwrap();
        owing.store(true, Ordering::Release);
        drop(session);
        assert!(IdleFence::try_acquire(&path).unwrap().is_none());
        owing.store(false, Ordering::Release);
        holds.refresh().unwrap();
        assert!(IdleFence::try_acquire(&path).unwrap().is_some());
    }
}
