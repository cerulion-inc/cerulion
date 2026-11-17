// SPDX-License-Identifier: AGPL-3.0-only
//! The writer's side of the robot-local file handoff in
//! `<state_root>/remoted/`: the directory check, the writers' lock, leaving
//! the request, taking it back and reading the result. Each function does one
//! file operation the run's machine asked for ([`super::Step`]) and reports
//! what it found in the machine's terms.

use std::fs::{File, OpenOptions};
use std::io::{self, Read as _, Write as _};
use std::os::unix::fs::{MetadataExt as _, OpenOptionsExt as _};
use std::path::{Path, PathBuf};

use cerulion_pairing::pair_code::{
    PairCodeRequest, PairCodeResult, MAX_FILE_BYTES, REQUEST_FILE, REQUEST_STAGING_PREFIX,
    RESULT_FILE, WRITER_LOCK_FILE,
};

use super::{result_verdict, DirState, FileFacts, Pulled, ResultSeen, Verdict, TOO_LONG};

/// How many times, in all, one leave writes its staging file, while a
/// starting daemon's removal takes it before the rename.
const LEAVE_TRIES: usize = 3;
/// Why a leave failed when every one of its [`LEAVE_TRIES`] staging files
/// was removed before its rename.
const STAGING_REMOVED: &str = "the staging file of the request was removed before it could be \
                               renamed into place, three times";
const _: () = assert!(LEAVE_TRIES == 3, "STAGING_REMOVED names LEAVE_TRIES");

/// The state of the remoted directory `dir`, through `symlink_metadata`, so a
/// symbolic link at its name is not followed.
pub(super) fn dir_state(dir: &Path) -> DirState {
    match std::fs::symlink_metadata(dir) {
        Err(e) if e.kind() == io::ErrorKind::NotFound => DirState::Missing,
        Ok(meta)
            if meta.is_dir()
                && meta.uid() == cerulion_hygiene::euid()
                && meta.mode() & 0o077 == 0 =>
        {
            DirState::Ready
        }
        Ok(_) | Err(_) => DirState::NotPrivate,
    }
}

/// Whether a lock file whose handle shows `is_file`, `uid` and `mode` is one
/// the writers lock, for a runner whose effective user is `euid`: a regular
/// file of that user that gives its group and others no permission bit.
pub(super) fn lock_is_private(is_file: bool, uid: u32, mode: u32, euid: u32) -> bool {
    is_file && uid == euid && mode & 0o077 == 0
}

/// The writers' lock file in `dir`, opened read and write without following a
/// symbolic link, created owner-only when missing and never truncated, and
/// refused unless [`lock_is_private`].
pub(super) fn open_lock(dir: &Path) -> io::Result<File> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open(dir.join(WRITER_LOCK_FILE))?;
    let meta = file.metadata()?;
    if !lock_is_private(
        meta.is_file(),
        meta.uid(),
        meta.mode(),
        cerulion_hygiene::euid(),
    ) {
        return Err(io::Error::other(format!(
            "{WRITER_LOCK_FILE} is not an owner-only file of this user"
        )));
    }
    Ok(file)
}

/// Try the exclusive lock on `file`: whether it is now held.
pub(super) fn try_lock(file: &File) -> io::Result<bool> {
    match file.try_lock() {
        Ok(()) => Ok(true),
        Err(std::fs::TryLockError::WouldBlock) => Ok(false),
        Err(std::fs::TryLockError::Error(e)) => Err(e),
    }
}

/// This writer's staging name for the request in `dir`.
fn staging_path(dir: &Path) -> PathBuf {
    dir.join(format!(
        "{REQUEST_STAGING_PREFIX}{}.tmp",
        std::process::id()
    ))
}

/// Remove `path`; no file there is no error.
fn remove_if_present(path: &Path) -> io::Result<()> {
    match std::fs::remove_file(path) {
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(()),
        removed => removed,
    }
}

/// Remove the result in `dir`, then leave `request` there: a new 0600 staging
/// file, written and synced, renamed onto the request's name. A staging file
/// a starting daemon removed before the rename is written again, for at most
/// [`LEAVE_TRIES`] writes in all. A write or a rename that fails removes the
/// staging file it made.
pub(super) fn leave(dir: &Path, request: &PairCodeRequest) -> io::Result<()> {
    leave_through(dir, request, |from, to| std::fs::rename(from, to))
}

/// [`leave`], with each rename of the staging file onto the request's name
/// done by `rename`.
pub(super) fn leave_through(
    dir: &Path,
    request: &PairCodeRequest,
    mut rename: impl FnMut(&Path, &Path) -> io::Result<()>,
) -> io::Result<()> {
    remove_if_present(&dir.join(RESULT_FILE))?;
    let staging = staging_path(dir);
    for _ in 0..LEAVE_TRIES {
        remove_if_present(&staging)?;
        let written = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
            .open(&staging)
            .and_then(|mut file| {
                file.write_all(&request.render())?;
                file.sync_all()
            });
        if let Err(e) = written {
            let _ = remove_if_present(&staging);
            return Err(e);
        }
        match rename(&staging, &dir.join(REQUEST_FILE)) {
            Ok(()) => return Ok(()),
            Err(e) if e.kind() == io::ErrorKind::NotFound => continue,
            Err(e) => {
                let _ = remove_if_present(&staging);
                return Err(e);
            }
        }
    }
    Err(io::Error::new(io::ErrorKind::NotFound, STAGING_REMOVED))
}

/// Whether a request is at its name in `dir`.
pub(super) fn request_present(dir: &Path) -> bool {
    std::fs::symlink_metadata(dir.join(REQUEST_FILE)).is_ok()
}

/// Take the request in `dir` back: rename it to this writer's staging name,
/// so the daemon cannot claim it, then remove that. A request that is gone,
/// or cannot be renamed, may have been claimed.
pub(super) fn pull_back(dir: &Path) -> Pulled {
    let staging = staging_path(dir);
    match std::fs::rename(dir.join(REQUEST_FILE), &staging) {
        Ok(()) => {
            if let Err(error) = remove_if_present(&staging) {
                tracing::debug!(
                    path = %staging.display(),
                    %error,
                    "cerulion pair --start: could not remove the request it took back; a \
                     starting daemon removes it"
                );
            }
            Pulled::Back
        }
        Err(error) => {
            if error.kind() != io::ErrorKind::NotFound {
                tracing::debug!(
                    path = %dir.join(REQUEST_FILE).display(),
                    %error,
                    "cerulion pair --start: could not take the request back; the daemon may \
                     claim it"
                );
            }
            Pulled::Claimed
        }
    }
}

/// Read the result in `dir`: opened without following a symbolic link,
/// inspected through its handle ([`result_verdict`]), read to at most one
/// byte past [`MAX_FILE_BYTES`] and parsed.
pub(super) fn read_result(dir: &Path) -> ResultSeen {
    let opened = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK | libc::O_CLOEXEC)
        .open(dir.join(RESULT_FILE));
    let file = match opened {
        Ok(file) => file,
        Err(e) if e.kind() == io::ErrorKind::NotFound => return ResultSeen::Gone,
        Err(e) if e.raw_os_error() == Some(libc::ELOOP) => {
            return ResultSeen::Unreadable("it is a symbolic link".into())
        }
        Err(e) => return ResultSeen::Unreadable(e.to_string()),
    };
    let meta = match file.metadata() {
        Ok(meta) => meta,
        Err(e) => return ResultSeen::Unreadable(e.to_string()),
    };
    let facts = FileFacts {
        is_file: meta.is_file(),
        nlink: meta.nlink(),
        uid: meta.uid(),
        mode: meta.mode(),
        len: meta.len(),
    };
    match result_verdict(&facts, cerulion_hygiene::euid()) {
        Verdict::Gone => return ResultSeen::Gone,
        Verdict::Unreadable(why) => return ResultSeen::Unreadable(why.into()),
        Verdict::Read => {}
    }
    let mut bytes = Vec::new();
    if let Err(e) = file.take(MAX_FILE_BYTES as u64 + 1).read_to_end(&mut bytes) {
        return ResultSeen::Unreadable(e.to_string());
    }
    if bytes.len() > MAX_FILE_BYTES {
        return ResultSeen::Unreadable(TOO_LONG.into());
    }
    match PairCodeResult::parse(&bytes) {
        Ok(result) => ResultSeen::Line(result),
        Err(e) => ResultSeen::Unreadable(e.to_string()),
    }
}

/// The port the robot's endpoint is bound to, from the public beacon facts
/// under `state_root`, when they name the endpoint `endpoint_id`; `None`
/// otherwise.
pub(super) fn port(state_root: &Path, endpoint_id: &[u8; 32]) -> Option<u16> {
    match cerulion_mdns::read_beacon_facts_at(state_root) {
        Ok(Some(facts)) if facts.eid.as_deref() == Some(hex::encode(endpoint_id).as_str()) => {
            facts.iroh_port.filter(|port| *port != 0)
        }
        Ok(_) => None,
        Err(error) => {
            tracing::debug!(
                %error,
                "cerulion pair --start: the robot's beacon facts could not be read; the port \
                 is shown as unknown"
            );
            None
        }
    }
}
