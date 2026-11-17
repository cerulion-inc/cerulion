// SPDX-License-Identifier: AGPL-3.0-only
//! Keep the latest durable robot commitment in a private, per-run directory.

use std::fmt::Display;
use std::fs::File;
use std::io::{self, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use cerulion_ops_proto::{RecordingWitness, ServerFrame};
use tokio::sync::watch;

use crate::session_witness::{safe_id, WitnessStore};
use crate::{sanitize_peer_text, ConnectError, ConnectResult};

// Leave two seconds for terminal/process cleanup before the CLI's five-second
// signal backstop. The same budget bounds writes and ordinary failure drains.
pub(crate) const IO_BUDGET: Duration = Duration::from_secs(3);

/// A checkpoint whose file and containing directory were synced.
#[derive(Clone)]
pub struct SavedCheckpoint {
    pub witness: RecordingWitness,
    pub path: PathBuf,
}

#[derive(Clone)]
enum Status {
    Pending,
    Saved(SavedCheckpoint),
    Done(Option<SavedCheckpoint>),
    Failed(String),
}

/// Both channels retain only their newest value. Filesystem work never runs
/// on the session's signal/output task, and notification cannot block a write.
/// Call `finish` and consume `progress` before shutting down the Tokio runtime.
/// Dropping this handle closes input but does not guarantee completion.
pub struct CheckpointWriter {
    input: Option<watch::Sender<Option<RecordingWitness>>>,
    status: watch::Receiver<Status>,
    reported: Option<RecordingWitness>,
    offered: Option<RecordingWitness>,
    done: bool,
    worker: tokio::task::JoinHandle<()>,
}

impl CheckpointWriter {
    /// Start a runtime-owned worker; no checkpoint file is created until an offer.
    pub fn new(root: &WitnessStore, robot_eid: String) -> ConnectResult<Self> {
        if !safe_id(&robot_eid) {
            return Err(protocol_error("checkpoint robot ID is invalid"));
        }
        let root_dir = root.dir.try_clone()?;
        let root_path = root.path.clone();
        let (input, receiver) = watch::channel(None);
        let (status, output) = watch::channel(Status::Pending);
        let worker = tokio::spawn(run(receiver, status, root_dir, root_path, robot_eid));
        Ok(Self {
            input: Some(input),
            status: output,
            reported: None,
            offered: None,
            done: false,
            worker,
        })
    }

    /// Queue the newest valid commitment for this writer's first session ID.
    /// An accepted offer is not a durability result; consume `progress` for that.
    pub fn offer(&mut self, witness: RecordingWitness) -> ConnectResult<()> {
        if self.done {
            return Err(protocol_error("recording checkpoint writer stopped"));
        }
        if !safe_id(&witness.session_id) {
            return Err(protocol_error("checkpoint session ID is invalid"));
        }
        ServerFrame::Witness {
            witness: witness.clone(),
        }
        .validate()
        .map_err(protocol_error)?;
        if self.offered.as_ref().is_some_and(|previous| {
            witness.session_id != previous.session_id
                || witness.events < previous.events
                || (witness.events == previous.events && witness.head != previous.head)
        }) {
            return Err(protocol_error(
                "recording checkpoint changed session or moved backwards",
            ));
        }
        self.input
            .as_ref()
            .ok_or_else(|| protocol_error("checkpoint received after session exit"))?
            .send(Some(witness.clone()))
            .map_err(|_| protocol_error("recording checkpoint writer stopped"))?;
        self.offered = Some(witness);
        Ok(())
    }

    /// Close input, allowing the worker to finish its newest pending commitment.
    pub fn finish(&mut self) {
        self.input.take();
    }

    /// Whether progress reported completion/failure, or `abort` was requested.
    pub fn is_done(&self) -> bool {
        self.done
    }

    /// Stop the async worker. An in-flight blocking write may still complete.
    pub fn abort(&mut self) {
        self.finish();
        self.worker.abort();
        self.done = true;
    }

    /// Observe durable progress until `is_done`; repeated identical saves are omitted.
    /// A timeout or write failure ends this writer without restarting the artifact.
    pub async fn progress(&mut self) -> ConnectResult<Option<SavedCheckpoint>> {
        if self.status.changed().await.is_err() {
            self.done = true;
            return Err(protocol_error("recording checkpoint writer stopped"));
        }
        let status = self.status.borrow_and_update().clone();
        let saved = match status {
            Status::Pending => return Ok(None),
            Status::Saved(saved) => Some(saved),
            Status::Done(saved) => {
                self.done = true;
                saved
            }
            Status::Failed(message) => {
                self.done = true;
                return Err(protocol_error(message));
            }
        };
        Ok(saved.filter(|saved| {
            if self.reported.as_ref() == Some(&saved.witness) {
                false
            } else {
                self.reported = Some(saved.witness.clone());
                true
            }
        }))
    }
}

impl Drop for CheckpointWriter {
    fn drop(&mut self) {
        // Closing the watch lets the worker finish its in-flight write and
        // newest pending commitment, even when the session future is cancelled.
        self.finish();
    }
}

async fn run(
    mut input: watch::Receiver<Option<RecordingWitness>>,
    status: watch::Sender<Status>,
    root_dir: File,
    root_path: PathBuf,
    robot_eid: String,
) {
    let mut root = Some((root_dir, root_path));
    let mut store = None;
    let mut latest = None;
    while input.changed().await.is_ok() {
        let Some(witness) = input.borrow_and_update().clone() else {
            continue;
        };
        let create = root.take();
        let previous = store.take();
        let robot = robot_eid.clone();
        let write = tokio::task::spawn_blocking(move || {
            let mut store = match previous {
                Some(store) => store,
                None => {
                    let (root_dir, root_path) = create.expect("first checkpoint owns its root");
                    Store::create(&root_dir, root_path)?
                }
            };
            let path = store.persist(&robot, &witness)?;
            Ok::<_, io::Error>((store, SavedCheckpoint { witness, path }))
        });
        let outcome = tokio::time::timeout(IO_BUDGET, write).await;
        match outcome {
            Ok(Ok(Ok((next, saved)))) => {
                store = Some(next);
                latest = Some(saved.clone());
                status.send_replace(Status::Saved(saved));
            }
            failure => {
                let message = match failure {
                    Ok(Ok(Err(error))) => format!("recording checkpoint failed: {error}"),
                    Ok(Err(error)) => format!("recording checkpoint worker failed: {error}"),
                    Err(_) => "recording checkpoint write timed out".to_owned(),
                    Ok(Ok(Ok(_))) => unreachable!(),
                };
                // A timed-out blocking syscall can still finish. Never start
                // another writer against that artifact after any failure.
                status.send_replace(Status::Failed(message));
                return;
            }
        }
    }
    status.send_replace(Status::Done(latest));
}

struct Store {
    dir: File,
    path: PathBuf,
    published: Option<File>,
}

impl Store {
    fn create(parent: &File, parent_path: PathBuf) -> io::Result<Self> {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_err(io::Error::other)?
            .as_nanos();
        for _ in 0..16 {
            let sequence = NEXT.fetch_add(1, Ordering::Relaxed);
            let name = format!("checkpoint-{}-{timestamp}-{sequence}", std::process::id());
            match Self::reserve(parent, parent_path.join(&name), &name) {
                Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                result => return result,
            }
        }
        Err(io::Error::new(
            io::ErrorKind::AlreadyExists,
            "cannot reserve a private checkpoint directory",
        ))
    }

    fn reserve(parent: &File, path: PathBuf, name: &str) -> io::Result<Self> {
        use rustix::fs::{fstat, mkdirat, openat, Mode, OFlags};
        mkdirat(parent, name, Mode::RUSR | Mode::WUSR | Mode::XUSR)?;
        let dir = File::from(openat(
            parent,
            name,
            OFlags::RDONLY | OFlags::DIRECTORY | OFlags::NOFOLLOW | OFlags::CLOEXEC,
            Mode::empty(),
        )?);
        let metadata = fstat(&dir)?;
        if metadata.st_uid != rustix::process::geteuid().as_raw() || metadata.st_mode & 0o077 != 0 {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "checkpoint directory ownership or permissions changed",
            ));
        }
        dir.sync_all()?;
        parent.sync_all()?;
        Ok(Self {
            dir,
            path,
            published: None,
        })
    }

    fn persist(&mut self, robot_eid: &str, witness: &RecordingWitness) -> io::Result<PathBuf> {
        use rustix::fs::{fstat, openat, renameat, statat, AtFlags, FileType, Mode, OFlags};
        let previous = self.published.as_ref().map(fstat).transpose()?;
        match statat(&self.dir, "witness.json", AtFlags::SYMLINK_NOFOLLOW) {
            Ok(metadata)
                if previous.as_ref().is_some_and(|old| {
                    old.st_dev == metadata.st_dev && old.st_ino == metadata.st_ino
                }) && FileType::from_raw_mode(metadata.st_mode) == FileType::RegularFile => {}
            Err(rustix::io::Errno::NOENT) if self.published.is_none() => {}
            _ => {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    "checkpoint destination was replaced",
                ));
            }
        }
        let mut file = File::from(openat(
            &self.dir,
            "pending.json",
            OFlags::WRONLY | OFlags::CREATE | OFlags::EXCL | OFlags::NOFOLLOW | OFlags::CLOEXEC,
            Mode::RUSR | Mode::WUSR,
        )?);
        serde_json::to_writer(
            &mut file,
            &serde_json::json!({
                "kind":"checkpoint", "robot_eid":robot_eid, "session_id":witness.session_id,
                "head":witness.head, "events":witness.events,
            }),
        )?;
        file.write_all(b"\n")?;
        file.sync_all()?;
        renameat(&self.dir, "pending.json", &self.dir, "witness.json")?;
        self.dir.sync_all()?;
        self.published = Some(file);
        Ok(self.path.join("witness.json"))
    }
}

fn protocol_error(error: impl Display) -> ConnectError {
    ConnectError::Control(sanitize_peer_text(&error.to_string()))
}

#[cfg(test)]
#[path = "session_checkpoint_test.rs"]
mod tests;
