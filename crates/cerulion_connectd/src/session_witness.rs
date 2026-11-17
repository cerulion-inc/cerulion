// SPDX-License-Identifier: AGPL-3.0-only
//! Private, durable final session witnesses retained independently of the robot.

use std::fmt::Display;
use std::fs::File;
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use cerulion_ops_proto::{RecordingWitness, ServerFrame, SessionExit};

use crate::{sanitize_peer_text, ConnectError, ConnectResult};

/// An owned directory descriptor used to create immutable final witness files.
///
/// All methods perform blocking filesystem work. Callers keep them off async
/// executor threads and authenticate the robot before trusting its commitment.
pub struct WitnessStore {
    pub(crate) dir: File,
    pub(crate) path: PathBuf,
}

impl WitnessStore {
    /// Open a private directory belonging to this user, creating it as 0700.
    ///
    /// The parent must exist. A final-component symlink is refused. The directory
    /// and ancestor entries are synced before a caller can depend on this store.
    pub fn open(path: &Path) -> ConnectResult<Self> {
        use rustix::fs::{fstat, open, Mode, OFlags};
        use std::os::unix::fs::DirBuilderExt;
        match std::fs::DirBuilder::new().mode(0o700).create(path) {
            Ok(()) => {}
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {}
            Err(error) => return Err(error.into()),
        }
        let dir = File::from(
            open(
                path,
                OFlags::RDONLY | OFlags::DIRECTORY | OFlags::NOFOLLOW | OFlags::CLOEXEC,
                Mode::empty(),
            )
            .map_err(io::Error::from)?,
        );
        let metadata = fstat(&dir).map_err(io::Error::from)?;
        if metadata.st_uid != rustix::process::geteuid().as_raw() || metadata.st_mode & 0o077 != 0 {
            return Err(protocol_error(
                "recording witness directory must belong to this user and be private",
            ));
        }
        dir.sync_all()?;
        for parent in path.canonicalize()?.ancestors().skip(1) {
            File::open(parent)?.sync_all()?;
        }
        Ok(Self {
            dir,
            path: path.to_path_buf(),
        })
    }

    /// Save a matching final witness and Exit in a new 0600 file.
    ///
    /// Robot and session IDs contain at most 128 ASCII letters, digits, hyphens
    /// or underscores. Validation precedes file creation; an existing name is
    /// never replaced. Success includes syncing the file and its directory.
    /// An I/O failure can leave a partial file, which is retained and never
    /// overwritten by a later call.
    pub fn persist(
        &self,
        robot_eid: &str,
        witness: &RecordingWitness,
        exit: &SessionExit,
    ) -> ConnectResult<PathBuf> {
        use rustix::fs::{openat, Mode, OFlags};
        if !safe_id(&witness.session_id)
            || !safe_id(robot_eid)
            || witness.head != exit.recording_head
            || witness.events != exit.events
        {
            return Err(protocol_error(
                "robot supplied an invalid final recording witness",
            ));
        }
        ServerFrame::Exit(exit.clone())
            .validate()
            .map_err(protocol_error)?;
        let name = format!("{robot_eid}-{}.json", witness.session_id);
        let mut file = File::from(
            openat(
                &self.dir,
                name.as_str(),
                OFlags::WRONLY | OFlags::CREATE | OFlags::EXCL | OFlags::NOFOLLOW | OFlags::CLOEXEC,
                Mode::RUSR | Mode::WUSR,
            )
            .map_err(io::Error::from)?,
        );
        serde_json::to_writer(
            &mut file,
            &serde_json::json!({
                "robot_eid":robot_eid, "session_id":witness.session_id,
                "head":witness.head, "events":witness.events, "exit":exit,
            }),
        )
        .map_err(protocol_error)?;
        file.write_all(b"\n")?;
        file.sync_all()?;
        self.dir.sync_all()?;
        Ok(self.path.join(name))
    }
}

pub(crate) fn safe_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 128
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"-_".contains(&b))
}

fn protocol_error(error: impl Display) -> ConnectError {
    ConnectError::Control(sanitize_peer_text(&error.to_string()))
}

#[cfg(test)]
#[path = "session_witness_test.rs"]
mod tests;
