// SPDX-License-Identifier: AGPL-3.0-only
//! Private, durable final session witnesses retained independently of the robot.

use std::ffi::OsStr;
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
    /// The caller supplies an existing durable parent that can be opened for
    /// reading and syncing. A final-component symlink is refused. The store and
    /// its immediate parent are synced; higher ancestors need only be searchable.
    pub fn open(path: &Path) -> ConnectResult<Self> {
        use rustix::fs::{open, Mode, OFlags};
        let name = path
            .file_name()
            .ok_or_else(|| protocol_error("recording witness directory needs a final component"))?;
        let parent_path = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        let parent = File::from(
            open(
                parent_path,
                OFlags::RDONLY | OFlags::DIRECTORY | OFlags::CLOEXEC,
                Mode::empty(),
            )
            .map_err(io::Error::from)?,
        );
        let dir = private_directory(&parent, name)?;
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
    /// Files use lowercase, unpadded base32 of each full ID at
    /// `final-<encoded robot>/<encoded session>.json`. Components stay below
    /// 255 bytes and IDs remain distinct on case-insensitive filesystems.
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
        let robot_name = format!("final-{}", encoded_id(robot_eid));
        let robot_dir = private_directory(&self.dir, OsStr::new(&robot_name))?;
        let name = format!("{}.json", encoded_id(&witness.session_id));
        let mut file = File::from(
            openat(
                &robot_dir,
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
        robot_dir.sync_all()?;
        Ok(self.path.join(robot_name).join(name))
    }
}

// RFC 4648 base32, lowercase and without padding. Encoding every byte preserves
// the complete identifier without relying on filename case distinctions or a hash.
fn encoded_id(id: &str) -> String {
    const ALPHABET: &[u8; 32] = b"abcdefghijklmnopqrstuvwxyz234567";
    let mut encoded = String::with_capacity((id.len() * 8).div_ceil(5));
    let mut pending = 0u16;
    let mut bits = 0;
    for byte in id.bytes() {
        pending = (pending << 8) | u16::from(byte);
        bits += 8;
        while bits >= 5 {
            bits -= 5;
            encoded.push(ALPHABET[((pending >> bits) & 31) as usize] as char);
        }
    }
    if bits != 0 {
        encoded.push(ALPHABET[((pending << (5 - bits)) & 31) as usize] as char);
    }
    encoded
}

fn private_directory(parent: &File, name: &OsStr) -> ConnectResult<File> {
    use rustix::fs::{fstat, mkdirat, openat, Mode, OFlags};
    match mkdirat(parent, name, Mode::RUSR | Mode::WUSR | Mode::XUSR) {
        Ok(()) | Err(rustix::io::Errno::EXIST) => {}
        Err(error) => return Err(io::Error::from(error).into()),
    }
    let dir = File::from(
        openat(
            parent,
            name,
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
    // Sync even after EEXIST: a concurrent creator may not have synced its entry.
    parent.sync_all()?;
    Ok(dir)
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
