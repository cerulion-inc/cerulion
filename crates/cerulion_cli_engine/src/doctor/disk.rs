// SPDX-License-Identifier: AGPL-3.0-only
//! `disk.recordings`: the filesystem a recording started here is written to.
//!
//! `graph run --record` writes under `recordings/` relative to the current
//! directory, flashback captures go to `<workspace>/recordings/flashbacks`,
//! and `bag record` writes `recording.mcap` in the current directory. The
//! check reads the first of `./recordings` and `<workspace>/recordings` that
//! exists, a recording destination: a problem there is a `fail`. With
//! neither, it reads the current directory, where `bag record` writes and
//! where `graph run --record` would create `recordings/`; a problem there is a
//! `warn`, because a diagnosis is often run from a directory nothing records
//! in. `CERULION_FLASHBACK_DIR`, which moves flashback captures, is not read.
//! It reads `statvfs` (space and the read-only flag), `access(W_OK)` and, on
//! Linux, `statfs`'s filesystem type, where a tmpfs is memory that a reboot
//! empties.

use std::path::{Path, PathBuf};

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const RECORDINGS: Check = Check {
    id: "disk.recordings",
    why: "`graph run --record` and `bag record` write their bags under the current \
          directory; a bag that runs out of space or cannot be written ends the recording, \
          and a bag on tmpfs is held in memory and lost at reboot.",
    network: false,
    run,
};

/// What was read about the recording directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum DiskReading {
    /// The filesystem was read.
    Read {
        path: String,
        total: u64,
        available: u64,
        read_only: bool,
        writable: bool,
        tmpfs: bool,
    },
    /// The read failed.
    Unreadable { path: String, error: String },
}

/// The decision over the reading. `destination` is whether the path is an
/// existing recordings directory rather than the current directory.
pub(super) fn decide(r: &DiskReading, destination: bool) -> Outcome {
    let blocked = if destination {
        Status::Fail
    } else {
        Status::Warn
    };
    match r {
        DiskReading::Unreadable { path, error } => {
            Outcome::new(Status::Warn, format!("could not read {path}: {error}"))
                .with_fix(format!("run `df -h {path}` to see the filesystem"))
                .reading("path", Reading::Text(path.clone()))
                .reading("recordings_dir", Reading::Bool(destination))
        }
        DiskReading::Read {
            path,
            total,
            available,
            read_only,
            writable,
            tmpfs,
        } => {
            let base = |status, summary: String| {
                Outcome::new(status, summary)
                    .reading("path", Reading::Text(path.clone()))
                    .reading("total_bytes", Reading::Int(*total))
                    .reading("available_bytes", Reading::Int(*available))
                    .reading("read_only", Reading::Bool(*read_only))
                    .reading("writable", Reading::Bool(*writable))
                    .reading("tmpfs", Reading::Bool(*tmpfs))
                    .reading("recordings_dir", Reading::Bool(destination))
            };
            let elsewhere = "record somewhere else: `graph run --record=<DIR>` or \
                             `bag record -o <PATH>`";
            if *read_only {
                base(blocked, format!("{path} is on a read-only filesystem")).with_fix(elsewhere)
            } else if !*writable {
                base(blocked, format!("{path} is not writable by this user")).with_fix(format!(
                    "make it writable (`chmod u+w {path}`), or {elsewhere}"
                ))
            } else if *available == 0 {
                base(
                    blocked,
                    format!("{path} has no space available (of {total} bytes)"),
                )
                .with_fix(format!("free space on that filesystem, or {elsewhere}"))
            } else if *tmpfs {
                base(
                    Status::Warn,
                    format!(
                        "{path} is on tmpfs ({available} bytes available of {total}): a bag \
                         there uses memory and is lost at reboot"
                    ),
                )
                .with_fix(format!("to keep recordings, {elsewhere} on a disk"))
            } else {
                base(
                    Status::Pass,
                    format!("{path} has {available} bytes available of {total}"),
                )
            }
        }
    }
}

/// The directory to read and whether it is a recordings directory: the first
/// of `cwd/recordings` and `workspace_root/recordings` that exists, else
/// `cwd` itself.
pub(super) fn recording_dir(cwd: &Path, workspace_root: Option<&Path>) -> (PathBuf, bool) {
    std::iter::once(cwd)
        .chain(workspace_root)
        .map(|base| base.join("recordings"))
        .find(|dir| dir.is_dir())
        .map_or_else(|| (cwd.to_path_buf(), false), |dir| (dir, true))
}

#[cfg(unix)]
fn read(path: &Path) -> DiskReading {
    let shown = path.display().to_string();
    let stat = match rustix::fs::statvfs(path) {
        Ok(stat) => stat,
        Err(err) => {
            return DiskReading::Unreadable {
                path: shown,
                error: err.to_string(),
            }
        }
    };
    let writable = rustix::fs::access(path, rustix::fs::Access::WRITE_OK).is_ok();
    DiskReading::Read {
        path: shown,
        total: stat.f_blocks.saturating_mul(stat.f_frsize),
        available: stat.f_bavail.saturating_mul(stat.f_frsize),
        read_only: stat.f_flag.contains(rustix::fs::StatVfsMountFlags::RDONLY),
        writable,
        tmpfs: is_tmpfs(path),
    }
}

/// `statfs` reports the filesystem's magic number on Linux; `TMPFS_MAGIC` is
/// 0x01021994 (`linux/magic.h`).
#[cfg(target_os = "linux")]
fn is_tmpfs(path: &Path) -> bool {
    const TMPFS_MAGIC: i64 = 0x0102_1994;
    // `f_type` is a `c_long`: the conversion is the identity on 64-bit Linux
    // and widens on 32-bit Linux.
    #[allow(clippy::useless_conversion)]
    let is_tmpfs = rustix::fs::statfs(path).is_ok_and(|s| i64::from(s.f_type) == TMPFS_MAGIC);
    is_tmpfs
}

#[cfg(all(unix, not(target_os = "linux")))]
fn is_tmpfs(_path: &Path) -> bool {
    false
}

#[cfg(unix)]
fn run(ctx: &Context) -> Outcome {
    let (dir, destination) = recording_dir(&ctx.cwd, ctx.workspace_root.as_deref());
    decide(&read(&dir), destination)
}

#[cfg(not(unix))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("recording is supported on Unix only")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_ok(available: u64) -> DiskReading {
        DiskReading::Read {
            path: "/w/recordings".to_string(),
            total: 1000,
            available,
            read_only: false,
            writable: true,
            tmpfs: false,
        }
    }

    #[test]
    fn passes_with_space_on_a_writable_disk() {
        let o = decide(&read_ok(10), true);
        assert_eq!(o.status, Status::Pass);
        assert_eq!(o.summary, "/w/recordings has 10 bytes available of 1000");
        assert_eq!(o.fix, None);
    }

    #[test]
    fn fails_with_no_space() {
        let o = decide(&read_ok(0), true);
        assert_eq!(o.status, Status::Fail);
        assert!(o.fix.unwrap().contains("--record=<DIR>"));
    }

    #[test]
    fn fails_read_only_before_anything_else() {
        let mut r = read_ok(10);
        if let DiskReading::Read {
            read_only,
            writable,
            ..
        } = &mut r
        {
            *read_only = true;
            *writable = false;
        }
        let o = decide(&r, true);
        assert_eq!(o.status, Status::Fail);
        assert_eq!(o.summary, "/w/recordings is on a read-only filesystem");
    }

    #[test]
    fn fails_when_not_writable() {
        let mut r = read_ok(10);
        if let DiskReading::Read { writable, .. } = &mut r {
            *writable = false;
        }
        let o = decide(&r, true);
        assert_eq!(o.status, Status::Fail);
        assert!(o.fix.unwrap().contains("chmod u+w /w/recordings"));
    }

    #[test]
    fn warns_on_tmpfs() {
        let mut r = read_ok(10);
        if let DiskReading::Read { tmpfs, .. } = &mut r {
            *tmpfs = true;
        }
        let o = decide(&r, true);
        assert_eq!(o.status, Status::Warn);
        assert!(o.summary.contains("lost at reboot"));
    }

    #[test]
    fn unreadable_warns() {
        let o = decide(
            &DiskReading::Unreadable {
                path: "/w".to_string(),
                error: "EACCES".to_string(),
            },
            true,
        );
        assert_eq!(o.status, Status::Warn);
    }

    #[test]
    fn a_blocked_current_directory_warns_where_a_recordings_directory_fails() {
        for (read_only, writable, available) in
            [(true, false, 10), (false, false, 10), (false, true, 0)]
        {
            let mut r = read_ok(available);
            if let DiskReading::Read {
                read_only: ro,
                writable: w,
                ..
            } = &mut r
            {
                *ro = read_only;
                *w = writable;
            }
            assert_eq!(decide(&r, true).status, Status::Fail, "{r:?}");
            let here = decide(&r, false);
            assert_eq!(here.status, Status::Warn, "{r:?}");
            assert!(here.fix.unwrap().contains("--record=<DIR>"));
            assert_eq!(
                here.readings.get("recordings_dir"),
                Some(&Reading::Bool(false))
            );
        }
    }

    #[test]
    fn the_recording_dir_is_cwd_recordings_then_the_workspace_then_cwd() {
        let cwd = tempfile::tempdir().unwrap();
        let ws = tempfile::tempdir().unwrap();
        let (c, w) = (cwd.path(), ws.path());
        assert_eq!(recording_dir(c, Some(w)), (c.to_path_buf(), false));
        std::fs::create_dir(w.join("recordings")).unwrap();
        assert_eq!(recording_dir(c, Some(w)), (w.join("recordings"), true));
        assert_eq!(recording_dir(c, None), (c.to_path_buf(), false));
        std::fs::create_dir(c.join("recordings")).unwrap();
        assert_eq!(recording_dir(c, Some(w)), (c.join("recordings"), true));
    }
}
