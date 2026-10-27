// SPDX-License-Identifier: AGPL-3.0-only
//! `disk.recordings`: the filesystem a recording started here is written to.
//!
//! `graph run --record` writes under `recordings/` relative to the current
//! directory and `bag record` writes `recording.mcap` in the current
//! directory, so the check reads `./recordings` when it exists and the current
//! directory otherwise. It reads
//! `statvfs` (space and the read-only flag), `access(W_OK)` and, on Linux,
//! `statfs`'s filesystem type, where a tmpfs is memory that a reboot empties.

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

/// The decision over the reading.
pub(super) fn decide(r: &DiskReading) -> Outcome {
    match r {
        DiskReading::Unreadable { path, error } => {
            Outcome::new(Status::Warn, format!("could not read {path}: {error}"))
                .with_fix(format!("run `df -h {path}` to see the filesystem"))
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
            };
            let elsewhere = "record somewhere else: `graph run --record=<DIR>` or \
                             `bag record -o <PATH>`";
            if *read_only {
                base(Status::Fail, format!("{path} is on a read-only filesystem"))
                    .with_fix(elsewhere)
            } else if !*writable {
                base(Status::Fail, format!("{path} is not writable by this user")).with_fix(
                    format!("make it writable (`chmod u+w {path}`), or {elsewhere}"),
                )
            } else if *available == 0 {
                base(
                    Status::Fail,
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

/// The directory a recording started in `cwd` lands in: `cwd/recordings` when
/// it exists, else `cwd`.
pub(super) fn recording_dir(cwd: &Path) -> PathBuf {
    let recordings = cwd.join("recordings");
    if recordings.is_dir() {
        recordings
    } else {
        cwd.to_path_buf()
    }
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
    decide(&read(&recording_dir(&ctx.cwd)))
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
        let o = decide(&read_ok(10));
        assert_eq!(o.status, Status::Pass);
        assert_eq!(o.summary, "/w/recordings has 10 bytes available of 1000");
        assert_eq!(o.fix, None);
    }

    #[test]
    fn fails_with_no_space() {
        let o = decide(&read_ok(0));
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
        let o = decide(&r);
        assert_eq!(o.status, Status::Fail);
        assert_eq!(o.summary, "/w/recordings is on a read-only filesystem");
    }

    #[test]
    fn fails_when_not_writable() {
        let mut r = read_ok(10);
        if let DiskReading::Read { writable, .. } = &mut r {
            *writable = false;
        }
        let o = decide(&r);
        assert_eq!(o.status, Status::Fail);
        assert!(o.fix.unwrap().contains("chmod u+w /w/recordings"));
    }

    #[test]
    fn warns_on_tmpfs() {
        let mut r = read_ok(10);
        if let DiskReading::Read { tmpfs, .. } = &mut r {
            *tmpfs = true;
        }
        let o = decide(&r);
        assert_eq!(o.status, Status::Warn);
        assert!(o.summary.contains("lost at reboot"));
    }

    #[test]
    fn unreadable_warns() {
        let o = decide(&DiskReading::Unreadable {
            path: "/w".to_string(),
            error: "EACCES".to_string(),
        });
        assert_eq!(o.status, Status::Warn);
    }

    #[test]
    fn the_recording_dir_prefers_recordings_when_it_exists() {
        let dir = std::env::temp_dir().join(format!("doctor-disk-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        assert_eq!(recording_dir(&dir), dir);
        std::fs::create_dir(dir.join("recordings")).unwrap();
        assert_eq!(recording_dir(&dir), dir.join("recordings"));
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
