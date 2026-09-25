// SPDX-License-Identifier: AGPL-3.0-only
//! Enabled/disabled resolution and the shared consent file.
//!
//! Precedence (highest first): `DO_NOT_TRACK=1` > `CERULION_TELEMETRY=0|1` >
//! `${CERULION_HOME:-~/.cerulion}/telemetry.json` > default (enabled).
//!
//! The file is shared with Cerulion Studio, which reads and writes the same
//! format from its own (non-linking) implementation:
//!
//! ```json
//! {"enabled": true, "anon_id": "anon:<uuid v4>", "notice_shown": false,
//!  "updated_at": "2026-09-09T21:00:00.000Z"}
//! ```
//!
//! [`status`] only READS (an opt-out via env var must never cause a write).
//! [`anon_id`], [`set_enabled`] and [`mark_notice_shown`] create the file on
//! first use, atomically (per-write unique temp, `create_new`, rename) with
//! mode 0600 inside a 0700 `${CERULION_HOME:-~/.cerulion}` (the directory is
//! shared with `cerulion_cli_engine`'s secrets, so it must never be created
//! world-listable). Every read-modify-write holds an advisory lock on the
//! sibling `telemetry.json.lock` (`std::fs::File::lock`), so concurrent
//! writers, threads or processes, including Studio, cannot revert each
//! other's change.

/// Which rule decided the outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// Feature `posthog` is not compiled in; always disabled.
    NotCompiled,
    /// `DO_NOT_TRACK=1` (<https://consoledonottrack.com>).
    DoNotTrack,
    /// `CERULION_TELEMETRY=0` or `=1`.
    EnvVar,
    /// `telemetry.json`'s `enabled` field.
    File,
    /// No rule matched: enabled.
    Default,
}

/// The resolved decision and the rule that produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    pub enabled: bool,
    pub source: Source,
}

/// Pure resolution over already-read inputs. The precedence table in one place;
/// [`status`] feeds it the real environment and file.
///
/// `DO_NOT_TRACK` counts as set for `1` or `true` (case-insensitive), the
/// convention says `1`, and treating `true` as a request to track would be the
/// wrong way to be strict. `CERULION_TELEMETRY` is honoured only for exactly
/// `0` or `1`; any other spelling falls through to the file.
pub fn resolve(
    do_not_track: Option<&str>,
    cerulion_telemetry: Option<&str>,
    file_enabled: Option<bool>,
) -> Status {
    if do_not_track
        .map(str::trim)
        .is_some_and(|v| v == "1" || v.eq_ignore_ascii_case("true"))
    {
        return Status {
            enabled: false,
            source: Source::DoNotTrack,
        };
    }
    match cerulion_telemetry.map(str::trim) {
        Some("0") => {
            return Status {
                enabled: false,
                source: Source::EnvVar,
            }
        }
        Some("1") => {
            return Status {
                enabled: true,
                source: Source::EnvVar,
            }
        }
        _ => {}
    }
    match file_enabled {
        Some(enabled) => Status {
            enabled,
            source: Source::File,
        },
        None => Status {
            enabled: true,
            source: Source::Default,
        },
    }
}

#[cfg(feature = "posthog")]
pub use enabled::{
    anon_id, claim_notice, file_path, mark_notice_shown, notice_shown, rotate_anon_id, set_enabled,
    status, TelemetryFile,
};

#[cfg(feature = "posthog")]
mod enabled {
    use super::{resolve, Status};
    use crate::rfc3339;
    use crate::Error;
    use serde::{Deserialize, Serialize};
    use std::fs;
    use std::io::Write;
    use std::path::{Path, PathBuf};
    use std::time::SystemTime;

    /// The on-disk record. Field names are the cross-surface contract. Only
    /// `enabled` is required to parse: a hand-edited `{"enabled": false}` or a
    /// file from another surface's older schema must still carry its opt-out;
    /// the other fields default and are filled in on the next write.
    #[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
    pub struct TelemetryFile {
        pub enabled: bool,
        #[serde(default)]
        pub anon_id: String,
        #[serde(default)]
        pub notice_shown: bool,
        #[serde(default)]
        pub updated_at: String,
    }

    impl TelemetryFile {
        fn fresh() -> Self {
            TelemetryFile {
                enabled: true,
                anon_id: format!("anon:{}", uuid::Uuid::new_v4()),
                notice_shown: false,
                updated_at: rfc3339::format(SystemTime::now()),
            }
        }
    }

    /// `${CERULION_HOME:-~/.cerulion}/telemetry.json`. Empty `CERULION_HOME`
    /// counts as unset, matching `cerulion_cli_engine::auth`.
    pub fn file_path() -> Result<PathBuf, Error> {
        let root = match std::env::var_os("CERULION_HOME") {
            Some(home) if !home.is_empty() => PathBuf::from(home),
            _ => dirs::home_dir().ok_or(Error::NoHome)?.join(".cerulion"),
        };
        Ok(root.join("telemetry.json"))
    }

    /// Resolve the decision from the real environment and file. Never writes.
    ///
    /// Fails closed: a file that exists but cannot be read (permissions, I/O)
    /// may hold an opt-out we cannot see, so it counts as `enabled: false`.
    /// An absent or corrupt file carries no decision and falls through.
    pub fn status() -> Status {
        let dnt = std::env::var("DO_NOT_TRACK").ok();
        let env = std::env::var("CERULION_TELEMETRY").ok();
        let file_enabled = match file_path().and_then(|p| read(&p)) {
            Ok(Some(file)) => Some(file.enabled),
            Ok(None) | Err(Error::Json(_)) => None,
            Err(Error::NoHome) => None,
            Err(Error::Io { .. }) => Some(false),
        };
        resolve(dnt.as_deref(), env.as_deref(), file_enabled)
    }

    /// The anonymous id, minting the file on first use. `Some` whenever the
    /// feature is compiled in; the `Option` exists so the feature-off signature
    /// is identical.
    pub fn anon_id() -> Result<Option<String>, Error> {
        let path = file_path()?;
        let _lock = Lock::acquire(&path)?;
        Ok(Some(load_or_create(&path)?.anon_id))
    }

    /// Replace the anonymous id with a fresh one and return it. Used when a
    /// different account signs in on the same machine, so one `anon:` id is
    /// never merged into two persons.
    pub fn rotate_anon_id() -> Result<Option<String>, Error> {
        let fresh = TelemetryFile::fresh().anon_id;
        let rotated = fresh.clone();
        update(move |f| f.anon_id = fresh)?;
        Ok(Some(rotated))
    }

    /// Persist an explicit opt-in/opt-out.
    pub fn set_enabled(enabled: bool) -> Result<(), Error> {
        update(|f| f.enabled = enabled)
    }

    /// Claim the first-run notice: under one lock, flip `notice_shown` from
    /// `false` to `true` and return whether THIS caller flipped it. Exactly one
    /// of any number of concurrent first runs gets `Ok(true)`. An already
    /// claimed notice is answered from a lock-free read: `notice_shown` only
    /// ever goes `false -> true`, so a stale `true` is still correct and the
    /// file is left untouched (no lock, fsync or `updated_at` bump).
    pub fn claim_notice() -> Result<bool, Error> {
        let path = file_path()?;
        if read(&path).ok().flatten().is_some_and(|f| f.notice_shown) {
            return Ok(false);
        }
        let mut claimed = false;
        update(|f| {
            claimed = !f.notice_shown;
            f.notice_shown = true;
        })?;
        Ok(claimed)
    }

    /// Record that the first-run notice has been printed.
    pub fn mark_notice_shown() -> Result<(), Error> {
        update(|f| f.notice_shown = true)
    }

    /// Whether the first-run notice has already been claimed. Read-only: an
    /// absent file is `Ok(false)`, a corrupt one an error.
    pub fn notice_shown() -> Result<bool, Error> {
        Ok(read(&file_path()?)?.is_some_and(|f| f.notice_shown))
    }

    fn update(apply: impl FnOnce(&mut TelemetryFile)) -> Result<(), Error> {
        let path = file_path()?;
        let _lock = Lock::acquire(&path)?;
        let mut file = load_or_create(&path)?;
        apply(&mut file);
        file.updated_at = rfc3339::format(SystemTime::now());
        write_atomic(&path, &file)
    }

    /// `Ok(None)` when the file is absent; a corrupt file is reported as an
    /// error by [`read`] and treated as absent by [`load_or_create`].
    fn read(path: &Path) -> Result<Option<TelemetryFile>, Error> {
        let text = match fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(source) => {
                return Err(Error::Io {
                    path: path.to_path_buf(),
                    source,
                })
            }
        };
        serde_json::from_str(&text)
            .map(Some)
            .map_err(|e| Error::Json(e.to_string()))
    }

    fn load_or_create(path: &Path) -> Result<TelemetryFile, Error> {
        match read(path) {
            Ok(Some(mut file)) => {
                // Empty, hand-edited or another schema's id: replace it, keeping
                // `enabled` and `notice_shown`.
                if !crate::guard::is_anon_id(&file.anon_id) {
                    file.anon_id = TelemetryFile::fresh().anon_id;
                    file.updated_at = rfc3339::format(SystemTime::now());
                    write_atomic(path, &file)?;
                }
                Ok(file)
            }
            Ok(None) | Err(Error::Json(_)) => {
                let file = TelemetryFile::fresh();
                write_atomic(path, &file)?;
                Ok(file)
            }
            Err(e) => Err(e),
        }
    }

    /// Exclusive advisory lock on `<path>.lock`, released on drop.
    struct Lock {
        _file: fs::File,
    }

    impl Lock {
        fn acquire(path: &Path) -> Result<Lock, Error> {
            let lock_path = path.with_extension("json.lock");
            let io = |source| Error::Io {
                path: lock_path.clone(),
                source,
            };
            create_dir_0700(path)?;
            let mut options = fs::OpenOptions::new();
            options.read(true).write(true).create(true).truncate(false);
            #[cfg(unix)]
            {
                use std::os::unix::fs::OpenOptionsExt;
                options.mode(0o600);
            }
            let file = options.open(&lock_path).map_err(io)?;
            file.lock().map_err(io)?;
            Ok(Lock { _file: file })
        }
    }

    fn create_dir_0700(path: &Path) -> Result<(), Error> {
        let dir = path.parent().ok_or(Error::NoHome)?;
        let mut builder = fs::DirBuilder::new();
        builder.recursive(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::DirBuilderExt;
            builder.mode(0o700);
        }
        builder.create(dir).map_err(|source| Error::Io {
            path: dir.to_path_buf(),
            source,
        })
    }

    fn write_atomic(path: &Path, file: &TelemetryFile) -> Result<(), Error> {
        let io = |source| Error::Io {
            path: path.to_path_buf(),
            source,
        };
        let dir = path.parent().ok_or(Error::NoHome)?;
        create_dir_0700(path)?;
        let tmp = dir.join(format!(
            ".telemetry.json.{}.{}.tmp",
            std::process::id(),
            uuid::Uuid::new_v4().as_simple()
        ));
        let mut options = fs::OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let mut out = options.open(&tmp).map_err(io)?;
        let body = serde_json::to_vec_pretty(file).map_err(|e| Error::Json(e.to_string()))?;
        let written = out
            .write_all(&body)
            .and_then(|()| out.write_all(b"\n"))
            .and_then(|()| out.sync_all());
        if let Err(source) = written {
            let _ = fs::remove_file(&tmp);
            return Err(io(source));
        }
        drop(out);
        fs::rename(&tmp, path).map_err(|source| {
            let _ = fs::remove_file(&tmp);
            io(source)
        })?;
        sync_dir(dir).map_err(io)
    }

    /// Make the rename itself durable, so a crash right after an opt-out
    /// cannot bring back the previous record. Windows has no directory
    /// handle to sync; its rename is already durable once it returns.
    #[cfg(unix)]
    fn sync_dir(dir: &Path) -> std::io::Result<()> {
        fs::File::open(dir)?.sync_all()
    }

    #[cfg(not(unix))]
    fn sync_dir(_dir: &Path) -> std::io::Result<()> {
        Ok(())
    }
}

#[cfg(not(feature = "posthog"))]
pub use disabled::{
    anon_id, claim_notice, mark_notice_shown, notice_shown, rotate_anon_id, set_enabled, status,
};

#[cfg(not(feature = "posthog"))]
mod disabled {
    use super::{Source, Status};
    use crate::Error;

    /// Feature off: always disabled, nothing is read.
    pub fn status() -> Status {
        Status {
            enabled: false,
            source: Source::NotCompiled,
        }
    }

    /// Feature off: there is no id and no file is created.
    pub fn anon_id() -> Result<Option<String>, Error> {
        Ok(None)
    }

    /// Feature off: there is no id to rotate.
    pub fn rotate_anon_id() -> Result<Option<String>, Error> {
        Ok(None)
    }

    /// Feature off: no-op.
    pub fn set_enabled(_enabled: bool) -> Result<(), Error> {
        Ok(())
    }

    /// Feature off: there is no notice to claim.
    pub fn claim_notice() -> Result<bool, Error> {
        Ok(false)
    }

    /// Feature off: no-op.
    pub fn mark_notice_shown() -> Result<(), Error> {
        Ok(())
    }

    /// Feature off: there is no notice.
    pub fn notice_shown() -> Result<bool, Error> {
        Ok(false)
    }
}
