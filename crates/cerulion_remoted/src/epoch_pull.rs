// SPDX-License-Identifier: AGPL-3.0-only
//! Robot-side revocation delivery.
//!
//! On a registered robot the network daemon's supervisor runs a sync worker
//! that fetches this robot's signed access-list epoch from the account service
//! (with the robot's own login) and writes it to [`EPOCH_PENDING_FILE`] in the
//! remoted state directory, as `hex(postcard(EpochSyncWire))`, the encoding a
//! desk's push carries. [`EpochPull`] picks it up every
//! [`EPOCH_POLL_INTERVAL`] and applies it through [`SharedTrust::apply_epoch`]:
//! the intermediate is re-verified against this robot's root set, the epoch's
//! signature, issuer and robot id are checked, and only a strictly newer epoch
//! changes anything. A missing, unreadable or rejected file changes nothing.
//!
//! When a key connects that has not been seen for the recheck interval
//! ([`KEY_RECHECK_ENV`], default [`KEY_RECHECK_INTERVAL`]),
//! [`EpochPull::note_accept`] writes [`EPOCH_REQUEST_FILE`], which the
//! supervisor reads as a request to fetch now rather than at the next period.

use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_pairing::verify::{EpochOutcome, EpochSyncWire};

use crate::clock::RemotedClock;
use crate::trust::SharedTrust;

/// The fetched epoch the sync worker leaves for remoted to apply.
pub const EPOCH_PENDING_FILE: &str = "epoch.pending";
/// The name a fetched epoch is moved to while it is applied.
const EPOCH_CLAIMED_FILE: &str = "epoch.pending.applying";
/// The file remoted writes to ask the supervisor for an early fetch.
pub const EPOCH_REQUEST_FILE: &str = "epoch.request";
/// How often remoted looks for a fetched epoch: the longest a fetched
/// revocation waits before it is enforced; a stat of one file per poll.
pub const EPOCH_POLL_INTERVAL: Duration = Duration::from_secs(2);
/// The environment variable setting how long a connecting key that was seen
/// before goes without requesting another early fetch, in whole seconds
/// from 5 to 3600.
pub const KEY_RECHECK_ENV: &str = "CERULION_REMOTED_EPOCH_RECHECK_S";
/// The default recheck interval. A revoked key reconnecting after this long away
/// is cut within seconds; one reconnecting sooner waits for the next periodic
/// fetch. Each recheck costs at most one early fetch per key per interval.
pub const KEY_RECHECK_INTERVAL: Duration = Duration::from_secs(60);
/// The shortest accepted recheck interval.
const MIN_KEY_RECHECK_INTERVAL: Duration = Duration::from_secs(5);
/// The longest accepted recheck interval (one hour).
const MAX_KEY_RECHECK_INTERVAL: Duration = Duration::from_secs(3_600);

/// The recheck interval from a [`KEY_RECHECK_ENV`] value: whole seconds within
/// the accepted range, else [`KEY_RECHECK_INTERVAL`] with a warning.
pub fn resolve_key_recheck(env_value: Option<&str>) -> Duration {
    let Some(value) = env_value.map(str::trim).filter(|v| !v.is_empty()) else {
        return KEY_RECHECK_INTERVAL;
    };
    match value.parse::<u64>().map(Duration::from_secs) {
        Ok(interval)
            if (MIN_KEY_RECHECK_INTERVAL..=MAX_KEY_RECHECK_INTERVAL).contains(&interval) =>
        {
            interval
        }
        _ => {
            tracing::warn!(
                "{KEY_RECHECK_ENV}={value:?} is not whole seconds from {} to {}; using the \
                 default {} s",
                MIN_KEY_RECHECK_INTERVAL.as_secs(),
                MAX_KEY_RECHECK_INTERVAL.as_secs(),
                KEY_RECHECK_INTERVAL.as_secs()
            );
            KEY_RECHECK_INTERVAL
        }
    }
}

/// The recheck interval this process's environment sets ([`KEY_RECHECK_ENV`]).
pub fn key_recheck_from_env() -> Duration {
    resolve_key_recheck(std::env::var(KEY_RECHECK_ENV).ok().as_deref())
}
/// The largest pending artifact read (far above a real epoch); a larger file is
/// discarded unread.
const MAX_PENDING_BYTES: u64 = 1024 * 1024;
/// Keys remembered for the recheck interval, bounding the map's memory; past
/// this the expired entries, then all, are dropped.
const MAX_REMEMBERED_KEYS: usize = 4096;

/// What one look at [`EPOCH_PENDING_FILE`] did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PullOutcome {
    /// No pending file.
    Nothing,
    /// A newer epoch was applied.
    Applied {
        /// The applied epoch number.
        epoch: u64,
    },
    /// The epoch was not newer than the robot's; nothing changed.
    NotNewer,
    /// The file could not be read, decoded or verified; nothing changed.
    Rejected {
        /// Why.
        reason: String,
    },
}

/// Applies fetched epochs and asks for early fetches (see the module docs).
pub struct EpochPull {
    dir: PathBuf,
    shared: SharedTrust,
    clock: RemotedClock,
    recheck: Duration,
    seen: Mutex<HashMap<[u8; 32], Instant>>,
}

impl EpochPull {
    /// A puller over the remoted state directory `dir`.
    pub fn new(dir: impl Into<PathBuf>, shared: SharedTrust, clock: RemotedClock) -> Self {
        Self {
            dir: dir.into(),
            shared,
            clock,
            recheck: KEY_RECHECK_INTERVAL,
            seen: Mutex::new(HashMap::new()),
        }
    }

    /// Use `recheck` in place of [`KEY_RECHECK_INTERVAL`].
    #[must_use]
    pub fn with_recheck(mut self, recheck: Duration) -> Self {
        self.recheck = recheck;
        self
    }

    /// Apply [`EPOCH_PENDING_FILE`] if present. The file is first claimed by
    /// renaming it aside, so an epoch the worker writes while this one is
    /// being applied is left for the next poll; the claimed file is removed.
    pub fn apply_pending(&self) -> PullOutcome {
        let pending = self.dir.join(EPOCH_PENDING_FILE);
        let claimed = self.dir.join(EPOCH_CLAIMED_FILE);
        match std::fs::rename(&pending, &claimed) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return PullOutcome::Nothing,
            Err(e) => {
                let reason = format!("could not claim the fetched epoch: {e}");
                tracing::warn!(%reason, "cerulion_remoted: a fetched revocation epoch waits");
                return PullOutcome::Rejected { reason };
            }
        }
        let outcome = match read_pending(&claimed) {
            Ok(None) => PullOutcome::Nothing,
            Ok(Some(bytes)) => self.apply(&bytes),
            Err(reason) => PullOutcome::Rejected { reason },
        };
        remove_quietly(&claimed);
        match &outcome {
            PullOutcome::Applied { epoch } => {
                tracing::info!(
                    epoch,
                    "cerulion_remoted: applied the fetched revocation epoch"
                )
            }
            PullOutcome::Rejected { reason } => {
                tracing::warn!(%reason, "cerulion_remoted: refused a fetched revocation epoch")
            }
            PullOutcome::NotNewer | PullOutcome::Nothing => {}
        }
        outcome
    }

    fn apply(&self, bytes: &[u8]) -> PullOutcome {
        let text = String::from_utf8_lossy(bytes);
        let Ok(decoded) = hex::decode(text.trim()) else {
            return PullOutcome::Rejected {
                reason: "the fetched epoch is not hex".into(),
            };
        };
        let wire = match EpochSyncWire::from_postcard(&decoded) {
            Ok(wire) => wire,
            Err(e) => {
                return PullOutcome::Rejected {
                    reason: format!("the fetched epoch did not decode: {e}"),
                }
            }
        };
        let epoch = wire.signed_epoch.epoch_data.epoch;
        match self
            .shared
            .apply_epoch(&wire.signed_epoch, &wire.intermediate, self.clock.now_ns())
        {
            Ok(EpochOutcome::Applied { .. }) => PullOutcome::Applied { epoch },
            Ok(EpochOutcome::NotNewer { .. }) => PullOutcome::NotNewer,
            // A persist failure leaves the live store enforcing the epoch.
            Err(crate::trust::TrustError::Persist(reason)) => {
                tracing::error!(%reason, "cerulion_remoted: a fetched epoch is enforced but was not saved");
                PullOutcome::Applied { epoch }
            }
            Err(e) => PullOutcome::Rejected {
                reason: format!("the fetched epoch was refused: {e}"),
            },
        }
    }

    /// Note an accepted connection from `key`; write [`EPOCH_REQUEST_FILE`]
    /// when the key is one this robot admits (the only kind a revocation can
    /// cut) and it has not been seen within the recheck interval. Returns
    /// whether a request was written.
    pub fn note_accept(&self, key: &[u8; 32]) -> bool {
        self.note_accept_at(key, Instant::now())
    }

    /// [`EpochPull::note_accept`] at `now`.
    pub fn note_accept_at(&self, key: &[u8; 32], now: Instant) -> bool {
        if !matches!(
            self.shared.snapshot_for_key(key).1,
            crate::trust::KeyAccess::Allowed(_)
        ) {
            return false;
        }
        let mut seen = self.seen.lock().unwrap_or_else(|p| p.into_inner());
        if seen
            .get(key)
            .is_some_and(|last| now.duration_since(*last) < self.recheck)
        {
            return false;
        }
        if seen.len() >= MAX_REMEMBERED_KEYS {
            seen.retain(|_, last| now.duration_since(*last) < self.recheck);
            if seen.len() >= MAX_REMEMBERED_KEYS {
                seen.clear();
            }
        }
        seen.insert(*key, now);
        drop(seen);
        match std::fs::write(self.dir.join(EPOCH_REQUEST_FILE), b"") {
            Ok(()) => true,
            Err(e) => {
                tracing::debug!(error = %e, "cerulion_remoted: could not request an early epoch fetch");
                false
            }
        }
    }

    /// Apply pending epochs every [`EPOCH_POLL_INTERVAL`] until the task is dropped.
    pub async fn run(self: Arc<Self>) {
        let mut tick = tokio::time::interval(EPOCH_POLL_INTERVAL);
        loop {
            tick.tick().await;
            let pull = Arc::clone(&self);
            let _ = tokio::task::spawn_blocking(move || pull.apply_pending()).await;
        }
    }
}

/// Read `path` when it is a regular, single-link file of at most
/// [`MAX_PENDING_BYTES`], owned by the owner of its directory, with no group
/// or other permission bits, in a directory with none either. The file is
/// opened without following a symbolic link and checked through the open
/// handle, so a file swapped in after the check is the one read and refused.
fn read_pending(path: &Path) -> Result<Option<Vec<u8>>, String> {
    use std::os::unix::fs::{MetadataExt, OpenOptionsExt};
    let file = match std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)
    {
        Ok(file) => file,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("could not open the fetched epoch: {e}")),
    };
    let dir = path.parent().unwrap_or(Path::new("."));
    let dir_meta = std::fs::symlink_metadata(dir)
        .map_err(|e| format!("could not inspect the state directory: {e}"))?;
    if !dir_meta.is_dir() || dir_meta.mode() & 0o077 != 0 {
        return Err("the state directory is not owner-only".into());
    }
    let meta = file
        .metadata()
        .map_err(|e| format!("could not inspect the fetched epoch: {e}"))?;
    if !meta.is_file() || meta.nlink() != 1 {
        return Err("the fetched epoch is not a regular single-link file".into());
    }
    if meta.uid() != dir_meta.uid() || meta.mode() & 0o077 != 0 {
        return Err("the fetched epoch is not owner-only and owned by this robot's user".into());
    }
    if meta.len() > MAX_PENDING_BYTES {
        return Err(format!(
            "the fetched epoch is {} bytes, above the {MAX_PENDING_BYTES}-byte bound",
            meta.len()
        ));
    }
    let mut bytes = Vec::new();
    file.take(MAX_PENDING_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| format!("could not read the fetched epoch: {e}"))?;
    Ok(Some(bytes))
}

fn remove_quietly(path: &Path) {
    if let Err(e) = std::fs::remove_file(path) {
        if e.kind() != std::io::ErrorKind::NotFound {
            tracing::warn!(error = %e, path = %path.display(), "cerulion_remoted: could not remove a fetched epoch");
        }
    }
}
