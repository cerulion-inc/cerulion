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
//! When a key connects that has not been seen for [`KEY_RECHECK_INTERVAL`],
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
/// The file remoted writes to ask the supervisor for an early fetch.
pub const EPOCH_REQUEST_FILE: &str = "epoch.request";
/// How often remoted looks for a fetched epoch: the longest a fetched
/// revocation waits before it is enforced.
pub const EPOCH_POLL_INTERVAL: Duration = Duration::from_secs(2);
/// A key seen more recently than this does not request another early fetch.
pub const KEY_RECHECK_INTERVAL: Duration = Duration::from_secs(60);
/// The largest pending artifact read; a larger file is discarded unread.
const MAX_PENDING_BYTES: u64 = 1024 * 1024;
/// Keys remembered for the recheck interval; past this the oldest are dropped.
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
    seen: Mutex<HashMap<[u8; 32], Instant>>,
}

impl EpochPull {
    /// A puller over the remoted state directory `dir`.
    pub fn new(dir: impl Into<PathBuf>, shared: SharedTrust, clock: RemotedClock) -> Self {
        Self {
            dir: dir.into(),
            shared,
            clock,
            seen: Mutex::new(HashMap::new()),
        }
    }

    /// Apply [`EPOCH_PENDING_FILE`] if present, then remove it.
    pub fn apply_pending(&self) -> PullOutcome {
        let path = self.dir.join(EPOCH_PENDING_FILE);
        let bytes = match read_pending(&path) {
            Ok(None) => return PullOutcome::Nothing,
            Ok(Some(bytes)) => bytes,
            Err(reason) => {
                remove_quietly(&path);
                tracing::warn!(%reason, "cerulion_remoted: discarded a fetched revocation epoch");
                return PullOutcome::Rejected { reason };
            }
        };
        remove_quietly(&path);
        let outcome = self.apply(&bytes);
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
    /// cut) and it has not been seen within [`KEY_RECHECK_INTERVAL`]. Returns
    /// whether a request was written.
    pub fn note_accept(&self, key: &[u8; 32]) -> bool {
        if !matches!(
            self.shared.snapshot_for_key(key).1,
            crate::trust::KeyAccess::Allowed(_)
        ) {
            return false;
        }
        let now = Instant::now();
        let mut seen = self.seen.lock().unwrap_or_else(|p| p.into_inner());
        if seen
            .get(key)
            .is_some_and(|last| now.duration_since(*last) < KEY_RECHECK_INTERVAL)
        {
            return false;
        }
        if seen.len() >= MAX_REMEMBERED_KEYS {
            seen.retain(|_, last| now.duration_since(*last) < KEY_RECHECK_INTERVAL);
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

/// Read `path` if it is a regular file of at most [`MAX_PENDING_BYTES`].
fn read_pending(path: &Path) -> Result<Option<Vec<u8>>, String> {
    let meta = match std::fs::symlink_metadata(path) {
        Ok(meta) => meta,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("could not inspect the fetched epoch: {e}")),
    };
    if !meta.file_type().is_file() {
        return Err("the fetched epoch is not a regular file".into());
    }
    if meta.len() > MAX_PENDING_BYTES {
        return Err(format!(
            "the fetched epoch is {} bytes, above the {MAX_PENDING_BYTES}-byte bound",
            meta.len()
        ));
    }
    let mut bytes = Vec::new();
    std::fs::File::open(path)
        .and_then(|f| f.take(MAX_PENDING_BYTES + 1).read_to_end(&mut bytes))
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
