// SPDX-License-Identifier: AGPL-3.0-only
//! The discovery-ladder PEER CACHE: WRITER half.
//!
//! The FORMAT (`~/.cerulion/peers.json`), the TTL, and the READER
//! (`load_peers` / `cache_rung`) live in [`cerulion_discovery::peer_cache`] and
//! are RE-EXPORTED here, so every call site in this crate keeps reading as one
//! `crate::peer_cache::…` module. This file adds only the WRITE half.
//!
//! # Why the write half stays here
//!
//! `cerulion-netd` folds TTL-fresh cached locators
//! into its own zenoh session at boot, which required the format + reader to
//! move into a crate BOTH the CLI and netd can depend on (`cerulion_cli_engine`
//! already depends on `cerulion_netd`, so the reverse edge would be cyclic).
//!
//! The writer deliberately did NOT move, and that is a capability boundary
//! rather than packaging convenience. A cache write must be backed by evidence
//! that a gather CONFIRMED a robot live — only a VERIFIED `(robot, locator)`
//! pair may insert or refresh (see [`upsert_confirmed`]). netd runs no gather
//! and holds no such evidence, so it must never write; leaving the writer here
//! means netd links a crate that has no [`save_peers`] to call.
//!
//! It also keeps the atomic-write primitive where it belongs: [`save_peers`]
//! goes through `auth::atomic_write_secret` (a `pub(crate)` helper — named here,
//! deliberately NOT intra-doc-linked, since a public doc cannot link a private
//! item and the docs gate runs with `-D warnings`), so `peers.json` gets the
//! same crash-durable 0600 write as the credential files rather than a second
//! hand-rolled copy of that logic in a discovery crate.
//!
//! # Durability
//!
//! Writes are ATOMIC (temp-file + rename) so a crash mid-write never corrupts
//! the cache. There is NO lock: two CLIs writing concurrently resolve
//! last-writer-wins, which is self-healing here — each writer's payload is a
//! full, valid snapshot, and the next discovery refreshes whatever was lost.
//!
//! # Size bound
//!
//! Independently of TTL, each save caps the file to [`MAX_CACHED_PEERS`] rows,
//! keeping the most-recently-seen. A churning DHCP lab that mints a fresh
//! `(robot, locator)` per lease cannot grow the file — nor the number of
//! candidate locators the next run's `cache_rung` folds into a session —
//! without bound.

pub use cerulion_discovery::peer_cache::{
    cache_rung, current_unix_secs, default_cache_path, load_peers, CachedPeer, PeerCacheFile,
    CACHE_RUNG_BUDGET, CACHE_VERSION, MAX_CACHED_PEERS, PEER_TTL_SECS,
};

use std::path::Path;

/// Keep only the [`MAX_CACHED_PEERS`] most-recently-seen peers. TTL alone lets a
/// churning DHCP lab mint a fresh `(robot, locator)` row per lease for a whole
/// [`PEER_TTL_SECS`] week; unbounded, that bloats the file AND — worse — the
/// number of candidate locators the NEXT reader folds into a session. The cap
/// itself lives with the FORMAT in `cerulion_discovery`, which applies it on read
/// too (a file we did not write reaches the reader unbounded). Under the cap
/// the list is returned untouched (on-disk order preserved); over it, the peers
/// are sorted newest-`last_seen`-first (stable → deterministic among ties) and
/// truncated so the survivors are the freshest ones. Pure — oracle-tested.
fn cap_to_most_recent(mut peers: Vec<CachedPeer>) -> Vec<CachedPeer> {
    if peers.len() <= MAX_CACHED_PEERS {
        return peers;
    }
    peers.sort_by_key(|p| std::cmp::Reverse(p.last_seen));
    peers.truncate(MAX_CACHED_PEERS);
    peers
}

/// Atomically persist `peers` to `path`. The list is first capped to
/// [`MAX_CACHED_PEERS`] via the private `cap_to_most_recent` (newest-seen kept) so a
/// churning environment cannot grow the file — or the next run's candidate
/// set — without bound. Creates parent directories, writes a
/// [`PeerCacheFile`] to a same-directory temp file, then renames it over the
/// target (last-writer-wins; NO lock — see the module docs). Returns `true` on
/// success; any failure emits a `warn!` and returns `false` (best-effort — a
/// failed save never propagates).
pub fn save_peers(path: &Path, peers: &[CachedPeer]) -> bool {
    let doc = PeerCacheFile {
        v: CACHE_VERSION,
        peers: cap_to_most_recent(peers.to_vec()),
    };
    let json = match serde_json::to_string_pretty(&doc) {
        Ok(j) => j,
        Err(e) => {
            tracing::warn!(error = %e, path = %path.display(), "failed to serialize peer cache");
            return false;
        }
    };
    // Crash-durable atomic write (parent 0700 → unique temp `create_new` 0600 →
    // fsync → rename), shared with the auth secrets via
    // [`crate::auth::atomic_write_secret`] — so a crash between write and rename
    // never corrupts `peers.json` (best-effort: any failure warns + returns
    // false, never propagates). `peers.json` landing at 0600 is fine — it is
    // per-user CLI state.
    if let Err(e) = crate::auth::atomic_write_secret(path, json.as_bytes()) {
        tracing::warn!(
            error = %e,
            path = %path.display(),
            "failed to persist the peer cache"
        );
        return false;
    }
    true
}

/// Merge the robots a gather CONFIRMED live into an existing cache
/// list, keyed by `(robot, locator)`. Each confirmed entry is
/// `(robot, Option<locator>)`:
///
/// - `Some(locator)` — an mDNS-enriched row (name AND locator both VERIFIED):
///   refresh an existing `(robot, locator)` entry's `last_seen`, else APPEND it.
/// - `None` — an announce-only row (the robot is live but no locator was
///   verified): INERT here (no touch-by-name; a
///   locator with no fresh evidence must age toward the TTL, or a
///   provably-dead address stays warm forever). NEVER invents a locator.
///   [`record_confirmed`] logs these robots instead.
///
/// Pure (no IO, no clock) so it is exhaustively oracle-tested; [`record_confirmed`]
/// delegates the merge here.
pub fn upsert_confirmed(
    mut peers: Vec<CachedPeer>,
    confirmed: &[(String, Option<String>)],
    now: u64,
) -> Vec<CachedPeer> {
    for (robot, locator) in confirmed {
        // Announce-only rows (no verified locator) are deliberately inert —
        // only a verified (robot, locator) pair may insert or refresh.
        let Some(loc) = locator else { continue };
        match peers
            .iter_mut()
            .find(|p| &p.robot == robot && &p.locator == loc)
        {
            Some(existing) => existing.last_seen = now,
            None => peers.push(CachedPeer {
                robot: robot.clone(),
                locator: loc.clone(),
                last_seen: now,
            }),
        }
    }
    peers
}

/// Post-gather cache write-back (best-effort) into the cache file at
/// `path` — refresh/insert the robots the gather CONFIRMED live. Each entry is
/// `(robot, Option<locator>)`: `Some` (an mDNS row) inserts/refreshes via
/// [`upsert_confirmed`]; `None` (announce-only — live but with NO verified
/// locator to cache) is logged with ONE `tracing::info!` naming the robot, so
/// the silent cost — an mDNS-unreachable network re-discovers that robot every
/// run — is visible, and is never written. Loads the current cache (evicting
/// stale entries), merges with a real `now`, and saves. Failures at any step
/// are warned inside the helpers, never propagated. Skips entirely on an empty
/// list.
///
/// `path` is a parameter so the write-back is
/// testable against a temp file — the CLI call site resolves
/// [`default_cache_path`]. Callers must ALSO gate on the scouting path
/// (`topic_cmd::resolve_write_back`) so hermetic tests never reach here with
/// the real path.
pub fn record_confirmed(path: &Path, confirmed: &[(String, Option<String>)]) {
    if confirmed.is_empty() {
        return;
    }
    for (robot, locator) in confirmed {
        if locator.is_none() {
            tracing::info!(
                robot = %robot,
                "robot confirmed live but not cacheable — no verified locator \
                 (mDNS unreachable?); mDNS-unreachable networks re-discover it each run"
            );
        }
    }
    let now = current_unix_secs();
    let existing = load_peers(path, now);
    let merged = upsert_confirmed(existing, confirmed, now);
    save_peers(path, &merged);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};

    static DIR_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// A unique temp directory removed on drop. No env-var games, no
    /// serial_test — every test owns an isolated directory.
    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn new() -> Self {
            let n = DIR_COUNTER.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!("peercache_{}_{}", std::process::id(), n));
            std::fs::create_dir_all(&path).unwrap();
            TempDir { path }
        }

        fn file(&self, name: &str) -> PathBuf {
            self.path.join(name)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.path);
        }
    }

    fn cp(robot: &str, locator: &str, last_seen: u64) -> CachedPeer {
        CachedPeer {
            robot: robot.to_string(),
            locator: locator.to_string(),
            last_seen,
        }
    }

    #[test]
    fn save_then_load_round_trips() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        let want = vec![
            cp("go2", "tcp/10.0.0.5:7683", 2_000),
            cp("rover", "tcp/10.0.0.6:7683", 2_100),
        ];
        assert!(save_peers(&path, &want));
        // Loaded within TTL → identical to the hand oracle.
        assert_eq!(load_peers(&path, 2_200), want);
    }

    /// What this crate WRITES is exactly what the shared reader (and
    /// therefore `cerulion-netd`'s boot fold) accepts — the cross-crate format
    /// contract, pinned on the BYTES rather than on a same-crate round trip.
    /// Without this, the writer here and the reader in `cerulion_discovery`
    /// could drift (a renamed field, a changed `v`) with every test in both
    /// crates still green.
    #[test]
    fn the_written_document_is_the_shape_the_shared_reader_declares() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        assert!(save_peers(&path, &[cp("go2", "tcp/10.0.0.5:7683", 2_000)]));
        let raw = std::fs::read_to_string(&path).unwrap();
        // Hand oracle over the on-disk shape (whitespace-insensitive): the
        // version gate the reader checks, and the three field names it binds.
        assert!(
            raw.contains(&format!("\"v\": {CACHE_VERSION}")),
            "the document must carry the version the shared reader gates on; got {raw}"
        );
        for key in ["\"robot\"", "\"locator\"", "\"last_seen\""] {
            assert!(
                raw.contains(key),
                "missing {key} in the written document: {raw}"
            );
        }
        // And the shared reader really binds them (not merely that the bytes
        // look right).
        assert_eq!(
            load_peers(&path, 2_100),
            vec![cp("go2", "tcp/10.0.0.5:7683", 2_000)]
        );
    }

    /// A confirmed row WITH a locator (mDNS-enriched) refreshes
    /// an existing `(robot, locator)` entry, or appends a new one — hand oracle.
    #[test]
    fn upsert_confirmed_some_locator_refreshes_or_appends() {
        let existing = vec![
            cp("go2", "tcp/10.0.0.5:7683", 1_000),
            cp("rover", "tcp/10.0.0.6:7683", 1_000),
        ];
        // go2@same locator → last_seen refreshed; drone@new → appended; rover
        // (not confirmed) untouched.
        let merged = upsert_confirmed(
            existing,
            &[
                ("go2".to_string(), Some("tcp/10.0.0.5:7683".to_string())),
                ("drone".to_string(), Some("tcp/10.0.0.7:7683".to_string())),
            ],
            9_999,
        );
        assert_eq!(
            merged,
            vec![
                cp("go2", "tcp/10.0.0.5:7683", 9_999),   // refreshed
                cp("rover", "tcp/10.0.0.6:7683", 1_000), // untouched
                cp("drone", "tcp/10.0.0.7:7683", 9_999), // appended
            ]
        );
    }

    /// A confirmed row for a KNOWN robot at a NEW locator is a
    /// new entry (the key is `(robot, locator)`).
    #[test]
    fn upsert_confirmed_same_robot_new_locator_is_a_new_entry() {
        let existing = vec![cp("go2", "tcp/10.0.0.5:7683", 1_000)];
        let merged = upsert_confirmed(
            existing,
            &[("go2".to_string(), Some("tcp/10.0.0.9:7683".to_string()))],
            2_000,
        );
        assert_eq!(
            merged,
            vec![
                cp("go2", "tcp/10.0.0.5:7683", 1_000),
                cp("go2", "tcp/10.0.0.9:7683", 2_000),
            ]
        );
    }

    /// An announce-only confirmed row
    /// (locator `None`) is INERT — it neither touches existing entries by name
    /// (a locator with no fresh evidence must age toward the TTL) nor invents
    /// a locator for a never-seen robot. Hand oracle.
    #[test]
    fn upsert_confirmed_none_locator_is_inert() {
        let existing = vec![
            cp("go2", "tcp/10.0.0.5:7683", 1_000),
            cp("go2", "tcp/10.0.0.9:7683", 1_000),
            cp("rover", "tcp/10.0.0.6:7683", 1_000),
        ];
        // Announce-only "go2" (no verified locator): NOTHING changes — the two
        // remembered go2 addresses keep their old last_seen and age normally.
        let merged = upsert_confirmed(existing.clone(), &[("go2".to_string(), None)], 5_000);
        assert_eq!(
            merged, existing,
            "an announce-only confirmation must not keep unverified locators warm"
        );
        // A never-seen announce-only robot invents nothing (no fabricated locator).
        assert_eq!(
            upsert_confirmed(Vec::new(), &[("ghost".to_string(), None)], 5_000),
            Vec::<CachedPeer>::new()
        );
    }

    /// `record_confirmed` (path-parameterized,
    /// so the REAL write-back fn is testable) mutates the
    /// file exactly: an mDNS tuple refreshes/appends, a `None`-locator tuple
    /// changes NOTHING on disk (it is only logged), and an empty list is a
    /// no-op. Hand-built tuples → expected `peers.json` mutation.
    #[test]
    fn record_confirmed_mutates_the_file_exactly() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        assert!(save_peers(&path, &[cp("go2", "tcp/10.0.0.5:7683", 1_000)]));

        // The REAL write-back: refresh go2 (mDNS), append new (mDNS), and a
        // None-locator row ("silent") that must not touch the file.
        record_confirmed(
            &path,
            &[
                ("go2".to_string(), Some("tcp/10.0.0.5:7683".to_string())),
                ("new".to_string(), Some("tcp/10.0.0.8:7683".to_string())),
                ("silent".to_string(), None),
            ],
        );
        let after = load_peers(&path, current_unix_secs());
        // `record_confirmed` stamps a REAL wall-clock `now`, so pin the
        // structure (who is present, who is absent) rather than exact stamps.
        assert_eq!(after.len(), 2, "exactly go2 (refreshed) + new (appended)");
        assert!(after
            .iter()
            .any(|p| p.robot == "go2" && p.locator == "tcp/10.0.0.5:7683" && p.last_seen > 1_000));
        assert!(after
            .iter()
            .any(|p| p.robot == "new" && p.locator == "tcp/10.0.0.8:7683"));
        assert!(
            !after.iter().any(|p| p.robot == "silent"),
            "a None-locator confirmation must never reach the file"
        );

        // Empty list: a pure no-op (file bytes untouched).
        let before_bytes = std::fs::read(&path).unwrap();
        record_confirmed(&path, &[]);
        assert_eq!(std::fs::read(&path).unwrap(), before_bytes);
    }

    #[test]
    fn save_is_atomic_no_temp_left_behind() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        // Save twice (the second overwrites an existing file).
        assert!(save_peers(&path, &[cp("go2", "tcp/10.0.0.5:7683", 1)]));
        assert!(save_peers(&path, &[cp("go2", "tcp/10.0.0.5:7683", 2)]));

        // The final content is the second write.
        assert_eq!(
            load_peers(&path, 100),
            vec![cp("go2", "tcp/10.0.0.5:7683", 2)]
        );

        // No staging temp survives in the directory (the atomic-write helper
        // stages `.<name>.<pid>.tmp` then renames it away).
        let leftovers: Vec<_> = std::fs::read_dir(&dir.path)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.ends_with(".tmp"))
            .collect();
        assert!(
            leftovers.is_empty(),
            "temp file(s) left behind: {leftovers:?}"
        );
    }

    #[test]
    fn cap_keeps_newest_32_by_last_seen() {
        // 33 peers, last_seen == index (all distinct). Capping keeps the newest
        // 32 (last_seen 33..=2, newest first) and evicts the single OLDEST
        // (last_seen 1).
        let input: Vec<CachedPeer> = (1u64..=33)
            .map(|i| cp(&format!("robot{i}"), &format!("tcp/10.0.0.{i}:7683"), i))
            .collect();
        let capped = cap_to_most_recent(input);

        // Hand oracle: exactly MAX_CACHED_PEERS survivors, newest-first.
        let expected: Vec<CachedPeer> = (2u64..=33)
            .rev()
            .map(|i| cp(&format!("robot{i}"), &format!("tcp/10.0.0.{i}:7683"), i))
            .collect();
        assert_eq!(capped.len(), MAX_CACHED_PEERS);
        assert_eq!(
            capped, expected,
            "cap keeps the newest 32 by last_seen, newest first"
        );
        assert!(
            !capped.iter().any(|p| p.robot == "robot1"),
            "the single oldest peer (last_seen 1) is the evictee"
        );
    }

    #[test]
    fn cap_under_limit_is_untouched() {
        // At/under the cap the list is returned verbatim (order + contents),
        // so an ordinary save never reorders the file.
        let peers = vec![
            cp("go2", "tcp/10.0.0.5:7683", 2_000),
            cp("rover", "tcp/10.0.0.6:7683", 2_100),
        ];
        assert_eq!(cap_to_most_recent(peers.clone()), peers);
    }
}
