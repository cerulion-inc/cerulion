// SPDX-License-Identifier: AGPL-3.0-only
//! The discovery-ladder PEER CACHE: FORMAT + READER.
//!
//! Robots move around a lab, but a robot a machine talked to yesterday is
//! overwhelmingly likely to be reachable again today at the same address.
//! The cache remembers `(robot, locator)` pairs across invocations in a tiny
//! JSON file (`~/.cerulion/peers.json`) so a caller can try the last-known
//! gateways FIRST — before paying for hostname resolution or an mDNS scout.
//!
//! # Candidate-only (no verify session)
//!
//! A cached locator is a HINT, never a promise: the robot may be off, moved,
//! or re-addressed. [`cache_rung`] surfaces every TTL-fresh entry DIRECTLY as a
//! candidate — it opens NO zenoh session. The caller's single session connects
//! to every candidate's locator, and PRESENCE in the gather (the robot's live
//! announce tokens actually arriving, or an mDNS browse answer) is what
//! confirms it — so a dead cached robot simply produces no ROBOTS row, and the
//! bounded-connect query session never stalls on its black-hole locator.
//!
//! # Two consumers, one format
//!
//! - `cerulion` (`topic list`): runs the full ladder, and after a gather
//!   WRITES back the robots it confirmed live.
//! - `cerulion-netd`: folds TTL-fresh cached locators into its own zenoh
//!   session at boot, so first contact works with no `CERULION_NETD_CONNECT`.
//!
//! # Reader here, WRITER in `cerulion_cli_engine` — a capability boundary
//!
//! This module holds the format types, [`load_peers`], and [`cache_rung`]. The
//! WRITE half (`save_peers` / `upsert_confirmed` / `record_confirmed`, plus the
//! `MAX_CACHED_PEERS` save-time cap) lives in `cerulion_cli_engine::peer_cache`,
//! which re-exports everything here so its call sites read as one module.
//!
//! That split is deliberate, not packaging convenience. A cache write must be
//! backed by evidence that a gather CONFIRMED a robot live (only a VERIFIED
//! `(robot, locator)` pair may insert or refresh — an announce-only row has no
//! robot↔locator proof and is logged, never cached, so its old entries age
//! toward the TTL like any other). netd runs no gather and holds no such
//! evidence, so it must never write; keeping the writer out of this crate means
//! netd links a crate with no `save_peers` to call.
//!
//! # Durability (writer side)
//!
//! Writes are ATOMIC (temp-file + rename) so a crash mid-write never corrupts
//! the cache. There is NO lock: two writers resolve last-writer-wins, which is
//! self-healing here — each writer's payload is a full, valid snapshot, and the
//! next discovery refreshes whatever was lost. A reader therefore only ever
//! observes a complete document or the previous complete document; a corrupt or
//! version-mismatched one degrades to an empty cache with a loud warn, never an
//! error.
//!
//! # TTL
//!
//! Entries older than [`PEER_TTL_SECS`] are evicted on load, so a robot that
//! has been gone for a week stops costing connect attempts.
//!
//! # Out of scope
//!
//! Cryptographic pairing / trust of a discovered peer is a separate concern — explicitly
//! NOT handled here. This module records and re-verifies reachability only; it
//! makes no authenticity claim about who is answering at a locator.

use crate::ladder::{DiscoveredPeer, DiscoveryRung};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// On-disk schema version for [`PeerCacheFile`]. A file whose `v` differs is
/// treated as unreadable (evicted, fresh cache started) rather than migrated —
/// the cache is a rebuildable convenience, never a source of truth.
pub const CACHE_VERSION: u32 = 1;

/// Time-to-live for a cached peer entry: 7 days. A robot not seen within this
/// window is evicted on the next [`load_peers`] so the cache does not keep
/// re-trying long-gone addresses.
pub const PEER_TTL_SECS: u64 = 7 * 24 * 60 * 60;

/// Hard cap on the number of cached peers HONORED — applied on BOTH sides.
///
/// The writer (`cerulion_cli_engine::peer_cache::save_peers`) trims to this on
/// save, keeping the most-recently-seen rows. [`load_peers`] applies the SAME cap
/// on READ, which is not redundant: the writer only bounds files WE wrote, while a
/// hand-edited, restored, or foreign `peers.json` reaches the reader unbounded —
/// and every surviving row becomes a candidate that `cerulion-netd`'s boot fold
/// hands to `probe_reachable_locators`, i.e. one detached DNS+connect thread each.
/// Capping on read bounds that per boot. A well-formed cache is at or under the
/// cap, so this is a no-op for every file the writer produced.
pub const MAX_CACHED_PEERS: usize = 32;

/// Per-rung budget handed to the cached-peer rung (a fast local file read).
/// Accepted by [`cache_rung`] for rung-interface uniformity; the rung is a pure
/// local file read, so it always completes well inside any budget.
pub const CACHE_RUNG_BUDGET: Duration = Duration::from_millis(300);

/// The `~/.cerulion/peers.json` document. `v` gates the schema; `peers` is the
/// flat list of remembered gateways.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeerCacheFile {
    /// Schema version — must equal [`CACHE_VERSION`] to be honored.
    pub v: u32,
    /// The remembered peers.
    pub peers: Vec<CachedPeer>,
}

/// One remembered gateway: a `(robot, locator)` identity plus the unix-seconds
/// timestamp it was last confirmed at (drives TTL eviction).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CachedPeer {
    /// The robot's advertised name (e.g. `"go2"`).
    pub robot: String,
    /// The zenoh locator its gateway listens on (e.g. `"tcp/192.168.1.7:7683"`).
    pub locator: String,
    /// Unix seconds when this peer was last recorded / re-confirmed.
    pub last_seen: u64,
}

/// Default cache path: `~/.cerulion/peers.json`. `None` when there is no home
/// directory — the call sites that resolve it (the `topic_cmd` write-back, the
/// `discovery_ladder` cache-rung wiring, and `cerulion-netd`'s boot fold;
/// `record_confirmed` and [`cache_rung`] themselves take an explicit path) emit
/// the loud warn (this resolver stays quiet so it composes cleanly).
pub fn default_cache_path() -> Option<PathBuf> {
    Some(dirs::home_dir()?.join(".cerulion").join("peers.json"))
}

/// Keep an entry iff its age is strictly LESS than the TTL.
///
/// Boundary (pinned): with `age = now.saturating_sub(last_seen)`, an entry is
/// KEPT when `age < PEER_TTL_SECS` and EVICTED when `age >= PEER_TTL_SECS`.
/// So `now == last_seen + PEER_TTL_SECS` ⇒ evicted (age == TTL), and
/// `now == last_seen + PEER_TTL_SECS - 1` ⇒ kept (age == TTL - 1). A
/// future-stamped entry (`now < last_seen`, clock skew) saturates to age 0 and
/// is kept.
fn is_fresh(last_seen: u64, now_unix_secs: u64) -> bool {
    now_unix_secs.saturating_sub(last_seen) < PEER_TTL_SECS
}

/// Load cached peers, evicting any that have exceeded [`PEER_TTL_SECS`] as of
/// `now_unix_secs`. INFALLIBLE by contract:
///
/// - Missing file ⇒ empty (a debug breadcrumb; first run is normal).
/// - Unreadable / corrupt JSON / wrong `v` ⇒ empty + ONE loud `warn!` naming
///   the path and stating a fresh cache is being started.
///
/// Never panics, never returns an error — a broken cache degrades the ladder,
/// it does not break it.
pub fn load_peers(path: &Path, now_unix_secs: u64) -> Vec<CachedPeer> {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            tracing::debug!(path = %path.display(), "no peer cache yet (first run)");
            return Vec::new();
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "failed to read the peer cache; starting a fresh peer cache"
            );
            return Vec::new();
        }
    };
    let doc: PeerCacheFile = match serde_json::from_slice(&bytes) {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!(
                error = %e,
                path = %path.display(),
                "peer cache is corrupt or unreadable; starting a fresh peer cache"
            );
            return Vec::new();
        }
    };
    if doc.v != CACHE_VERSION {
        tracing::warn!(
            found = doc.v,
            expected = CACHE_VERSION,
            path = %path.display(),
            "peer cache schema version mismatch; starting a fresh peer cache"
        );
        return Vec::new();
    }
    let mut fresh: Vec<CachedPeer> = doc
        .peers
        .into_iter()
        .filter(|p| is_fresh(p.last_seen, now_unix_secs))
        .collect();
    if fresh.len() > MAX_CACHED_PEERS {
        // Only reachable via a file we did not write (the writer caps on save),
        // so it is worth a loud line rather than a silent trim.
        tracing::warn!(
            rows = fresh.len(),
            cap = MAX_CACHED_PEERS,
            path = %path.display(),
            "peer cache holds more rows than the cap (hand-edited or foreign file?); \
             keeping the most-recently-seen and ignoring the rest"
        );
        fresh.sort_by_key(|p| std::cmp::Reverse(p.last_seen));
        fresh.truncate(MAX_CACHED_PEERS);
    }
    fresh
}

/// Current wall-clock time in unix seconds; 0 if the clock is before the epoch
/// (pathological — keeps this infallible). `pub` so the writer half
/// (`cerulion_cli_engine::peer_cache`) stamps `last_seen` from the SAME clock
/// reading this module's TTL is measured against.
pub fn current_unix_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// The cache rung of the discovery ladder (candidate-only — NO zenoh
/// session, no verify probe). Loads every TTL-fresh entry from the cache file
/// at `path` and surfaces it DIRECTLY as a [`DiscoveryRung::Cache`] candidate;
/// the caller's single session connects to their locators and the gather
/// confirms which are actually live (a dead cached robot simply produces no
/// ROBOTS row, and the bounded-connect session never stalls on its black-hole
/// locator). INFALLIBLE — a missing/corrupt cache yields an empty `Vec`.
///
/// `path` is a parameter so the REAL rung is testable against a temp file — the
/// production wirings (`cerulion_cli_engine::discovery_ladder::discover_peers`
/// and `cerulion_netd::discovery_fold`) resolve [`default_cache_path`] and warn
/// there when no home directory exists. `_budget` is accepted for rung-interface
/// uniformity but unused: the rung is a pure local file read (no bounded I/O),
/// so it always completes well inside any ladder budget.
pub fn cache_rung(_budget: Duration, path: &Path) -> Vec<DiscoveredPeer> {
    load_peers(path, current_unix_secs())
        .into_iter()
        .map(|c| DiscoveredPeer {
            robot: c.robot,
            locator: c.locator,
            rung: DiscoveryRung::Cache,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};
    use tracing_test::traced_test;

    /// Does this captured line carry `level` as a WHOLE token in its header?
    ///
    /// A bare `line.contains("WARN")` would also match a field VALUE or a span
    /// name (`tracing-test` renders the test fn name into every line), so an
    /// "exactly N" oracle could be silently inverted by a rename. Whole-token
    /// matching is the discipline.
    fn line_is(line: &str, level: &str) -> bool {
        line.split_whitespace().any(|t| t == level)
    }

    static DIR_COUNTER: AtomicU64 = AtomicU64::new(0);

    /// A unique temp directory removed on drop. No env-var games, no
    /// serial_test — every test owns an isolated directory.
    struct TempDir {
        path: PathBuf,
    }

    impl TempDir {
        fn new() -> Self {
            let n = DIR_COUNTER.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!("peerread_{}_{}", std::process::id(), n));
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

    fn dp(robot: &str, locator: &str) -> DiscoveredPeer {
        DiscoveredPeer {
            robot: robot.to_string(),
            locator: locator.to_string(),
            rung: DiscoveryRung::Cache,
        }
    }

    /// Write a `peers.json` BY HAND (not through our own writer, which lives in
    /// another crate): these are reader tests, so the fixture must be an oracle
    /// over the on-disk FORMAT rather than a round-trip through the code that
    /// produced it.
    fn write_cache(path: &Path, peers: &[CachedPeer]) {
        let rows: Vec<String> = peers
            .iter()
            .map(|p| {
                format!(
                    r#"{{"robot":"{}","locator":"{}","last_seen":{}}}"#,
                    p.robot, p.locator, p.last_seen
                )
            })
            .collect();
        std::fs::write(
            path,
            format!(r#"{{"v":{},"peers":[{}]}}"#, CACHE_VERSION, rows.join(",")),
        )
        .unwrap();
    }

    /// An UNREADABLE cache degrades LOUDLY, at `warn!`.
    ///
    /// The level is the contract, not decoration: "your peers.json is corrupt"
    /// and "you have no cached robots" produce the SAME empty candidate list, so
    /// the log line is the only thing that separates them for an operator whose
    /// desk suddenly stopped finding a robot. A predicate matching only the
    /// MESSAGE would pass a demotion to `debug!`, which the shipped default
    /// filter (`info`) discards — so this matches the LEVEL TOKEN too.
    #[test]
    #[traced_test]
    fn an_unreadable_cache_degrades_at_warn_not_silently() {
        let dir = TempDir::new();

        let corrupt = dir.file("corrupt.json");
        std::fs::write(&corrupt, b"{ not json").unwrap();
        assert!(load_peers(&corrupt, 1_000).is_empty());

        let skewed = dir.file("skewed.json");
        std::fs::write(&skewed, br#"{"v":99,"peers":[]}"#).unwrap();
        assert!(load_peers(&skewed, 1_000).is_empty());

        logs_assert(|lines: &[&str]| {
            let warns = lines.iter().filter(|l| line_is(l, "WARN")).count();
            if warns != 2 {
                return Err(format!(
                    "expected 2 WARN degrade lines, got {warns}: {lines:?}"
                ));
            }
            for needle in ["corrupt or unreadable", "schema version mismatch"] {
                if !lines
                    .iter()
                    .any(|l| line_is(l, "WARN") && l.contains(needle))
                {
                    return Err(format!("no WARN line carrying {needle:?}: {lines:?}"));
                }
            }
            Ok(())
        });
    }

    /// ANTI-TAUTOLOGY for the arm above: a MISSING cache (an ordinary first run)
    /// is NOT a warn — otherwise "exactly 2 WARN" could be satisfied by a reader
    /// that warns about everything.
    #[test]
    #[traced_test]
    fn a_missing_cache_is_not_a_warning() {
        let dir = TempDir::new();
        assert!(load_peers(&dir.file("absent.json"), 1_000).is_empty());
        logs_assert(
            |lines: &[&str]| match lines.iter().filter(|l| line_is(l, "WARN")).count() {
                0 => Ok(()),
                n => Err(format!("a first run must not warn, got {n}: {lines:?}")),
            },
        );
    }

    /// The READ-side row cap. The writer trims on save,
    /// so this is only reachable via a file we did not write — but every surviving
    /// row becomes a candidate that netd's boot fold hands to a detached
    /// DNS+connect thread, so an unbounded foreign `peers.json` is an unbounded
    /// thread spawn per boot. Hand oracle: the survivors are the NEWEST rows.
    #[test]
    #[traced_test]
    fn an_over_cap_foreign_cache_is_trimmed_to_the_newest_rows_loudly() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        let over = (MAX_CACHED_PEERS + 8) as u64;
        // last_seen == i, all TTL-fresh relative to the `now` used below.
        let rows: Vec<CachedPeer> = (1..=over)
            .map(|i| cp(&format!("r{i}"), &format!("tcp/10.0.0.{i}:7683"), i))
            .collect();
        write_cache(&path, &rows);

        let loaded = load_peers(&path, PEER_TTL_SECS - 1);
        assert_eq!(
            loaded.len(),
            MAX_CACHED_PEERS,
            "the read side caps the row count"
        );
        // Hand oracle: newest-first, i.e. last_seen `over` down to `over - cap + 1`.
        let expected: Vec<CachedPeer> = ((over - MAX_CACHED_PEERS as u64 + 1)..=over)
            .rev()
            .map(|i| cp(&format!("r{i}"), &format!("tcp/10.0.0.{i}:7683"), i))
            .collect();
        assert_eq!(
            loaded, expected,
            "the survivors are the most-recently-seen rows"
        );

        // The FULL sentence, not a prefix: a botched line continuation inside a
        // multi-line Rust string literal leaves the run of source indentation
        // embedded in the message an operator reads, and a prefix-only predicate
        // cannot see it (the damage is always past the prefix).
        const CAP_WARN: &str = "peer cache holds more rows than the cap \
             (hand-edited or foreign file?); keeping the most-recently-seen and \
             ignoring the rest";
        logs_assert(|lines: &[&str]| {
            match lines
                .iter()
                .filter(|l| line_is(l, "WARN") && l.contains(CAP_WARN))
                .count()
            {
                1 => Ok(()),
                n => Err(format!(
                    "expected exactly 1 WARN carrying the full cap sentence, got {n}: {lines:?}"
                )),
            }
        });
    }

    /// The cap is a NO-OP for every file the writer produced: at/under the cap the
    /// on-disk order survives untouched and nothing is logged. Without this, the
    /// trim above could silently reorder every ordinary cache.
    #[test]
    #[traced_test]
    fn a_cache_at_the_cap_is_returned_verbatim_and_silently() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        // Deliberately NOT newest-first, so a stray sort would be visible.
        let cap = MAX_CACHED_PEERS as u64;
        let rows: Vec<CachedPeer> = (1..=cap)
            .map(|i| {
                cp(
                    &format!("r{i}"),
                    &format!("tcp/10.0.0.{i}:7683"),
                    cap - i + 1,
                )
            })
            .collect();
        write_cache(&path, &rows);

        assert_eq!(
            load_peers(&path, PEER_TTL_SECS - 1),
            rows,
            "at the cap the reader preserves on-disk order exactly"
        );
        logs_assert(
            |lines: &[&str]| match lines.iter().filter(|l| line_is(l, "WARN")).count() {
                0 => Ok(()),
                n => Err(format!(
                    "a within-cap cache must be silent, got {n} WARN: {lines:?}"
                )),
            },
        );
    }

    #[test]
    fn load_missing_file_is_empty() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        assert_eq!(load_peers(&path, 1_000), Vec::<CachedPeer>::new());
    }

    #[test]
    fn corrupt_json_is_empty() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        std::fs::write(&path, b"this is not json {{{").unwrap();
        assert_eq!(load_peers(&path, 1_000), Vec::<CachedPeer>::new());
    }

    #[test]
    fn wrong_version_is_empty() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        // Structurally valid, but v != CACHE_VERSION.
        std::fs::write(
            &path,
            br#"{"v":2,"peers":[{"robot":"go2","locator":"tcp/1.2.3.4:7683","last_seen":1000}]}"#,
        )
        .unwrap();
        assert_eq!(load_peers(&path, 1_000), Vec::<CachedPeer>::new());
    }

    /// The hand-written fixture really IS the shape `load_peers` accepts — the
    /// anti-tautology control for every other reader test here (without it, a
    /// `write_cache` that emitted garbage would make the "empty" assertions
    /// vacuously true).
    #[test]
    fn the_hand_written_fixture_round_trips_through_the_reader() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        let want = vec![
            cp("go2", "tcp/10.0.0.5:7683", 2_000),
            cp("rover", "tcp/10.0.0.6:7683", 2_100),
        ];
        write_cache(&path, &want);
        assert_eq!(load_peers(&path, 2_200), want);
    }

    #[test]
    fn is_fresh_boundary_is_exact() {
        // Age strictly < TTL is kept; age == TTL is evicted.
        assert!(is_fresh(1_000, 1_000 + PEER_TTL_SECS - 1));
        assert!(!is_fresh(1_000, 1_000 + PEER_TTL_SECS));
        // Future-stamped entry (clock skew) saturates to age 0 → kept.
        assert!(is_fresh(5_000, 1_000));
    }

    #[test]
    fn ttl_eviction_boundary_through_load() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        write_cache(&path, &[cp("go2", "tcp/10.0.0.5:7683", 1_000)]);

        // now == last_seen + TTL → evicted.
        assert_eq!(
            load_peers(&path, 1_000 + PEER_TTL_SECS),
            Vec::<CachedPeer>::new()
        );
        // now == last_seen + TTL - 1 → kept.
        assert_eq!(
            load_peers(&path, 1_000 + PEER_TTL_SECS - 1),
            vec![cp("go2", "tcp/10.0.0.5:7683", 1_000)]
        );
    }

    /// The real `cache_rung`: maps TTL-fresh
    /// entries to `DiscoveryRung::Cache` candidates with NO zenoh session (a pure
    /// file read: stale entries evicted, survivors tagged with the cache rung).
    /// Hand oracle.
    #[test]
    fn cache_rung_maps_ttl_fresh_entries_to_candidates_session_free() {
        let dir = TempDir::new();
        let path = dir.file("peers.json");
        // A fresh entry (stamped NOW, so it survives the rung's real-clock TTL
        // check) and an epoch-stale one (evicted).
        let now = current_unix_secs();
        write_cache(
            &path,
            &[
                cp("go2", "tcp/10.0.0.5:7683", now),
                cp("stale", "tcp/10.0.0.9:7683", 1),
            ],
        );
        assert_eq!(
            cache_rung(CACHE_RUNG_BUDGET, &path),
            vec![dp("go2", "tcp/10.0.0.5:7683")],
            "only the TTL-fresh entry survives, mapped to a Cache candidate"
        );
        // A missing file is an empty candidate set, never an error.
        assert!(cache_rung(CACHE_RUNG_BUDGET, &dir.file("absent.json")).is_empty());
    }
}
