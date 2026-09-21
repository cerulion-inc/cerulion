// SPDX-License-Identifier: AGPL-3.0-only
//! The PRODUCTION peer-cache fold end to end — a REAL
//! `~/.cerulion/peers.json` document, the REAL TTL reader, the REAL bounded TCP
//! pre-filter, and the REAL `NetworkConfig` mutation `main` performs before
//! `TransportManager::init`.
//!
//! # Hermetic, and how
//!
//! Every test owns a temp directory and writes the cache document BY HAND (an
//! oracle over the on-disk format, never a round trip through a writer this
//! crate cannot even call — see `cerulion_discovery::peer_cache`). "Reachable"
//! is a REAL `TcpListener` bound on `127.0.0.1:0`; "dead" is a port bound then
//! DROPPED, so the loopback stack refuses immediately. No zenoh, no iceoryx2, no
//! process spawn, no env mutation, no traffic that leaves the machine ⇒
//! parallel-safe, no `#[serial]`.
//!
//! # Scope of the pre-filter arms
//!
//! A SYN black hole (a packet-dropping address, the failure that motivated the
//! pre-filter) cannot be produced hermetically, so what these tests pin
//! is that an unreachable candidate is DROPPED and that the whole fold stays
//! bounded — not the timeout duration itself, which is
//! `cerulion_discovery::ladder::LADDER_PROBE_TIMEOUT` and is pinned as a
//! constant in that crate. The three shapes driven here (refused port,
//! unresolvable host, malformed locator) are the ones a stale cache actually
//! produces.

use cerulion_discovery::ladder::DiscoveryRung;
use cerulion_netd::discovery_fold::{fold_cached_peers_at, fold_from_cache_at, PeerFold};
use cerulion_netd::net::desk_network_config;
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tracing_test::traced_test;

static DIR_COUNTER: AtomicU64 = AtomicU64::new(0);

/// A unique temp directory removed on drop.
struct TempDir {
    path: PathBuf,
}

impl TempDir {
    fn new() -> Self {
        let n = DIR_COUNTER.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("fold_{}_{}", std::process::id(), n));
        std::fs::create_dir_all(&path).unwrap();
        TempDir { path }
    }
    fn cache(&self) -> PathBuf {
        self.path.join("peers.json")
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.path);
    }
}

fn now_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// Write a `peers.json` BY HAND at the schema version the reader gates on. Rows
/// are `(robot, locator, last_seen)`.
fn write_cache(path: &Path, rows: &[(&str, String, u64)]) {
    let body: Vec<String> = rows
        .iter()
        .map(|(robot, locator, last_seen)| {
            format!(r#"{{"robot":"{robot}","locator":"{locator}","last_seen":{last_seen}}}"#)
        })
        .collect();
    std::fs::write(path, format!(r#"{{"v":1,"peers":[{}]}}"#, body.join(","))).unwrap();
}

/// A REAL listening socket on loopback — the "robot is up" fixture. The returned
/// listener must be held for the lifetime of the probe.
fn live_locator() -> (TcpListener, String) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind a loopback listener");
    let port = listener.local_addr().unwrap().port();
    (listener, format!("tcp/127.0.0.1:{port}"))
}

/// A locator whose port was bound and then RELEASED — loopback refuses a connect
/// to it immediately, which is what a stale cache row looks like when the robot
/// is off.
fn dead_locator() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind a loopback listener");
    let port = listener.local_addr().unwrap().port();
    drop(listener);
    format!("tcp/127.0.0.1:{port}")
}

fn strs(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

/// THE ACCEPTANCE CRITERION: a brand-new desk whose only knowledge of the robot
/// is one `topic list`-written cache row dials that robot's locator with NO
/// `CERULION_NETD_CONNECT` set — through the REAL production entry point, so the
/// resulting `NetworkConfig` is exactly what `TransportManager::init` receives.
#[test]
fn a_cached_robot_is_dialled_with_no_env_set() {
    let dir = TempDir::new();
    let (_listener, locator) = live_locator();
    write_cache(&dir.cache(), &[("go2", locator.clone(), now_secs())]);

    // The pre-fold config is exactly what `network_config_from_env` produces with
    // no CONNECT env: scouting on, NO connect endpoints.
    let before = desk_network_config(Vec::new(), Vec::new());
    assert!(
        before.connect_endpoints.is_empty(),
        "precondition: with no env set the daemon starts with nothing to dial"
    );

    let (after, fold) = fold_cached_peers_at(before, &dir.cache());
    assert_eq!(
        after.connect_endpoints,
        vec![locator.clone()],
        "the cached robot must be dialled with no env var set"
    );
    assert_eq!(
        fold,
        PeerFold {
            connect: vec![locator.clone()],
            folded_cached: vec![locator],
            dropped_cached: vec![],
        }
    );
}

/// The TCP PRE-FILTER, with its own anti-tautology control in the same body: a
/// dead cached row is dropped while a LIVE sibling in the same document is
/// dialled. Without the live half, a probe that refused everything would pass.
#[test]
fn a_dead_cached_peer_is_dropped_while_a_live_sibling_is_dialled() {
    let dir = TempDir::new();
    let (_listener, live) = live_locator();
    let dead = dead_locator();
    let now = now_secs();
    write_cache(
        &dir.cache(),
        &[("gone", dead.clone(), now), ("go2", live.clone(), now)],
    );

    let started = Instant::now();
    let fold = fold_from_cache_at(&[], &dir.cache());
    let elapsed = started.elapsed();

    assert_eq!(
        fold.connect,
        vec![live.clone()],
        "only the reachable cached peer is dialled"
    );
    assert_eq!(fold.folded_cached, vec![live]);
    assert_eq!(
        fold.dropped_cached,
        vec![(dead, DiscoveryRung::Cache)],
        "the dead row is dropped, and reported with the rung that produced it"
    );
    // BOUNDED: the probe round is parallel and budgeted, so a stale cache can
    // never stall the daemon's boot. (The budget itself is a `cerulion_discovery`
    // constant; this is the "it is bounded at all" pin.)
    assert!(
        elapsed < Duration::from_secs(5),
        "the fold must stay bounded even with a dead row; took {elapsed:?}"
    );
}

/// Cache rows that cannot even be probed — an unresolvable host and a malformed
/// locator — are dropped rather than folded, and neither stalls the fold. These
/// are what a hand-edited or version-skewed cache produces.
#[test]
fn unprobeable_cached_rows_are_dropped_not_dialled() {
    let dir = TempDir::new();
    let (_listener, live) = live_locator();
    let now = now_secs();
    write_cache(
        &dir.cache(),
        &[
            // `.invalid` is reserved by RFC 2606 and never resolves.
            ("bogus-host", "tcp/robot.invalid:7683".to_string(), now),
            ("malformed", "not-a-locator".to_string(), now),
            ("go2", live.clone(), now),
        ],
    );

    let started = Instant::now();
    let fold = fold_from_cache_at(&[], &dir.cache());
    assert!(
        started.elapsed() < Duration::from_secs(5),
        "an unresolvable row must not stall the fold"
    );
    assert_eq!(fold.connect, vec![live], "only the live row survives");
    assert_eq!(
        fold.dropped_cached.len(),
        2,
        "both unprobeable rows are dropped: {:?}",
        fold.dropped_cached
    );
}

/// THE POLICY PIN over REAL probes: an operator's `CERULION_NETD_CONNECT`
/// locator is dialled even when it is provably unreachable RIGHT NOW, and it
/// leads the connect set — netd's session is long-lived and unbounded, so zenoh
/// retries it on a background connector and a robot that is merely booting comes
/// up on its own. The cached rows in the SAME document are still probe-gated, so
/// this asserts the two policies against each other.
#[test]
fn an_unreachable_explicit_locator_still_leads_the_connect_set() {
    let dir = TempDir::new();
    let (_listener, live) = live_locator();
    let dead_explicit = dead_locator();
    let dead_cached = dead_locator();
    let now = now_secs();
    write_cache(
        &dir.cache(),
        &[
            ("gone", dead_cached.clone(), now),
            ("go2", live.clone(), now),
        ],
    );

    let before = desk_network_config(vec![dead_explicit.clone()], Vec::new());
    let (after, fold) = fold_cached_peers_at(before, &dir.cache());

    assert_eq!(
        after.connect_endpoints,
        vec![dead_explicit, live.clone()],
        "the operator's locator survives a failed probe AND comes first; the cached \
         rows are still probe-gated"
    );
    assert_eq!(fold.folded_cached, vec![live]);
    assert_eq!(
        fold.dropped_cached,
        vec![(dead_cached, DiscoveryRung::Cache)]
    );
}

/// A cache that cannot be read — absent, corrupt, or written by a future schema
/// version — degrades to EXACTLY the explicit locators (earlier behaviour),
/// never an error and never a stall. The reader is loud about each case; the
/// fold's job is to keep going.
#[test]
fn an_unreadable_cache_degrades_to_exactly_the_explicit_locators() {
    let dir = TempDir::new();
    let explicit = strs(&["tcp/1.1.1.1:7683"]);
    let expected = PeerFold {
        connect: explicit.clone(),
        folded_cached: vec![],
        dropped_cached: vec![],
    };

    // (a) absent — a first run on a brand-new laptop.
    assert_eq!(
        fold_from_cache_at(&explicit, &dir.path.join("never-written.json")),
        expected
    );

    // (b) corrupt.
    let corrupt = dir.path.join("corrupt.json");
    std::fs::write(&corrupt, b"{ this is not json").unwrap();
    assert_eq!(fold_from_cache_at(&explicit, &corrupt), expected);

    // (c) a future schema version — structurally valid, deliberately not honored.
    let future = dir.path.join("future.json");
    std::fs::write(
        &future,
        br#"{"v":99,"peers":[{"robot":"go2","locator":"tcp/10.0.0.5:7683","last_seen":9}]}"#,
    )
    .unwrap();
    assert_eq!(fold_from_cache_at(&explicit, &future), expected);

    // (d) and with no env locators either, an unreadable cache folds NOTHING —
    // the daemon is exactly as it was before this feature (scouting only).
    assert_eq!(fold_from_cache_at(&[], &corrupt), PeerFold::default());
}

/// A TTL-EXPIRED row is never dialled — even though its locator is genuinely
/// reachable right now. The live control in the same document proves the fold is
/// not simply refusing everything, so this isolates the TTL from reachability.
#[test]
fn an_expired_cache_entry_is_never_dialled_even_when_reachable() {
    let dir = TempDir::new();
    let (_stale_listener, stale_but_live) = live_locator();
    let (_fresh_listener, fresh) = live_locator();
    let now = now_secs();
    // `PEER_TTL_SECS` is 7 days; stamp the first row well beyond it.
    let long_ago = now.saturating_sub(30 * 24 * 60 * 60);
    write_cache(
        &dir.cache(),
        &[
            ("ancient", stale_but_live.clone(), long_ago),
            ("go2", fresh.clone(), now),
        ],
    );

    let fold = fold_from_cache_at(&[], &dir.cache());
    assert_eq!(
        fold.connect,
        vec![fresh],
        "an expired row is evicted by the reader before the probe ever sees it"
    );
    assert!(
        !fold
            .dropped_cached
            .iter()
            .any(|(loc, _)| loc == &stale_but_live),
        "an expired row is not even a candidate — it is never probed, so it cannot \
         appear as a probe drop: {:?}",
        fold.dropped_cached
    );
}

/// The fold touches the connect endpoints and NOTHING else — scouting stays on,
/// the listen endpoints and the machine identity are carried through untouched.
/// A fold that reset the session posture would break LAN discovery for every
/// desk while looking like it worked.
#[test]
fn the_fold_changes_only_the_connect_endpoints() {
    let dir = TempDir::new();
    let (_listener, locator) = live_locator();
    write_cache(&dir.cache(), &[("go2", locator.clone(), now_secs())]);

    let mut before = desk_network_config(Vec::new(), strs(&["tcp/0.0.0.0:7683"]));
    before.robot_identity = Some("desk-42".to_string());
    let expected_rest = before.clone();

    let (after, _fold) = fold_cached_peers_at(before, &dir.cache());

    assert_eq!(after.connect_endpoints, vec![locator]);
    assert!(
        after.multicast_scouting && after.gossip_scouting,
        "scouting stays ON"
    );
    assert_eq!(after.listen_endpoints, expected_rest.listen_endpoints);
    assert_eq!(after.robot_identity, expected_rest.robot_identity);
    assert_eq!(after.mode, expected_rest.mode);
    assert_eq!(
        after.bounded_connect, expected_rest.bounded_connect,
        "netd's session is deliberately NOT bounded_connect — it is long-lived, so \
         zenoh retries a configured endpoint on a background connector"
    );
}

/// Two folds over the same cache produce the same connect set, in the same
/// ORDER — the daemon's dial list must not depend on `HashSet` iteration order.
#[test]
fn the_fold_is_deterministic() {
    let dir = TempDir::new();
    let (_a, live_a) = live_locator();
    let (_b, live_b) = live_locator();
    let dead = dead_locator();
    let now = now_secs();
    write_cache(
        &dir.cache(),
        &[
            ("alpha", live_a.clone(), now),
            ("gone", dead.clone(), now),
            ("beta", live_b.clone(), now),
        ],
    );
    let explicit = strs(&["tcp/1.1.1.1:7683"]);

    let first = fold_from_cache_at(&explicit, &dir.cache());
    let second = fold_from_cache_at(&explicit, &dir.cache());
    assert_eq!(first, second, "two folds over one cache must agree");
    // And against a HAND oracle, so this is not a self-compare: explicit first,
    // then the live rows in the document's own order, dead row dropped.
    assert_eq!(
        first,
        PeerFold {
            connect: vec![
                "tcp/1.1.1.1:7683".to_string(),
                live_a.clone(),
                live_b.clone()
            ],
            folded_cached: vec![live_a, live_b],
            dropped_cached: vec![(dead, DiscoveryRung::Cache)],
        }
    );
}

// ─── LEVEL pins for the fold's log arms ──────────────────────────────────────
//
// netd's binary installs `EnvFilter::new("info")` by default, so a line emitted
// at `debug!` is INVISIBLE on a shipping desk. Every claim this module makes
// about being "loud" is therefore a claim about a LEVEL, and only a
// level-matching predicate can pin it: a message-only filter passes a demotion
// to `debug!` with the text intact. Same discipline as the flood-latch pins.

/// Does this captured line carry `level` as a WHOLE token in its header?
/// `tracing-test` renders the span name (the test fn's own name) into every
/// line, so a bare `contains("INFO")` could be satisfied by a rename or a field
/// value and silently invert an "exactly N" oracle.
fn line_is(line: &str, level: &str) -> bool {
    line.split_whitespace().any(|t| t == level)
}

/// THE SUCCESS LINE, at `info!`. This is the only thing that tells an operator
/// why a desk with no env var suddenly reaches a robot, and it must survive the
/// shipped default filter. It also carries `dropped`, because folding 1 of 9 is
/// a different situation from folding 1 of 1.
#[test]
#[traced_test]
fn the_successful_fold_is_announced_at_info_with_both_counts() {
    let dir = TempDir::new();
    let (_listener, live) = live_locator();
    let dead = dead_locator();
    let now = now_secs();
    write_cache(
        &dir.cache(),
        &[("go2", live.clone(), now), ("gone", dead, now)],
    );

    let (_cfg, fold) =
        fold_cached_peers_at(desk_network_config(Vec::new(), Vec::new()), &dir.cache());
    assert_eq!(fold.folded_cached.len(), 1);
    assert_eq!(fold.dropped_cached.len(), 1);

    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| line_is(l, "INFO") && l.contains("folded verified cached peers"))
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "expected exactly 1 INFO success line, got {}: {lines:?}",
                hits.len()
            ));
        }
        // The dropped count rides the same line — an operator reading only the
        // success line must still see that something did NOT make it.
        if !hits[0].split_whitespace().any(|t| t == "dropped=1") {
            return Err(format!(
                "success line must carry dropped=1 as a field: {}",
                hits[0]
            ));
        }
        Ok(())
    });
}

/// THE FAILURE LINE this feature newly creates: cached rows EXISTED and none
/// survived the probe (the robot was asleep at daemon boot). The resulting
/// connect set is byte-identical to "cache empty", so without a loud line the
/// operator has nothing at all — and per the once-at-boot cadence it persists
/// for the life of a vizd-held daemon. Fires once per boot, so no flood risk.
#[test]
#[traced_test]
fn a_fold_where_every_cached_peer_failed_is_announced_at_info() {
    let dir = TempDir::new();
    let now = now_secs();
    write_cache(
        &dir.cache(),
        &[
            ("gone", dead_locator(), now),
            ("also-gone", dead_locator(), now),
        ],
    );

    let (_cfg, fold) =
        fold_cached_peers_at(desk_network_config(Vec::new(), Vec::new()), &dir.cache());
    assert!(fold.folded_cached.is_empty());
    assert_eq!(fold.dropped_cached.len(), 2);

    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| line_is(l, "INFO") && l.contains("failed the reachability probe"))
            .collect();
        if hits.len() != 1 {
            return Err(format!(
                "expected exactly 1 INFO all-failed line, got {}: {lines:?}",
                hits.len()
            ));
        }
        if !hits[0].split_whitespace().any(|t| t == "dropped=2") {
            return Err(format!("all-failed line must carry dropped=2: {}", hits[0]));
        }
        Ok(())
    });
}

/// ANTI-TAUTOLOGY for both arms above: the ORDINARY cold desk — no cache at all —
/// says NOTHING at `info!`. Without this, a fold that logged unconditionally
/// would satisfy every "exactly 1" oracle here, and a brand-new laptop would
/// carry a scary line about a feature that simply had nothing to do.
#[test]
#[traced_test]
fn a_desk_with_no_cache_at_all_says_nothing_at_info() {
    let dir = TempDir::new();
    let (_cfg, fold) =
        fold_cached_peers_at(desk_network_config(Vec::new(), Vec::new()), &dir.cache());
    assert_eq!(fold, PeerFold::default());

    logs_assert(
        |lines: &[&str]| match lines.iter().filter(|l| line_is(l, "INFO")).count() {
            0 => Ok(()),
            n => Err(format!(
                "an empty cache must be quiet at info, got {n}: {lines:?}"
            )),
        },
    );
}
