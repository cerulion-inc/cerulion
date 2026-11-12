// SPDX-License-Identifier: AGPL-3.0-only
//! The LOUD once-per-robot fallback warn on an undecodable `catalog`
//! reply. A robot serves GARBAGE bytes on `cerulion_q/{robot}/catalog`; the
//! demander's `discovery::query_robot_catalog` returns `None` (contributes
//! nothing to the topic list) AND emits EXACTLY ONE `warn!` naming the robot +
//! the reason. Two faulted robots warn INDEPENDENTLY (per-robot, not deduped /
//! shared).
//!
//! Lives in `cerulion_core` (not the cli_engine loopback file the review
//! suggested) because the fault injection declares a RAW `session.declare_queryable`
//! replying garbage — which needs to name `zenoh::Session` / `zenoh::Wait`, and
//! `zenoh` is not a dependency of `cerulion_cli_engine`. `query_robot_catalog` is
//! called DIRECTLY on the test thread (not through the scoped-thread fan-out
//! `query_robot_catalogs`), so `#[traced_test]`'s thread-local subscriber
//! captures the warn deterministically. Real zenoh loopback sessions, no
//! iceoryx2 — parallel-safe (no `#[serial]`, pid-derived ports).

use cerulion_core::transport::cerulion_q;
use cerulion_core::transport::discovery::{query_announce_entries, query_robot_catalog};
use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
use std::time::Duration;
use tracing_test::traced_test;
use zenoh::Wait;

/// Marker substring of the once-per-robot fallback warn (see
/// `discovery::query_robot_catalog`).
const WARN_MARKER: &str = "catalog reply from this robot could not be used";
/// The window ONE catalog GET waits — GENEROUS on purpose: the warm-up below
/// confirms the link is up, so the garbage reply arrives well within this; a
/// single GET yields a single reply → a single warn (cardinality is per-call).
const CATALOG_WINDOW: Duration = Duration::from_secs(1);
const GATHER_WINDOW: Duration = Duration::from_millis(300);

/// Declare a RAW zenoh queryable on `cerulion_q/{robot}/catalog` that replies
/// GARBAGE bytes (never a valid catalog payload) — the "bad reply" fault. The
/// returned handle must be kept alive through the GET.
fn declare_bad_catalog_queryable(
    session: &zenoh::Session,
    robot: &str,
) -> zenoh::query::Queryable<()> {
    let key = cerulion_q::catalog_selector(robot);
    let reply_key = key.clone();
    session
        .declare_queryable(&key)
        .callback(move |query| {
            // Reply with bytes that decode neither as a valid catalog nor as any
            // JSON — the `Malformed` decode-error class.
            let _ = query
                .reply(reply_key.as_str(), b"not a catalog at all".to_vec())
                .wait();
        })
        .wait()
        .expect("declare bad catalog queryable")
}

#[traced_test]
#[test]
fn undecodable_catalog_reply_warns_once_per_robot_independently() {
    let robot_a = "warna";
    let robot_b = "warnb";

    // ── Robot side: LISTEN, declare a bad catalog queryable for BOTH robot
    //    identities on the one session, and announce identity (for the observer's
    //    warm-up link check).
    let pid = std::process::id();
    let candidates = [
        22100 + (pid % 300) as u16,
        22500 + (pid % 300) as u16,
        22900 + (pid % 300) as u16,
    ];
    let mut robot_side = None;
    for port in candidates {
        let mgr = NetworkManager::new(NetworkConfig {
            listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
            robot_identity: Some(robot_a.to_string()),
            ..NetworkConfig::default()
        });
        if mgr.session().is_ok() {
            robot_side = Some((mgr, port));
            break;
        }
    }
    let (robot_mgr, port) =
        robot_side.expect("no candidate localhost port could be bound for the robot session");
    let robot_session = robot_mgr.session().expect("robot session (cached)");
    let _qa = declare_bad_catalog_queryable(robot_session, robot_a);
    let _qb = declare_bad_catalog_queryable(robot_session, robot_b);
    // Announce identity so the observer can confirm the link is up (warm-up)
    // WITHOUT ever touching the catalog path (no catalog warn during warm-up).
    robot_mgr
        .announce_gateway_identity()
        .expect("announce identity token");

    // ── Observer side: CONNECT, one persistent session.
    let observer = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        ..NetworkConfig::default()
    });
    let obs_session = observer.session().expect("observer session opens");

    // Warm-up: retry the ANNOUNCE gather until the robot's identity token is
    // visible on THIS session — proving the peer link + the accepter's
    // declarations (incl. the catalog queryables) have propagated. This uses the
    // liveliness query path only, so it emits NO catalog warn.
    let mut linked = false;
    for _ in 0..40 {
        if let Ok(entries) = query_announce_entries(obs_session, GATHER_WINDOW) {
            if entries.iter().any(|(robot, _)| robot == robot_a) {
                linked = true;
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    assert!(
        linked,
        "the observer never saw the robot's announce token — link not established, \
         cannot exercise the catalog GET deterministically"
    );

    // ── Exactly ONE catalog GET per robot, on the warm session. Each receives the
    //    garbage reply → returns None (contributes nothing) → warns ONCE.
    assert!(
        query_robot_catalog(obs_session, robot_a, CATALOG_WINDOW).is_none(),
        "an undecodable catalog reply must contribute nothing (None) for robot_a"
    );
    assert!(
        query_robot_catalog(obs_session, robot_b, CATALOG_WINDOW).is_none(),
        "an undecodable catalog reply must contribute nothing (None) for robot_b"
    );

    // ── EXACTLY ONE warn per robot, naming the robot + the reason, INDEPENDENTLY
    //    (not shared / deduped across robots).
    logs_assert(|lines: &[&str]| {
        let warns: Vec<&&str> = lines.iter().filter(|l| l.contains(WARN_MARKER)).collect();
        if warns.len() != 2 {
            return Err(format!(
                "expected EXACTLY 2 catalog fallback warns (one per robot), got {}: {warns:?}",
                warns.len()
            ));
        }
        let names_a = warns.iter().filter(|l| l.contains(robot_a)).count();
        let names_b = warns.iter().filter(|l| l.contains(robot_b)).count();
        if names_a != 1 || names_b != 1 {
            return Err(format!(
                "each robot must be named by EXACTLY ONE warn (independent, not deduped): \
                 robot_a={names_a}, robot_b={names_b}; warns={warns:?}"
            ));
        }
        // The reason (the Malformed decode-error class Display) must be surfaced.
        if !warns.iter().all(|l| l.contains("did not parse as JSON")) {
            return Err(format!(
                "every warn must name the actionable reason (JSON decode class); warns={warns:?}"
            ));
        }
        Ok(())
    });
}
