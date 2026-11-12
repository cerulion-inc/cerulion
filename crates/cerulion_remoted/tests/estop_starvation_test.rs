// SPDX-License-Identifier: AGPL-3.0-only
//! — the un-starvable e-stop channel, over REAL iroh endpoints.
//!
//! The safety requirement: a legitimate remote `engage-estop` must reach the lease
//! within a SMALL bounded latency regardless of how many other sessions (hostile
//! or not) are in flight. Ops sessions SERIALIZED behind one permit would let a
//! stalled/reconnect-loop attacker delay a legitimate e-stop up to the
//! session deadline. The daemon serves sessions CONCURRENTLY over a shared
//! `Arc<OpsServer>` whose receipt sink is a brief-append `Mutex`, so the safety
//! verb never waits on an attacker's session.
//!
//! Every await is bounded so CI can never hang; every oracle is hand-built (real
//! crypto, deterministic fixed-seed keys — Principle #13); the lease + receipt
//! state is asserted against the shared handle / a disk reload, never a
//! self-compare.

mod common;

use std::time::{Duration, Instant};

use cerud::lease::EstopState;
use cerud::receipt::{read_all, verify_chain, ReceiptOutcome};

use common::{build_robot, disabled_endpoint, loopback_addr, run_client, with_serving, OWNER};

/// The concrete latency ceiling for a remote `engage-estop` while hostile
/// sessions are stalled in-flight. With concurrent serving the e-stop
/// returns in well under this; serialized behind one permit it blocks on a staller's
/// session and exceeds it — so this bound is the discriminating pin.
const ESTOP_CEILING: Duration = Duration::from_secs(3);
/// How long each hostile staller holds its session (well over the ceiling, so a
/// serialized model would make the e-stop wait this long, but under the harness
/// bound so the test never hangs).
const STALL_SECS: u64 = 6;
/// Number of concurrent stalled sessions (proves "regardless of HOW MANY").
const STALLERS: usize = 3;

// ── doc pin ───────────────────────────────────────────────────────────────────

#[test]
fn ops_session_deadline_is_the_120s_value_the_docs_cite() {
    // DOC PIN: `docs/remote_plane.md` documents the per-session
    // deadline as 120 s (the resource bound that unwedges a stalled session, no
    // longer a serialization point). This hard-codes the value the doc cites, so
    // the constant cannot silently drift out from under the doc — re-tuning it
    // fails THIS test, forcing the doc to be updated in the same change (the
    // sibling of the 500 ms deadman pin in cerud/tests/constants_test.rs).
    assert_eq!(
        cerulion_remoted::ops::OPS_SESSION_DEADLINE,
        Duration::from_secs(120),
        "docs/remote_plane.md cites a 120 s per-session deadline; update the doc if you re-tune it"
    );
}

// ── the headline safety pin ───────────────────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn estop_is_served_under_multiple_stalled_sessions_within_a_small_bound() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([230; 32]).await;
    let estop_client = disabled_endpoint([70; 32]).await; // paired → runs engage-estop
    let estop_key = *estop_client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    // The e-stop client is a paired OWNER; the stallers are UNPAIRED (the
    // bootstrap surface admits any TLS-authed key — the realistic attacker).
    let r = build_robot(
        dir.path(),
        robot_key,
        /*claim_owner=*/ true,
        &[(estop_key, OWNER)],
    );

    // Build the staller endpoints up front (distinct keys, all unpaired).
    let mut stallers = Vec::new();
    for i in 0..STALLERS {
        stallers.push(disabled_endpoint([80 + i as u8; 32]).await);
    }

    let (elapsed, estop_res) = with_serving(&r, &robot, async {
        // Drive STALLERS concurrent hostile sessions (each opens the ops session,
        // completes the cerud handshake, then stalls) TOGETHER with a timed e-stop.
        // `tokio::join!` polls all concurrently; each staller's own future returns
        // only after its stall, but the e-stop future TIMES ITSELF internally, so
        // the measured latency is the e-stop's alone.
        let estop_fut = async {
            // Give the stallers a beat to establish their sessions first (in a
            // serialized model the first staller now holds the permit).
            tokio::time::sleep(Duration::from_millis(1000)).await;
            let start = Instant::now();
            let res = run_client(&estop_client, robot_addr.clone(), |c| {
                c.call("engage-estop", serde_json::json!({}))
            })
            .await;
            (start.elapsed(), res)
        };

        // A fixed fan-out of stallers (STALLERS == 3).
        let s0 = run_client(&stallers[0], robot_addr.clone(), |_c| {
            std::thread::sleep(Duration::from_secs(STALL_SECS));
        });
        let s1 = run_client(&stallers[1], robot_addr.clone(), |_c| {
            std::thread::sleep(Duration::from_secs(STALL_SECS));
        });
        let s2 = run_client(&stallers[2], robot_addr.clone(), |_c| {
            std::thread::sleep(Duration::from_secs(STALL_SECS));
        });

        let (_, _, _, out) = tokio::join!(s0, s1, s2, estop_fut);
        out
    })
    .await;

    // The e-stop was served + the lease reached the Engaged floor.
    let engaged = estop_res.expect("engage-estop is served despite the stalled sessions");
    assert_eq!(engaged["engaged"], true);
    assert_eq!(engaged["by"], hex::encode(estop_key));

    // THE discriminating assert: the e-stop reached the lease within the small
    // ceiling. A serialized model would block it on a staller's ~6 s session and
    // BLOW this bound (a serialized server fails exactly here).
    assert!(
        elapsed < ESTOP_CEILING,
        "engage-estop took {elapsed:?}, exceeding the {ESTOP_CEILING:?} ceiling — the safety \
         floor was starved by the stalled sessions (a serialized ops plane)"
    );

    // The lease is durably in the Engaged floor (the effect actually happened).
    {
        let lease = r.lease.lock().unwrap();
        assert_eq!(
            lease.estop(),
            &EstopState::Engaged {
                by: hex::encode(estop_key)
            }
        );
    }

    // Hash-chain integrity: the e-stop's receipt landed and the chain verifies
    // (the stallers ran no verb, so exactly one receipt — the e-stop outcome).
    let rx = read_all(&r.receipt_path).expect("read receipts");
    assert_eq!(rx.len(), 1, "the served e-stop is receipted exactly once");
    assert_eq!(rx[0].verb, "engage-estop");
    assert_eq!(rx[0].outcome, ReceiptOutcome::Ok);
    verify_chain(&rx).expect("the receipt chain is intact after the concurrent run");
}

// ── concurrent-append hash-chain integrity ────────────────────────────────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn concurrent_appends_keep_the_hash_chain_intact() {
    // Five PAIRED clients run a verb CONCURRENTLY, so the shared hash-chained
    // receipt sink takes five concurrent appends. The sink's brief-append
    // `Mutex` serializes each append atomically, so the interleaved entries still
    // form ONE valid chain with contiguous seqs — the concurrency invariant that
    // replaces the whole-session serialization.
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([231; 32]).await;
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    // Five distinct paired device keys, all bound to OWNER.
    let mut clients = Vec::new();
    for i in 0..5 {
        clients.push(disabled_endpoint([90 + i as u8; 32]).await);
    }
    let bindings: Vec<([u8; 32], _)> = clients
        .iter()
        .map(|c| (*c.id().as_bytes(), OWNER))
        .collect();
    let r = build_robot(dir.path(), robot_key, /*claim_owner=*/ true, &bindings);

    let results = with_serving(&r, &robot, async {
        let f0 = run_client(&clients[0], robot_addr.clone(), |c| {
            c.call("inventory", serde_json::json!({}))
        });
        let f1 = run_client(&clients[1], robot_addr.clone(), |c| {
            c.call("inventory", serde_json::json!({}))
        });
        let f2 = run_client(&clients[2], robot_addr.clone(), |c| {
            c.call("inventory", serde_json::json!({}))
        });
        let f3 = run_client(&clients[3], robot_addr.clone(), |c| {
            c.call("inventory", serde_json::json!({}))
        });
        let f4 = run_client(&clients[4], robot_addr.clone(), |c| {
            c.call("inventory", serde_json::json!({}))
        });
        let (r0, r1, r2, r3, r4) = tokio::join!(f0, f1, f2, f3, f4);
        [r0, r1, r2, r3, r4]
    })
    .await;

    // Every concurrent inventory succeeded.
    for res in &results {
        assert!(
            res.as_ref().expect("inventory served")["arch"].is_string(),
            "each concurrent paired inventory is served"
        );
    }

    // Exactly five receipts, all Ok, contiguous seqs, chain intact — proof the
    // concurrent appends serialized correctly at the shared sink.
    let rx = read_all(&r.receipt_path).expect("read receipts");
    assert_eq!(rx.len(), 5, "five concurrent verbs → five receipts");
    assert!(rx
        .iter()
        .all(|e| e.verb == "inventory" && e.outcome == ReceiptOutcome::Ok));
    for (i, e) in rx.iter().enumerate() {
        assert_eq!(
            e.seq, i as u64,
            "seqs are contiguous 0..5 under concurrency"
        );
    }
    verify_chain(&rx).expect("the hash chain is intact under concurrent appends");
}

// ── lease reconnect e2e (stable per-pairing token; e-stop persists) ───────────

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn estop_persists_across_a_reconnect_and_a_different_token_cannot_take_the_floor_over_iroh() {
    // The lease over reconnects, proven over REAL iroh: the engager token is the
    // STABLE per-pairing device key (`caller.id`), not the ephemeral QUIC
    // connection. Two DIFFERENT connections from the SAME device key present the
    // SAME token (so a reconnect re-engages under it); e-stop STATE persists across
    // the reconnect; a DIFFERENT paired device key cannot take the floor.
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([232; 32]).await;
    let owner_client = disabled_endpoint([71; 32]).await; // token A (reconnects)
    let other_client = disabled_endpoint([72; 32]).await; // token B (different key)
    let owner_key = *owner_client.id().as_bytes();
    let other_key = *other_client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    // BOTH device keys map to OWNER (both PAIRED so both reach the e-stop floor),
    // but they are DIFFERENT tokens (the token is the device key, not the account).
    let r = build_robot(
        dir.path(),
        robot_key,
        /*claim_owner=*/ true,
        &[(owner_key, OWNER), (other_key, OWNER)],
    );

    let (first, reengage, persisted, cleared_reengage, other_engage, final_estop) =
        with_serving(&r, &robot, async {
            // Connection 1 (token A): engage the floor.
            let first = run_client(&owner_client, robot_addr.clone(), |c| {
                c.call("engage-estop", serde_json::json!({}))
            })
            .await;

            // Connection 2 (token A, a NEW QUIC connection — the reconnect): re-engage.
            // The response echoes the CALLER identity, so its `by` == token A proves
            // this second, distinct connection authenticated as the SAME stable token.
            let reengage = run_client(&owner_client, robot_addr.clone(), |c| {
                c.call("engage-estop", serde_json::json!({}))
            })
            .await;
            // The floor PERSISTED across the reconnect: the new connection did not
            // reset it (idempotent-keep-first → the lease engager is still token A).
            let persisted = r.lease.lock().unwrap().estop().clone();

            // Clear the floor test-side (no clear verb yet), then a THIRD connection
            // from token A re-engages as the FIRST engager after the clear.
            r.lease.lock().unwrap().clear_estop();
            let cleared_reengage = run_client(&owner_client, robot_addr.clone(), |c| {
                c.call("engage-estop", serde_json::json!({}))
            })
            .await;

            // Connection from token B (a DIFFERENT paired key) while the floor is up:
            // it engages successfully (any paired session may) but does NOT take the
            // floor from token A (idempotent-keep-first — the lease keeps token A).
            let other_engage = run_client(&other_client, robot_addr.clone(), |c| {
                c.call("engage-estop", serde_json::json!({}))
            })
            .await;
            let final_estop = r.lease.lock().unwrap().estop().clone();

            (
                first,
                reengage,
                persisted,
                cleared_reengage,
                other_engage,
                final_estop,
            )
        })
        .await;

    // Connection 1 engaged; the response echoes the caller = token A.
    let f = first.expect("connection 1 engages the floor");
    assert_eq!(f["engaged"], true);
    assert_eq!(
        f["by"],
        hex::encode(owner_key),
        "connection 1 authenticated as token A"
    );

    // Connection 2 (a distinct reconnect) authenticated as the SAME stable token A —
    // the engager is the per-pairing device key, NOT the ephemeral QUIC connection.
    let re = reengage.expect("the reconnect re-engages");
    assert_eq!(
        re["by"],
        hex::encode(owner_key),
        "the reconnect (a NEW connection) presents the SAME stable per-pairing token (device key A)"
    );
    // e-stop STATE persisted across the reconnect (the new connection did not drop
    // the floor).
    assert_eq!(
        persisted,
        EstopState::Engaged {
            by: hex::encode(owner_key)
        },
        "e-stop state persists across the reconnect (still engaged by token A)"
    );

    // The third connection (token A, after the clear) also authenticated as token A.
    let cr = cleared_reengage.expect("connection 3 re-engages after the clear");
    assert_eq!(
        cr["by"],
        hex::encode(owner_key),
        "a reconnect after a clear presents the SAME stable token (device key A)"
    );

    // Token B engaged (any paired session may); its response echoes ITS OWN caller
    // id (token B), but the LEASE floor stays owned by token A.
    let ob = other_engage.expect("token B may engage the floor");
    assert_eq!(ob["engaged"], true);
    assert_eq!(
        ob["by"],
        hex::encode(other_key),
        "the e-stop response echoes the calling token (B)"
    );

    // The durable lease state proves a DIFFERENT token could NOT take the floor:
    // it is engaged by token A throughout (idempotent-keep-first).
    assert_eq!(
        final_estop,
        EstopState::Engaged {
            by: hex::encode(owner_key)
        },
        "a DIFFERENT token cannot take the e-stop floor from the first engager (token A)"
    );

    // Every engage was receipted and the chain is intact (four engage-estop calls).
    let rx = read_all(&r.receipt_path).expect("read receipts");
    assert_eq!(rx.len(), 4, "four engage-estop calls → four receipts");
    assert!(rx
        .iter()
        .all(|e| e.verb == "engage-estop" && e.outcome == ReceiptOutcome::Ok));
    // The first two engages carry token A, the last (after clear) also A, the
    // token-B call carries token B (the CALLER identity is the device key).
    assert_eq!(rx[0].caller, hex::encode(owner_key));
    assert_eq!(rx[3].caller, hex::encode(other_key));
    verify_chain(&rx).expect("the receipt chain is intact across the reconnects");
}
