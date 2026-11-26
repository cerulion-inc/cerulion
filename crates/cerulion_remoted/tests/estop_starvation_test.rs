// SPDX-License-Identifier: AGPL-3.0-only
//! — the un-starvable e-stop channel, over REAL iroh endpoints.
//!
//! The safety requirement: a legitimate remote `engage-estop` must reach the lease
//! within a SMALL bounded latency without waiting behind other sessions. Ops
//! sessions SERIALIZED behind one permit would let a stalled/reconnect-loop
//! attacker delay a legitimate e-stop up to the session deadline. The daemon
//! serves sessions CONCURRENTLY over a shared `Arc<OpsServer>` whose receipt
//! sink is a brief-append `Mutex`, caps unpaired `OpsBootstrapOnly`, refused,
//! and skeleton connections at 64 permits with a 5 s opening-frame deadline so
//! throwaway peers cannot saturate the runtime's 512-thread blocking pool, and
//! keeps paired `OpsAdmit` sessions exempt from that 64-permit cap (paired
//! sessions still share the runtime's 512-thread blocking pool with one
//! another).
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
/// Number of concurrent stalled sessions for the concurrency-vs-serialization
/// pin (the 600-session saturation test below pins the blocking-pool cap).
const STALLERS: usize = 3;

// ── doc pins ──────────────────────────────────────────────────────────────────

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

#[test]
fn bootstrap_first_frame_deadline_is_the_5s_value_the_docs_cite() {
    assert_eq!(
        cerulion_remoted::ops::OPS_BOOTSTRAP_FIRST_FRAME_DEADLINE,
        Duration::from_secs(5),
        "docs/remote_plane.md cites a 5 s bootstrap first-frame deadline; update the doc if you re-tune it"
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

/// Outgoing demand waits use control workers, leaving inbound ops independent.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn shared_endpoint_outbound_saturation_does_not_delay_estop_delivery() {
    use cerulion_netd::iroh_plane::IrohMirrorPlane;
    use cerulion_netd::mirror::MirrorPlane;
    use cerulion_netd::registry::TopicKey;
    use cerulion_netd::wan::{WanRegistry, WanRobot};
    use std::collections::HashMap;
    use std::sync::Arc;

    const OUTBOUND_CLIENTS: usize = 32;
    let dir = tempfile::tempdir().unwrap();
    let robot_seed = [233; 32];
    let robot = disabled_endpoint(robot_seed).await;
    let estop_client = disabled_endpoint([73; 32]).await;
    let estop_key = *estop_client.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());
    let r = build_robot(
        dir.path(),
        *robot.id().as_bytes(),
        true,
        &[(estop_key, OWNER)],
    );
    let manager = cerulion_core::TransportManager::init_for_test(
        cerulion_core::TransportConfig::default(),
        cerulion_core::testing::iceoryx_test_config(),
    )
    .unwrap();
    let socket = dir.path().join("outbound.sock");
    let service = cerulion_netd::shared_endpoint::OutboundServer::start(
        socket.clone(),
        robot.clone(),
        tokio::runtime::Handle::current(),
        robot_seed,
        manager.clone(),
    )
    .unwrap();
    let stalled = disabled_endpoint([234; 32]).await;
    let stalled_id = stalled.id();
    let addresses: Vec<_> = stalled
        .bound_sockets()
        .iter()
        .map(|address| match address {
            std::net::SocketAddr::V4(v4) if v4.ip().is_unspecified() => {
                std::net::SocketAddr::from((std::net::Ipv4Addr::LOCALHOST, v4.port()))
            }
            std::net::SocketAddr::V6(v6) if v6.ip().is_unspecified() => {
                std::net::SocketAddr::from((std::net::Ipv6Addr::LOCALHOST, v6.port()))
            }
            address => *address,
        })
        .collect();
    let (accepted_tx, mut accepted_rx) = tokio::sync::mpsc::channel(OUTBOUND_CLIENTS);
    let staller = tokio::spawn(async move {
        let mut connections = Vec::new();
        while let Some(accepted) = cerulion_link::accept_one(&stalled).await.unwrap() {
            connections.push(accepted.connection);
            if accepted_tx.send(()).await.is_err() {
                break;
            }
        }
    });
    let mut workers = Vec::new();
    for _ in 0..OUTBOUND_CLIENTS {
        let plane = IrohMirrorPlane::new_shared(
            manager.clone(),
            Arc::new(WanRegistry::new(
                HashMap::from([(
                    "stalled".into(),
                    WanRobot {
                        eid: stalled_id,
                        direct_addrs: addresses.clone(),
                    },
                )]),
                robot_seed,
                cerulion_link::RelayConfig::Disabled,
            )),
            socket.clone(),
        )
        .unwrap();
        workers.push(std::thread::spawn(move || {
            plane.ensure_mirror(&TopicKey::new("stalled", "/saturated"), 7)
        }));
    }
    common::bounded("all outbound demands connected", async {
        for _ in 0..OUTBOUND_CLIENTS {
            accepted_rx.recv().await.expect("outgoing wire connection");
        }
    })
    .await;
    assert_eq!(service.active_clients().unwrap(), OUTBOUND_CLIENTS);
    let started = Instant::now();
    let engaged = with_serving(
        &r,
        &robot,
        run_client(&estop_client, robot_addr, |client| {
            client.call("engage-estop", serde_json::json!({}))
        }),
    )
    .await
    .expect("inbound e-stop delivered during outbound saturation");
    assert!(started.elapsed() < ESTOP_CEILING);
    assert_eq!(engaged["engaged"], true);
    assert_eq!(
        r.lease.lock().unwrap().estop(),
        &EstopState::Engaged {
            by: hex::encode(estop_key),
        }
    );
    let receipts = read_all(&r.receipt_path).unwrap();
    assert_eq!(receipts.len(), 1);
    assert_eq!(receipts[0].verb, "engage-estop");
    assert_eq!(receipts[0].outcome, ReceiptOutcome::Ok);
    verify_chain(&receipts).unwrap();
    staller.abort();
    let _ = staller.await;
    tokio::task::spawn_blocking(move || {
        drop(service);
        for worker in workers {
            assert!(worker.join().unwrap().is_err());
        }
    })
    .await
    .unwrap();
    common::bounded("shared endpoint close", robot.close()).await;
    common::bounded("e-stop client close", estop_client.close()).await;
}

/// Run an ops client closure on an OS thread (not tokio's blocking pool) so the
/// client never contends for the robot's blocking pool in starvation tests.
async fn run_client_off_blocking_pool<F, R>(
    client: &cerulion_link::Endpoint,
    robot_addr: cerulion_link::EndpointAddr,
    f: F,
) -> R
where
    F: FnOnce(&mut cerud::client::OpsClient<cerulion_link::QuicOpsStream>) -> R + Send + 'static,
    R: Send + 'static,
{
    let conn = common::bounded(
        "dial",
        cerulion_link::dial(client, robot_addr, cerulion_link::alpn::OPS),
    )
    .await
    .expect("dial ok");
    let (send, recv) = common::bounded("open_bi", cerulion_link::open_frame_stream(&conn))
        .await
        .expect("open_bi ok");
    let ops = cerulion_link::QuicOpsStream::new(send, recv);
    let (tx, rx) = tokio::sync::oneshot::channel();
    let thread = std::thread::spawn(move || {
        let mut client = cerud::client::OpsClient::connect_current(ops).expect("cerud handshake");
        let _ = tx.send(f(&mut client));
    });
    let out = common::bounded("off-pool client task", rx)
        .await
        .expect("off-pool client returned a value");
    let _ = thread.join();
    drop(conn);
    out
}

/// Wait for `conn` to close within `budget` and return its QUIC application
/// close reason bytes if it closed with `ApplicationClosed`.
async fn wait_app_close_reason(
    conn: &cerulion_link::Connection,
    budget: Duration,
) -> Option<Vec<u8>> {
    match tokio::time::timeout(budget, conn.closed()).await {
        Ok(cerulion_link::ConnectionError::ApplicationClosed(close)) => Some(close.reason.to_vec()),
        Ok(_) | Err(_) => None,
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn bootstrap_session_completes_normally_within_first_frame_deadline() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([235; 32]).await;
    let peer = disabled_endpoint([75; 32]).await;
    let peer_key = *peer.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    let r = build_robot(dir.path(), *robot.id().as_bytes(), true, &[]);
    let viewer_account = cerulion_pairing::format::AccountId([21; 32]);
    let pres_hex = common::viewer_presentation_hex(viewer_account, peer_key);

    let paired = with_serving(&r, &robot, async {
        run_client(&peer, robot_addr.clone(), move |c| {
            c.call(
                "pair",
                serde_json::json!({ "presentation_postcard": pres_hex }),
            )
        })
        .await
    })
    .await
    .expect("unpaired bootstrap pair verb completes promptly within first-frame deadline");

    assert_eq!(paired["paired"], true);
    assert_eq!(
        r.classify
            .classify_accept(cerulion_link::alpn::OPS, &peer_key),
        cerulion_remoted::AcceptDecision::OpsAdmit
    );

    let rx = read_all(&r.receipt_path).expect("read receipts");
    assert_eq!(
        rx.len(),
        2,
        "mutating pair writes intent + outcome receipts"
    );
    verify_chain(&rx).expect("receipt chain intact");

    common::bounded("robot close", robot.close()).await;
    common::bounded("peer close", peer.close()).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn estop_is_served_under_600_stalled_bootstrap_sessions_within_bound() {
    use std::sync::Arc;

    const STALLED_CONNECTIONS: usize = 600;
    const UNPAIRED_CAP: usize = 64;
    const CLIENT_ENDPOINTS: usize = 6;
    const MAX_CONCURRENT_DIALS: usize = 12;

    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([240; 32]).await;
    let estop_client = disabled_endpoint([74; 32]).await;
    let estop_key = *estop_client.id().as_bytes();
    let robot_key = *robot.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    let r = build_robot(dir.path(), robot_key, true, &[(estop_key, OWNER)]);

    // Multiplex 600 real QUIC connections across 6 unpaired endpoints so macOS's
    // 256-FD soft limit is respected while establishing 600 distinct connections.
    let mut endpoints = Vec::with_capacity(CLIENT_ENDPOINTS);
    for i in 0..CLIENT_ENDPOINTS {
        endpoints.push(Arc::new(disabled_endpoint([100 + i as u8; 32]).await));
    }

    let (cap_refused, deadline_closed, still_open, elapsed, estop_res) =
        with_serving(&r, &robot, async {
            let dial_gate = Arc::new(tokio::sync::Semaphore::new(MAX_CONCURRENT_DIALS));
            let mut dial_tasks = tokio::task::JoinSet::new();

            for idx in 0..STALLED_CONNECTIONS {
                let ep = Arc::clone(&endpoints[idx % CLIENT_ENDPOINTS]);
                let addr = robot_addr.clone();
                let gate = Arc::clone(&dial_gate);
                dial_tasks.spawn(async move {
                    let _permit = gate.acquire_owned().await.unwrap();
                    let conn = cerulion_link::dial(&ep, addr, cerulion_link::alpn::OPS)
                        .await
                        .expect("unpaired bootstrap dial completes TLS");
                    let stream = match cerulion_link::open_frame_stream(&conn).await {
                        Ok((mut send, recv)) => {
                            if send.write_all(&[0x00]).await.is_ok() {
                                Some((send, recv))
                            } else {
                                None
                            }
                        }
                        Err(_) => None,
                    };
                    (conn, stream)
                });
            }

            let mut conns = Vec::with_capacity(STALLED_CONNECTIONS);
            while let Some(res) = dial_tasks.join_next().await {
                conns.push(res.expect("dial task did not panic"));
            }

            // Give server-side cap closes a beat to arrive on connections that
            // wrote their 1 byte before the close frame reached the client.
            tokio::time::sleep(Duration::from_millis(200)).await;

            let mut cap_refused = 0usize;
            let mut deadline_closed = 0usize;
            let mut still_open = 0usize;
            for (conn, _) in &conns {
                match conn.close_reason() {
                    Some(cerulion_link::ConnectionError::ApplicationClosed(ref close))
                        if close.reason.as_ref() == b"unpaired session cap" =>
                    {
                        cap_refused += 1;
                    }
                    Some(cerulion_link::ConnectionError::ApplicationClosed(ref close))
                        if close.reason.as_ref() == b"ops bootstrap first-frame deadline" =>
                    {
                        deadline_closed += 1;
                    }
                    None => still_open += 1,
                    Some(_) => {}
                }
            }

            let start = Instant::now();
            let res = tokio::time::timeout(
                ESTOP_CEILING + Duration::from_secs(2),
                run_client_off_blocking_pool(&estop_client, robot_addr.clone(), |c| {
                    c.call("engage-estop", serde_json::json!({}))
                }),
            )
            .await;
            let elapsed = start.elapsed();

            drop(conns);
            (cap_refused, deadline_closed, still_open, elapsed, res)
        })
        .await;

    let engaged = estop_res
        .expect("engage-estop did not time out behind 600 stalled bootstrap sessions")
        .expect("engage-estop succeeded");
    assert_eq!(engaged["engaged"], true);
    assert_eq!(engaged["by"], hex::encode(estop_key));
    assert!(
        elapsed < ESTOP_CEILING,
        "engage-estop took {elapsed:?}, exceeding {ESTOP_CEILING:?} under 600 stalled bootstrap sessions"
    );
    assert!(
        still_open <= UNPAIRED_CAP,
        "at most {UNPAIRED_CAP} unpaired bootstrap sessions may stay open, got {still_open}"
    );
    assert!(
        cap_refused > 0,
        "excess unpaired connections must hit the unpaired session cap (cap_refused={cap_refused}, deadline_closed={deadline_closed}, still_open={still_open})"
    );
    assert!(
        cap_refused + deadline_closed >= STALLED_CONNECTIONS - UNPAIRED_CAP,
        "expected at least {} excess connections closed by cap or first-frame deadline, got cap={cap_refused} deadline={deadline_closed} open={still_open}",
        STALLED_CONNECTIONS - UNPAIRED_CAP
    );

    {
        let lease = r.lease.lock().unwrap();
        assert_eq!(
            lease.estop(),
            &EstopState::Engaged {
                by: hex::encode(estop_key)
            }
        );
    }
    let rx = read_all(&r.receipt_path).expect("read receipts");
    assert_eq!(rx.len(), 1);
    assert_eq!(rx[0].verb, "engage-estop");
    assert_eq!(rx[0].outcome, ReceiptOutcome::Ok);
    verify_chain(&rx).expect("receipt chain intact");

    common::bounded("robot endpoint close", robot.close()).await;
    common::bounded("estop client close", estop_client.close()).await;
    for ep in endpoints {
        common::bounded("staller endpoint close", ep.close()).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn stalled_refused_and_skeleton_connections_share_the_unpaired_cap() {
    use std::sync::Arc;

    const UNPAIRED_CAP: usize = 64;
    const CLIENT_ENDPOINTS: usize = 4;
    const MAX_CONCURRENT_DIALS: usize = 8;

    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([241; 32]).await;
    let estop_client = disabled_endpoint([76; 32]).await;
    let estop_key = *estop_client.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    let r = build_robot(
        dir.path(),
        *robot.id().as_bytes(),
        true,
        &[(estop_key, OWNER)],
    );

    let mut endpoints = Vec::with_capacity(CLIENT_ENDPOINTS);
    for i in 0..CLIENT_ENDPOINTS {
        endpoints.push(Arc::new(disabled_endpoint([110 + i as u8; 32]).await));
    }
    let extra_unpaired = disabled_endpoint([119; 32]).await;

    let (refused_65th_wire, refused_65th_ops, elapsed, estop_res) =
        with_serving(&r, &robot, async {
            // Saturate all 64 permits with stalled refused wire connections
            // (unpaired peer on cerulion/wire/2 -> Refuse -> skeleton report
            // bounded by 10 s HANDSHAKE_DEADLINE).
            let dial_gate = Arc::new(tokio::sync::Semaphore::new(MAX_CONCURRENT_DIALS));
            let mut dial_tasks = tokio::task::JoinSet::new();
            for idx in 0..UNPAIRED_CAP {
                let ep = Arc::clone(&endpoints[idx % CLIENT_ENDPOINTS]);
                let addr = robot_addr.clone();
                let gate = Arc::clone(&dial_gate);
                dial_tasks.spawn(async move {
                    let _permit = gate.acquire_owned().await.unwrap();
                    let conn = cerulion_link::dial(&ep, addr, cerulion_link::alpn::WIRE)
                        .await
                        .expect("refused wire dial completes TLS");
                    let (send, recv) = cerulion_link::open_frame_stream(&conn)
                        .await
                        .expect("open_bi on refused wire connection");
                    (conn, send, recv)
                });
            }
            let mut held = Vec::with_capacity(UNPAIRED_CAP);
            while let Some(res) = dial_tasks.join_next().await {
                held.push(res.expect("dial task did not panic"));
            }

            // Let all 64 handlers classify and acquire their permits.
            tokio::time::sleep(Duration::from_millis(150)).await;

            // The 65th refused wire connection AND a 65th unpaired ops
            // bootstrap connection must both be closed immediately with
            // `unpaired session cap`.
            let extra_wire = common::bounded(
                "dial 65th wire",
                cerulion_link::dial(
                    &extra_unpaired,
                    robot_addr.clone(),
                    cerulion_link::alpn::WIRE,
                ),
            )
            .await
            .expect("65th wire TLS completes");
            let refused_65th_wire =
                wait_app_close_reason(&extra_wire, Duration::from_secs(2)).await;

            let extra_ops = common::bounded(
                "dial 65th ops",
                cerulion_link::dial(
                    &extra_unpaired,
                    robot_addr.clone(),
                    cerulion_link::alpn::OPS,
                ),
            )
            .await
            .expect("65th ops TLS completes");
            let refused_65th_ops = wait_app_close_reason(&extra_ops, Duration::from_secs(2)).await;

            // Paired engage-estop is exempt from the shared 64-permit cap.
            let start = Instant::now();
            let res = run_client_off_blocking_pool(&estop_client, robot_addr.clone(), |c| {
                c.call("engage-estop", serde_json::json!({}))
            })
            .await;
            let elapsed = start.elapsed();

            drop(held);
            (refused_65th_wire, refused_65th_ops, elapsed, res)
        })
        .await;

    assert_eq!(
        refused_65th_wire.as_deref(),
        Some(b"unpaired session cap".as_slice()),
        "65th refused wire connection must be closed at the unpaired cap"
    );
    assert_eq!(
        refused_65th_ops.as_deref(),
        Some(b"unpaired session cap".as_slice()),
        "ops bootstrap must share the same 64-permit cap with refused/skeleton connections"
    );
    let engaged = estop_res.expect("paired engage-estop is exempt from the unpaired cap");
    assert_eq!(engaged["engaged"], true);
    assert_eq!(engaged["by"], hex::encode(estop_key));
    assert!(
        elapsed < ESTOP_CEILING,
        "engage-estop took {elapsed:?}, exceeding {ESTOP_CEILING:?}"
    );

    common::bounded("robot close", robot.close()).await;
    common::bounded("estop client close", estop_client.close()).await;
    common::bounded("extra client close", extra_unpaired.close()).await;
    for ep in endpoints {
        common::bounded("staller endpoint close", ep.close()).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn bootstrap_first_frame_deadline_closes_one_byte_staller_and_frees_permit() {
    use std::sync::Arc;

    const UNPAIRED_CAP: usize = 64;
    const CLIENT_ENDPOINTS: usize = 4;
    const MAX_CONCURRENT_DIALS: usize = 8;

    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([242; 32]).await;
    let staller = disabled_endpoint([77; 32]).await;
    let next_peer = disabled_endpoint([78; 32]).await;
    let next_peer_key = *next_peer.id().as_bytes();
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    let mut r = build_robot(dir.path(), *robot.id().as_bytes(), true, &[]);
    // Leave OPS_SESSION_DEADLINE at 120 s; shorten ONLY the bootstrap
    // first-frame deadline so 0-byte, 1-byte, and mid-payload stallers are
    // closed in ~300 ms rather than 120 s.
    r.ops = Arc::new(
        cerulion_remoted::OpsServing::new(
            r.shared.clone(),
            r.lease.clone(),
            r.sessions.clone(),
            r.clock.clone(),
            &r.receipt_path,
            r.dir.clone(),
            "cerulion-remoted-test",
        )
        .expect("ops serving builds")
        .with_bootstrap_first_frame_deadline_for_test(Duration::from_millis(300)),
    );

    let mut pool = Vec::with_capacity(CLIENT_ENDPOINTS);
    for i in 0..CLIENT_ENDPOINTS {
        pool.push(Arc::new(disabled_endpoint([120 + i as u8; 32]).await));
    }

    let viewer_account = cerulion_pairing::format::AccountId([22; 32]);
    let pres_hex = common::viewer_presentation_hex(viewer_account, next_peer_key);

    let (zero_byte_reason, one_byte_reason, mid_payload_reason, next_peer_res) =
        with_serving(&r, &robot, async {
            // 1. 0-byte staller (never opens a bidi stream).
            let c0 = common::bounded(
                "dial 0-byte",
                cerulion_link::dial(&staller, robot_addr.clone(), cerulion_link::alpn::OPS),
            )
            .await
            .expect("dial 0-byte");
            let r0 = wait_app_close_reason(&c0, Duration::from_secs(3)).await;

            // 2. 1-byte staller (opens bidi stream, sends 1 byte of prefix).
            let c1 = common::bounded(
                "dial 1-byte",
                cerulion_link::dial(&staller, robot_addr.clone(), cerulion_link::alpn::OPS),
            )
            .await
            .expect("dial 1-byte");
            let (mut s1, _r1_stream) =
                common::bounded("open_bi 1-byte", cerulion_link::open_frame_stream(&c1))
                    .await
                    .expect("open_bi 1-byte");
            s1.write_all(&[0x00]).await.expect("write 1 byte");
            let r1 = wait_app_close_reason(&c1, Duration::from_secs(3)).await;

            // 3. Mid-payload staller (sends 4-byte length = 64 + 4 bytes of payload).
            let c2 = common::bounded(
                "dial mid-payload",
                cerulion_link::dial(&staller, robot_addr.clone(), cerulion_link::alpn::OPS),
            )
            .await
            .expect("dial mid-payload");
            let (mut s2, _r2_stream) =
                common::bounded("open_bi mid-payload", cerulion_link::open_frame_stream(&c2))
                    .await
                    .expect("open_bi mid-payload");
            s2.write_all(&[0, 0, 0, 64, b'{', b'"', b'x', b'"'])
                .await
                .expect("write partial payload");
            let r2 = wait_app_close_reason(&c2, Duration::from_secs(3)).await;

            // 4. Fill all 64 permits with 1-byte stallers, wait for the 300 ms
            //    first-frame deadline to close them and free all permits, then
            //    verify a subsequent unpaired peer completes `pair`.
            let dial_gate = Arc::new(tokio::sync::Semaphore::new(MAX_CONCURRENT_DIALS));
            let mut tasks = tokio::task::JoinSet::new();
            for idx in 0..UNPAIRED_CAP {
                let ep = Arc::clone(&pool[idx % CLIENT_ENDPOINTS]);
                let addr = robot_addr.clone();
                let gate = Arc::clone(&dial_gate);
                tasks.spawn(async move {
                    let _permit = gate.acquire_owned().await.unwrap();
                    let conn = cerulion_link::dial(&ep, addr, cerulion_link::alpn::OPS)
                        .await
                        .expect("dial staller wave");
                    if let Ok((mut send, recv)) = cerulion_link::open_frame_stream(&conn).await {
                        let _ = send.write_all(&[0x00]).await;
                        (conn, Some((send, recv)))
                    } else {
                        (conn, None)
                    }
                });
            }
            let mut wave = Vec::with_capacity(UNPAIRED_CAP);
            while let Some(res) = tasks.join_next().await {
                wave.push(res.expect("wave task"));
            }
            for (conn, _) in &wave {
                let reason = wait_app_close_reason(conn, Duration::from_secs(3)).await;
                assert_eq!(
                    reason.as_deref(),
                    Some(b"ops bootstrap first-frame deadline".as_slice()),
                    "every 1-byte staller must close on the short first-frame deadline"
                );
            }
            drop(wave);

            let next_peer_res = run_client(&next_peer, robot_addr.clone(), move |c| {
                c.call(
                    "pair",
                    serde_json::json!({ "presentation_postcard": pres_hex }),
                )
            })
            .await;

            (r0, r1, r2, next_peer_res)
        })
        .await;

    assert_eq!(
        zero_byte_reason.as_deref(),
        Some(b"ops bootstrap first-frame deadline".as_slice())
    );
    assert_eq!(
        one_byte_reason.as_deref(),
        Some(b"ops bootstrap first-frame deadline".as_slice())
    );
    assert_eq!(
        mid_payload_reason.as_deref(),
        Some(b"ops bootstrap first-frame deadline".as_slice())
    );
    let paired = next_peer_res.expect("subsequent bootstrap peer succeeds after permits are freed");
    assert_eq!(paired["paired"], true);

    common::bounded("robot close", robot.close()).await;
    common::bounded("staller close", staller.close()).await;
    common::bounded("next peer close", next_peer.close()).await;
    for ep in pool {
        common::bounded("pool close", ep.close()).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn bootstrap_first_frame_error_arms() {
    let dir = tempfile::tempdir().unwrap();
    let robot = disabled_endpoint([243; 32]).await;
    let peer = disabled_endpoint([79; 32]).await;
    let robot_addr = loopback_addr(robot.id(), &robot.bound_sockets());

    let r = build_robot(dir.path(), *robot.id().as_bytes(), true, &[]);

    let (eof_reason, huge_closed, poisoned_reason) = with_serving(&r, &robot, async {
        // 1. Clean EOF mid-prefix -> closed promptly with incomplete first frame.
        let c_eof = common::bounded(
            "dial eof",
            cerulion_link::dial(&peer, robot_addr.clone(), cerulion_link::alpn::OPS),
        )
        .await
        .expect("dial eof");
        let (mut s_eof, _r_eof) =
            common::bounded("open_bi eof", cerulion_link::open_frame_stream(&c_eof))
                .await
                .expect("open_bi eof");
        s_eof.write_all(&[0x00]).await.expect("write 1 byte");
        s_eof.finish().expect("finish mid-prefix");
        let eof_reason = wait_app_close_reason(&c_eof, Duration::from_secs(3)).await;

        // 2. Declared length > MAX_FRAME_BYTES -> preloads only 4 bytes and
        //    cerud::protocol::read_frame rejects with FrameTooLarge without
        //    allocating len bytes or waiting for payload bytes.
        let c_huge = common::bounded(
            "dial huge",
            cerulion_link::dial(&peer, robot_addr.clone(), cerulion_link::alpn::OPS),
        )
        .await
        .expect("dial huge");
        let (mut s_huge, mut r_huge) =
            common::bounded("open_bi huge", cerulion_link::open_frame_stream(&c_huge))
                .await
                .expect("open_bi huge");
        let huge_len = (cerud::protocol::MAX_FRAME_BYTES as u32) + 1;
        s_huge
            .write_all(&huge_len.to_be_bytes())
            .await
            .expect("write oversized length prefix");
        // The server closes the stream without sending any Ack bytes after
        // cerud rejects FrameTooLarge, without waiting for 16 MiB of payload.
        let mut buf = [0u8; 64];
        let huge_closed = matches!(
            tokio::time::timeout(Duration::from_secs(3), r_huge.read(&mut buf)).await,
            Ok(Ok(None)) | Ok(Err(_))
        );

        // 3. ReceiptSinkPoisoned on a bootstrap connection still closes with
        //    `ops receipt sink poisoned` after preloading a valid Hello frame.
        r.ops.poison_receipt_sink_for_test();
        let c_poison = common::bounded(
            "dial poisoned",
            cerulion_link::dial(&peer, robot_addr.clone(), cerulion_link::alpn::OPS),
        )
        .await
        .expect("dial poisoned");
        let (mut s_poison, _r_poison) = common::bounded(
            "open_bi poisoned",
            cerulion_link::open_frame_stream(&c_poison),
        )
        .await
        .expect("open_bi poisoned");
        let hello = serde_json::to_vec(&cerud::protocol::Hello::current()).unwrap();
        let hello_len = (hello.len() as u32).to_be_bytes();
        s_poison.write_all(&hello_len).await.expect("write len");
        s_poison.write_all(&hello).await.expect("write hello");
        let poisoned_reason = wait_app_close_reason(&c_poison, Duration::from_secs(3)).await;

        (eof_reason, huge_closed, poisoned_reason)
    })
    .await;

    assert_eq!(
        eof_reason.as_deref(),
        Some(b"ops bootstrap incomplete first frame".as_slice())
    );
    assert!(
        huge_closed,
        "oversized length prefix must be rejected promptly without waiting for payload"
    );
    assert_eq!(
        poisoned_reason.as_deref(),
        Some(b"ops receipt sink poisoned".as_slice())
    );
    assert!(
        read_all(&r.receipt_path).expect("read receipts").is_empty(),
        "no receipt was written by any rejected bootstrap error arm"
    );

    common::bounded("robot close", robot.close()).await;
    common::bounded("peer close", peer.close()).await;
}
