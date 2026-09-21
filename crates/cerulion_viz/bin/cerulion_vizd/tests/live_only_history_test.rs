// SPDX-License-Identifier: AGPL-3.0-only
//! HARD-GATE: the hosted gRPC proxy is LIVE-ONLY — a fresh viewer gets
//! the SCENE SKELETON (statics + blueprint) on connect but ZERO temporal replay.
//!
//! This drives a REAL hosted `re_grpc_server` message proxy through the EXACT
//! production options ([`cerulion_vizd::host::server_options`] — the Cerulion
//! fork's `drop_temporal_history` mode) + a REAL gRPC producer + REAL fresh gRPC
//! CONSUMERS (`re_grpc_client`), hermetically on loopback (no iceoryx2, no
//! daemon, no robot). It is the load-bearing pin for the decision:
//! "everything in Studio is instant-only — no backlog, no catch-up." Hand
//! oracles on FRAME IDENTITIES (the entity path / store kind each frame carries)
//! — never a self-compare.
//!
//! ## The skeleton vs temporal (the oracle)
//!
//! The producer logs, ONCE (as vizd does at worker startup):
//! - a RECORDING `SetStoreInfo` + a STATIC frame (`scene/axes`, `is_static`);
//! - a BLUEPRINT `SetStoreInfo` + a blueprint frame + a `BlueprintActivationCommand`.
//!
//! Then it streams PRE-connect temporal frames (`pre/<i>`), a viewer connects,
//! and it streams POST-connect (live) temporal frames (`live/<i>`).
//!
//! ## Arms
//!
//! - **(a) live-only (production options)** — a fresh viewer's per-client
//!   connect-history carries the statics + blueprint (+ its activation) and the
//!   LIVE `live/*` frames, but ZERO `pre/*` frames.
//! - **(b) anti-tautology (the OLD 1 GiB/OldestFirst defaults)** — the SAME flow
//!   REPLAYS the backlog: the fresh viewer DOES receive `pre/*`. Proves the
//!   harness detects replay, so (a)'s "no `pre/*`" is a real negative.
//! - **(c) clobber guard** — an EXISTING viewer's blueprint SURVIVES a new client
//!   connecting: NO `BlueprintActivationCommand` is broadcast to it after its
//!   initial connect (the heartbeat clobber this replaced is gone).
//! - **(d) no accumulation** — repeated fresh connects each receive the SAME
//!   (constant) count of skeleton markers (`SetStoreInfo` + activation): the
//!   proxy holds ONE copy, nothing accumulates.
//! - **(e) instant scene** — a fresh viewer receives the statics within a small
//!   bound of connecting (delivered in the connect-history, not after a wait).

use std::collections::BTreeSet;
use std::net::{Ipv4Addr, SocketAddr, TcpListener};
use std::time::{Duration, Instant};

use re_log_types::{
    BlueprintActivationCommand, LogMsg, SetStoreInfo, StoreId, StoreInfo, StoreKind, StoreSource,
};

fn probe_free_addr() -> SocketAddr {
    let l = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("probe a free loopback port");
    let addr = l.local_addr().expect("probe addr");
    drop(l);
    addr
}

fn proxy_uri(addr: SocketAddr) -> re_uri::ProxyUri {
    re_uri::ProxyUri::new(re_uri::Origin::from_scheme_and_socket_addr(
        re_uri::Scheme::RerunHttp,
        addr,
    ))
}

fn set_store_info(store_id: &StoreId) -> LogMsg {
    LogMsg::SetStoreInfo(SetStoreInfo {
        row_id: *re_chunk::RowId::new(),
        info: StoreInfo::new(
            store_id.clone(),
            StoreSource::RustSdk {
                rustc_version: String::new(),
                llvm_version: String::new(),
            },
        ),
    })
}

/// An `ArrowMsg` logging a trivial `Points2D` archetype to `entity` at `timepoint`.
fn arrow_msg(store_id: &StoreId, entity: &str, timepoint: re_log_types::TimePoint) -> LogMsg {
    let chunk = re_chunk::Chunk::builder(entity)
        .with_archetype(
            re_chunk::RowId::new(),
            timepoint,
            &rerun::archetypes::Points2D::new([(0.0, 0.0), (1.0, 1.0)]),
        )
        .build()
        .expect("build chunk");
    LogMsg::ArrowMsg(store_id.clone(), chunk.to_arrow_msg().expect("to arrow"))
}

fn static_frame(store_id: &StoreId, entity: &str) -> LogMsg {
    arrow_msg(store_id, entity, re_log_types::TimePoint::STATIC)
}

fn temporal_frame(store_id: &StoreId, entity: &str, seq: i64) -> LogMsg {
    let tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("frame"),
        re_log_types::TimeInt::new_temporal(seq),
    );
    arrow_msg(store_id, entity, tp)
}

/// The SCENE SKELETON: a recording store handshake + a static frame, a blueprint
/// store handshake + a blueprint frame + a `BlueprintActivationCommand`. All of
/// these are `persistent`/`static_` in re_grpc_server → retained + delivered
/// per-client, never dropped by drop-temporal.
fn skeleton(rec: &StoreId, bp: &StoreId) -> Vec<LogMsg> {
    let bp_tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("blueprint"),
        re_log_types::TimeInt::new_temporal(0),
    );
    vec![
        set_store_info(rec),
        static_frame(rec, "scene/axes"),
        set_store_info(bp),
        arrow_msg(bp, "blueprint/layout", bp_tp),
        LogMsg::BlueprintActivationCommand(BlueprintActivationCommand {
            blueprint_id: bp.clone(),
            make_active: true,
            make_default: true,
        }),
    ]
}

/// Tallied identities from a drained consumer stream.
#[derive(Default, Debug)]
struct Tally {
    store_infos: usize,
    blueprint_activations: usize,
    blueprint_arrows: usize,
    /// Recording STATIC entities (e.g. `scene/axes`).
    statics: BTreeSet<String>,
    /// Recording TEMPORAL entities (e.g. `pre/3` / `live/1`), leading `/` stripped.
    temporal: BTreeSet<String>,
}

fn tally(msgs: &[LogMsg]) -> Tally {
    let mut t = Tally::default();
    for m in msgs {
        match m {
            LogMsg::SetStoreInfo(_) => t.store_infos += 1,
            LogMsg::BlueprintActivationCommand(_) => t.blueprint_activations += 1,
            LogMsg::ArrowMsg(store_id, arrow) => {
                if store_id.kind() == StoreKind::Blueprint {
                    t.blueprint_arrows += 1;
                } else if let Ok(chunk) = re_chunk::Chunk::from_arrow_msg(arrow) {
                    // Entity paths render with a leading `/`; strip it so the keys
                    // match the names we logged.
                    let entity = chunk.entity_path().to_string();
                    let entity = entity.trim_start_matches('/').to_string();
                    if chunk.is_static() {
                        t.statics.insert(entity);
                    } else {
                        t.temporal.insert(entity);
                    }
                }
            }
        }
    }
    t
}

/// Drain a fresh consumer's stream: collect every `LogMsg` that arrives within
/// `total`, stopping early after `idle` of silence once at least one message has
/// landed. Robust to the async connect + subscribe lag.
fn drain(rx: &re_log_channel::LogReceiver, total: Duration, idle: Duration) -> Vec<LogMsg> {
    let mut out = Vec::new();
    let overall_deadline = Instant::now() + total;
    let mut last_recv: Option<Instant> = None;
    loop {
        if Instant::now() >= overall_deadline {
            break;
        }
        match rx.recv_timeout(Duration::from_millis(150)) {
            Ok(sm) => {
                if let Some(re_log_channel::DataSourceMessage::LogMsg(m)) = sm.into_data() {
                    out.push(m);
                }
                last_recv = Some(Instant::now());
            }
            Err(re_log_channel::RecvTimeoutError::Timeout) => {
                if let Some(t) = last_recv {
                    if t.elapsed() >= idle {
                        break;
                    }
                }
            }
            Err(re_log_channel::RecvTimeoutError::Disconnected) => break,
        }
    }
    out
}

fn host_proxy(options: rerun::ServerOptions) -> re_uri::ProxyUri {
    let addr = probe_free_addr();
    // The returned (LogReceiver, MessageProxyHandle) are the in-process viewer
    // feed + handle; we drive over gRPC, so leak them (kept alive by the server
    // task on the ambient runtime).
    let (rx, handle) =
        re_grpc_server::spawn_with_recv(addr, options, re_grpc_server::shutdown::never());
    std::mem::forget(rx);
    std::mem::forget(handle);
    proxy_uri(addr)
}

fn producer(uri: &re_uri::ProxyUri) -> re_grpc_client::Client {
    re_grpc_client::Client::new(uri.clone(), re_grpc_client::write::Options::default())
}

fn send(client: &re_grpc_client::Client, msgs: &[LogMsg]) {
    for m in msgs {
        client.send_blocking(m.clone());
    }
    client
        .flush_blocking(Duration::from_secs(10))
        .expect("producer flush");
}

/// Run the whole test body UNDER a multi-thread runtime (entered, not
/// `block_on`) so `re_grpc_server::spawn_with_recv` + `re_grpc_client::stream`'s
/// `tokio::spawn` find a runtime while the test thread stays free to block on
/// the drain.
fn with_runtime<R>(f: impl FnOnce() -> R) -> R {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .expect("build tokio runtime");
    let _guard = rt.enter();
    f()
}

/// (a)/(b): host with `options`, log the skeleton + PRE-connect temporal, connect
/// ONE fresh viewer, then stream LIVE temporal over several rounds (robust to the
/// consumer's async subscribe lag), and return the viewer's received tally.
fn fresh_viewer_tally(options: rerun::ServerOptions) -> Tally {
    let uri = host_proxy(options);
    let rec = StoreId::random(StoreKind::Recording, "hist");
    let bp = StoreId::random(StoreKind::Blueprint, "histbp");
    let prod = producer(&uri);

    // Skeleton (once) + a PRE-connect temporal backlog, NO viewer attached.
    send(&prod, &skeleton(&rec, &bp));
    let pre: Vec<LogMsg> = (0..12)
        .map(|i| temporal_frame(&rec, &format!("pre/{i}"), i))
        .collect();
    send(&prod, &pre);
    std::thread::sleep(Duration::from_millis(300));

    // A FRESH viewer connects (its connect-history is served here).
    let consumer = re_grpc_client::stream(uri.clone());

    // LIVE temporal from connect-time, over several rounds (the first may race
    // the subscription; later rounds always land).
    for round in 0..6 {
        send(
            &prod,
            &[temporal_frame(&rec, &format!("live/{round}"), 100 + round)],
        );
        std::thread::sleep(Duration::from_millis(150));
    }

    let received = drain(
        &consumer,
        Duration::from_secs(6),
        Duration::from_millis(800),
    );
    drop(prod);
    tally(&received)
}

#[test]
fn live_only_default_gives_a_fresh_viewer_the_skeleton_and_live_but_no_backlog() {
    with_runtime(|| {
        // The EXACT production options (the fork's drop-temporal mode).
        let t = fresh_viewer_tally(cerulion_vizd::host::server_options());

        // HEADLINE: ZERO pre-connect temporal replayed (no catch-up burst).
        let pre: BTreeSet<_> = t
            .temporal
            .iter()
            .filter(|e| e.starts_with("pre/"))
            .collect();
        assert!(
            pre.is_empty(),
            "live-only: a fresh viewer must receive NONE of the pre-connect temporal backlog, got {pre:?}"
        );
        // The SCENE SKELETON is delivered via the per-client connect-history.
        assert!(
            t.statics.contains("scene/axes"),
            "the fresh viewer gets the static scene frame, got {:?}",
            t.statics
        );
        assert!(
            t.blueprint_activations >= 1,
            "the fresh viewer gets the blueprint activation, got {}",
            t.blueprint_activations
        );
        assert!(
            t.blueprint_arrows >= 1,
            "the fresh viewer gets the blueprint data, got {}",
            t.blueprint_arrows
        );
        assert!(
            t.store_infos >= 2,
            "the fresh viewer gets both store handshakes (recording + blueprint), got {}",
            t.store_infos
        );
        // Live temporal flows from connect-time.
        let live: BTreeSet<_> = t
            .temporal
            .iter()
            .filter(|e| e.starts_with("live/"))
            .collect();
        assert!(
            !live.is_empty(),
            "the fresh viewer gets the LIVE temporal frames from connect-time, got {live:?}"
        );
    });
}

#[test]
fn anti_tautology_old_defaults_replay_the_pre_connect_backlog() {
    with_runtime(|| {
        // The OLD re_grpc_server defaults (1 GiB / OldestFirst, drop_temporal
        // OFF) DO retain + replay the pre-connect backlog. If this arm did NOT
        // see `pre/*`, the harness could not detect replay and the live-only arm
        // would be vacuous.
        let t = fresh_viewer_tally(rerun::ServerOptions::default());
        let pre: BTreeSet<_> = t
            .temporal
            .iter()
            .filter(|e| e.starts_with("pre/"))
            .collect();
        assert!(
            !pre.is_empty(),
            "anti-tautology: with the OLD defaults a fresh viewer REPLAYS the pre-connect backlog — the \
             harness can detect replay. Got no pre/* frames, meaning the pipe is broken."
        );
    });
}

#[test]
fn an_existing_viewers_blueprint_survives_a_new_client_connecting() {
    with_runtime(|| {
        // (c) The clobber regression guard: a new client's connect-history is
        // per-client (never broadcast), so an EXISTING viewer receives NO new
        // BlueprintActivationCommand when another viewer connects. The removed
        // heartbeat broadcast-re-sent the blueprint (make_active), clobbering
        // every viewer's layout every 2 s — this pins that it is gone.
        let uri = host_proxy(cerulion_vizd::host::server_options());
        let rec = StoreId::random(StoreKind::Recording, "histc");
        let bp = StoreId::random(StoreKind::Blueprint, "histcbp");
        let prod = producer(&uri);
        send(&prod, &skeleton(&rec, &bp));
        std::thread::sleep(Duration::from_millis(200));

        // Viewer 1 connects + drains its initial connect-history (skeleton).
        let c1 = re_grpc_client::stream(uri.clone());
        let first = drain(&c1, Duration::from_secs(4), Duration::from_millis(600));
        assert!(
            tally(&first).blueprint_activations >= 1,
            "viewer 1's initial connect-history carries the blueprint activation"
        );

        // Viewer 2 connects (gets its OWN connect-history).
        let _c2 = re_grpc_client::stream(uri.clone());
        std::thread::sleep(Duration::from_millis(500));

        // Viewer 1 must receive NOTHING more — crucially NO blueprint activation
        // (no broadcast re-send / clobber).
        let after = drain(&c1, Duration::from_secs(2), Duration::from_millis(600));
        let after_t = tally(&after);
        assert_eq!(
            after_t.blueprint_activations, 0,
            "an existing viewer must NOT be re-sent a BlueprintActivationCommand when a new client \
             connects (that would clobber its layout), got {after_t:?}"
        );
        drop(prod);
    });
}

#[test]
fn repeated_connects_do_not_accumulate_skeleton_markers() {
    with_runtime(|| {
        // (d) No accumulation: the proxy holds ONE copy of the skeleton, so every
        // fresh connect receives the SAME constant count of markers (2 store
        // handshakes + 1 blueprint activation) — never a growing pile.
        let uri = host_proxy(cerulion_vizd::host::server_options());
        let rec = StoreId::random(StoreKind::Recording, "histd");
        let bp = StoreId::random(StoreKind::Blueprint, "histdbp");
        let prod = producer(&uri);
        send(&prod, &skeleton(&rec, &bp));
        std::thread::sleep(Duration::from_millis(200));

        let mut counts = Vec::new();
        for _ in 0..3 {
            let c = re_grpc_client::stream(uri.clone());
            let t = tally(&drain(
                &c,
                Duration::from_secs(4),
                Duration::from_millis(500),
            ));
            counts.push((t.store_infos, t.blueprint_activations));
            drop(c);
            std::thread::sleep(Duration::from_millis(150));
        }
        // Hand oracle: every fresh connect sees EXACTLY (2 store infos, 1
        // activation) — identical across connects (no growth).
        assert!(
            counts.iter().all(|&c| c == (2, 1)),
            "every fresh connect must receive the same constant skeleton markers (2 store infos, 1 \
             activation) — no accumulation, got {counts:?}"
        );
        drop(prod);
    });
}

#[test]
fn a_fresh_viewer_gets_the_scene_instantly_not_after_a_wait() {
    with_runtime(|| {
        // (e) The scene skeleton is in the connect-history, delivered on connect —
        // NOT after a heartbeat wait. A fresh viewer receives the static frame
        // within a small bound of connecting (the removed heartbeat would have
        // left a ~2 s blank).
        let uri = host_proxy(cerulion_vizd::host::server_options());
        let rec = StoreId::random(StoreKind::Recording, "histe");
        let bp = StoreId::random(StoreKind::Blueprint, "histebp");
        let prod = producer(&uri);
        send(&prod, &skeleton(&rec, &bp));
        std::thread::sleep(Duration::from_millis(200));

        let connect_at = Instant::now();
        let c = re_grpc_client::stream(uri.clone());
        // Drain until the first static frame arrives (or a hard bound).
        let mut saw_static_at: Option<Instant> = None;
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut received = Vec::new();
        while Instant::now() < deadline && saw_static_at.is_none() {
            if let Ok(sm) = c.recv_timeout(Duration::from_millis(100)) {
                if let Some(re_log_channel::DataSourceMessage::LogMsg(m)) = sm.into_data() {
                    received.push(m);
                    if tally(&received).statics.contains("scene/axes") {
                        saw_static_at = Some(Instant::now());
                    }
                }
            }
        }
        let elapsed = saw_static_at
            .expect("the fresh viewer received the static scene frame from the connect-history")
            .duration_since(connect_at);
        assert!(
            elapsed < Duration::from_secs(2),
            "the scene skeleton must arrive on connect (not after a wait), took {elapsed:?}"
        );
        drop(prod);
    });
}

// ---------------------------------------------------------------------------
// The fork's superseded-blueprint eviction and
// entity-level static dedup + the #12721-class skeleton ORDER, proven at
// the VIZD level through the REAL patched proxy (`host::server_options`) + a
// real gRPC producer + a fresh gRPC viewer.
// ---------------------------------------------------------------------------

/// One runtime `set_blueprint`: a FRESH blueprint store handshake + one blueprint
/// chunk + its activation (the exact shape vizd emits when a Studio layout
/// changes). Each call mints a NEW blueprint store id.
fn blueprint_layout(name: &str) -> Vec<LogMsg> {
    let bp = StoreId::random(StoreKind::Blueprint, name);
    let bp_tp = re_log_types::TimePoint::default().with(
        re_log_types::Timeline::new_sequence("blueprint"),
        re_log_types::TimeInt::new_temporal(0),
    );
    vec![
        set_store_info(&bp),
        arrow_msg(&bp, "blueprint/layout", bp_tp),
        LogMsg::BlueprintActivationCommand(BlueprintActivationCommand {
            blueprint_id: bp,
            make_active: true,
            make_default: true,
        }),
    ]
}

/// Count the RECORDING static ArrowMsgs (`is_static` chunks) in a received stream.
fn count_recording_statics(msgs: &[LogMsg]) -> usize {
    msgs.iter()
        .filter(|m| {
            matches!(
                m,
                LogMsg::ArrowMsg(sid, arrow)
                    if sid.kind() == StoreKind::Recording
                        && re_chunk::Chunk::from_arrow_msg(arrow)
                            .map(|c| c.is_static())
                            .unwrap_or(false)
            )
        })
        .count()
}

/// Host the REAL patched proxy under production `server_options`, run `log` on a
/// producer with NO viewer attached (everything lands in the retained history),
/// then connect ONE fresh viewer and return its full received connect-history.
fn fresh_viewer_after(log: impl FnOnce(&re_grpc_client::Client)) -> Vec<LogMsg> {
    let uri = host_proxy(cerulion_vizd::host::server_options());
    let prod = producer(&uri);
    log(&prod);
    // Let the proxy's event loop process every send (incl. eviction and dedup) before
    // the viewer connects and is served its connect-history.
    std::thread::sleep(Duration::from_millis(300));
    let consumer = re_grpc_client::stream(uri.clone());
    let received = drain(
        &consumer,
        Duration::from_secs(6),
        Duration::from_millis(800),
    );
    drop(prod);
    received
}

/// (i) Blueprint eviction at the vizd level: N runtime `set_blueprint`s mid-session (each a FRESH
/// blueprint store) leave a fresh viewer with EXACTLY the latest blueprint store —
/// one activation + one chunk + one blueprint SetStoreInfo — plus the recording
/// SetStoreInfo. Without eviction the fresh viewer would replay ALL N.
#[test]
fn repeated_blueprint_sends_deliver_only_the_latest_to_a_fresh_viewer() {
    with_runtime(|| {
        const N: usize = 4;
        let rec = StoreId::random(StoreKind::Recording, "histf1");
        let received = fresh_viewer_after(|prod| {
            // A recording data store handshake — must survive blueprint churn.
            send(prod, &[set_store_info(&rec)]);
            // N layout changes, each minting a fresh blueprint store.
            for k in 0..N {
                send(prod, &blueprint_layout(&format!("histbp{k}")));
            }
        });
        let t = tally(&received);
        assert_eq!(
            t.blueprint_activations, 1,
            "fresh viewer must get exactly ONE (latest) blueprint activation, not {N}; got {}",
            t.blueprint_activations
        );
        assert_eq!(
            t.blueprint_arrows, 1,
            "fresh viewer must get exactly ONE (latest) blueprint chunk; got {}",
            t.blueprint_arrows
        );
        // Hand oracle: 1 recording SetStoreInfo + exactly 1 surviving blueprint
        // SetStoreInfo == 2 (without eviction: 1 + N).
        assert_eq!(
            t.store_infos, 2,
            "fresh viewer must get the recording SetStoreInfo + ONE blueprint SetStoreInfo; got {}",
            t.store_infos
        );
    });
}

/// (ii) Static dedup at the vizd level: a static re-logged for the SAME entity reaches a
/// fresh viewer ONCE (latest-wins), while a DISTINCT entity is retained
/// alongside. With plain append the same-entity re-logs would all replay.
#[test]
fn re_logged_statics_for_one_entity_reach_a_fresh_viewer_once() {
    with_runtime(|| {
        let rec = StoreId::random(StoreKind::Recording, "histf2");
        let received = fresh_viewer_after(|prod| {
            send(prod, &[set_store_info(&rec)]);
            // Same entity re-logged 3x (the reconnect re-log shape)...
            for _ in 0..3 {
                send(prod, &[static_frame(&rec, "scene/robot")]);
            }
            // ...plus a DISTINCT entity once.
            send(prod, &[static_frame(&rec, "scene/arm")]);
        });
        let t = tally(&received);
        assert_eq!(
            t.statics,
            BTreeSet::from(["scene/robot".to_string(), "scene/arm".to_string()]),
            "both distinct static entities present: {:?}",
            t.statics
        );
        assert_eq!(
            count_recording_statics(&received),
            2,
            "the 3 same-entity re-logs must dedup to 1 (+ the distinct entity) = 2 static frames"
        );
    });
}

/// (iii) The #12721-class skeleton ORDER under OldestFirst (B6): a fresh viewer
/// sees SetStoreInfo(blueprint B) BEFORE B's chunk BEFORE B's activation.
/// NewestFirst (pre-B6) reverses the persistent queue, delivering the activation
/// before its store info — breaking late-joiner skeleton delivery.
#[test]
fn a_fresh_viewers_blueprint_skeleton_arrives_in_activation_order() {
    with_runtime(|| {
        let received = fresh_viewer_after(|prod| {
            send(prod, &blueprint_layout("historder"));
        });
        let pos_info = received
            .iter()
            .position(|m| {
                matches!(m, LogMsg::SetStoreInfo(s) if s.info.store_id.kind() == StoreKind::Blueprint)
            })
            .expect("blueprint SetStoreInfo present");
        let pos_arrow = received
            .iter()
            .position(
                |m| matches!(m, LogMsg::ArrowMsg(sid, _) if sid.kind() == StoreKind::Blueprint),
            )
            .expect("blueprint chunk present");
        let pos_act = received
            .iter()
            .position(|m| matches!(m, LogMsg::BlueprintActivationCommand(_)))
            .expect("blueprint activation present");
        assert!(
            pos_info < pos_arrow && pos_arrow < pos_act,
            "skeleton order must be SetStoreInfo({pos_info}) < chunk({pos_arrow}) < activation({pos_act})"
        );
    });
}
