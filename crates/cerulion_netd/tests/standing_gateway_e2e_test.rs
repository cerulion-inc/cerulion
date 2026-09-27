// SPDX-License-Identifier: AGPL-3.0-only
//! Gateway existence WITHOUT a graph: the STANDING gateway a
//! LISTEN-configured machine boots at daemon start.
//!
//! Before the standing gateway, netd's embedded egress gateway booted ONLY inside `register_egress`
//! (which the daemon refuses for an empty announce set), and the mDNS beacon was
//! raised only in that boot — so a pure-ROS robot (rmw publishers + netd, no
//! graph run ever) had no zenoh session, no announce, no catalog and no beacon,
//! whatever `CERULION_NETD_LISTEN` said. Nine arms, each against a hand oracle:
//!
//! 1. `boot_standing_gateway_boots_with_no_egress_registration` — over REAL
//!    iceoryx2 + zenoh (the `egress_plane_iox2_test` harness, beacon advertise
//!    SUPPRESSED so nothing reaches the LAN): a robot-shaped plane is UNBOOTED
//!    (the earlier premise, asserted, not assumed), `boot_standing_gateway`
//!    boots it with NO plan registered, the beacon DECISION is `Advertise` on
//!    the bound port (the session opened, so the port-bound precondition held),
//!    and a later `register_egress` JOINS the running gateway (`Ok(false)`) and
//!    still lands its topic.
//! 2. `a_standing_daemon_never_idle_self_exits_and_the_default_pair_does` — the
//!    LIFECYCLE rule: `idle_self_exit: false` leaves the daemon up across many
//!    grace periods with its socket bound; the DEFAULT config (the anti-tautology
//!    pair, byte-identical to today) self-exits after one grace.
//! 3. `real_binary_local_only_wins_over_listen_and_still_idle_exits` — over the
//!    REAL binary: `CERULION_NETD_LISTEN` set AND `CERULION_NETD_NETWORK=off`
//!    ⇒ the resolved config is local-only, nothing boots, and the daemon still
//!    idle-self-exits (exit 0, socket removed). Hermetic — no session, no mDNS.
//!    (A real-binary LISTEN-set arm WITHOUT the kill switch is deliberately
//!    absent: the shipped binary has no beacon-suppression seam, so it would
//!    publish a genuine `_cerulion._tcp` record on the test machine's LAN.)
//! 4. `main_rs_wires_the_standing_boot_inside_the_gate_and_the_idle_exit_flag` —
//!    the binary's `main()` cannot be driven in-process, so its wiring is pinned
//!    STRUCTURALLY over a comment-stripped view: the ONE `boot_standing_gateway`
//!    call must sit INSIDE the `if standing_gateway { … }` block that follows the
//!    `standing_gateway_requested` gate binding (never before the gate, never
//!    outside the block), and `idle_self_exit` must derive from that same binding.
//! 5. `the_wiring_checker_accepts_only_a_call_inside_the_gates_block` — the
//!    checker's own hand-written oracle, including the exact variant arm 4 exists to
//!    catch: the call hoisted out of the block with every substring intact.
//! 6. `a_custom_registration_after_the_standing_boot_backfills_the_serving` —
//!    the backfill shape: a gateway the standing boot brought up with no
//!    custom serving, then a `register_egress` carrying a custom type ⇒ the
//!    topic catalogs by NAME and the doc serves over the REAL remote query
//!    surface (without the merge seam the boot serving is frozen: `schema_name: None`, zero
//!    docs, forever).
//! 7. `a_first_registration_boot_with_serving_resolves` — the control: the SAME
//!    serving handed at the boot registration resolves identically (proves the
//!    merge did not displace the boot-time path).
//! 8. `a_rebooted_gateway_retains_every_previously_registered_serving` — the
//!    replacement race class made deterministic: a gateway replaced after a
//!    drive-thread death (the fault seam) re-seeds from the plane's ACCUMULATED
//!    serving, so a type registered before the death still resolves after it.
//!    The threaded race itself has no clean seam (the replacement window closed
//!    STRUCTURALLY — the merge happens under the same slot-lock acquisition
//!    that resolved the gateway, inside `ensure_gateway_booted`), so this arm
//!    pins its HARM class on the deterministic replacement path instead.
//! 9. `a_partial_boot_does_not_strand_the_query_surface_on_dropped_maps` — the
//!    partial-boot shape: a boot that starts (and latches)
//!    the query surface and then FAILS leaves the surface reading the dropped
//!    construction's maps; the successful retry must bind `callback_serving` to
//!    the SURFACE's recorded captures (never the retry gateway's own handles)
//!    or every later custom schema lands in maps nobody serves.
//!
//! Parallel-safe: per-test SHM roots + probed ports + a unique temp socket per
//! daemon; the one real-binary arm shares the default iceoryx2 namespace with
//! nothing else in this binary.

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::discovery::{query_robot_catalog, query_robot_schema};
use cerulion_core::transport::network::{NetworkConfig, NetworkManager};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, SchemaDoc, SchemaEncoding, SchemaHashName, SchemaServing,
    TopicSchema,
};
use cerulion_netd::beacon::BeaconDecision;
use cerulion_netd::daemon::{self, NetdConfig};
use cerulion_netd::egress::{EgressPlane, GatewayEgressPlane};
use cerulion_netd::mirror::{MirrorError, MirrorPlane, MirrorRelease};
use cerulion_netd::registry::TopicKey;

fn unique_id() -> String {
    static N: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

fn probe_ephemeral_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral")
        .local_addr()
        .expect("local_addr")
        .port()
}

fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    f()
}

/// A robot-shaped shared manager G (an identity plus a `tcp/` listen locator —
/// the shape `CERULION_NETD_LISTEN` produces) wrapped in a plane whose beacon
/// advertise is SUPPRESSED (the DECISION path is byte-identical; only the mDNS
/// socket is not opened — see `GatewayEgressPlane::new_without_mdns_for_test`).
fn robot_shaped_plane(tag: &str, id: &str, attempt: usize) -> (Arc<GatewayEgressPlane>, u16) {
    let root = iceoryx_test_config();
    let port = probe_ephemeral_port();
    let g = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("sgw_g_{tag}_{id}_{attempt}"),
            network: Some(NetworkConfig {
                listen_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                robot_identity: Some("sgwrobot".to_string()),
                ..Default::default()
            }),
            ..Default::default()
        },
        root,
    )
    .expect("init shared gateway G");
    (
        Arc::new(GatewayEgressPlane::new_without_mdns_for_test(g)),
        port,
    )
}

/// Arm 1: the standing boot boots the gateway with NO egress registration, the
/// beacon decision is `Advertise` on the bound port, and a later `register_egress`
/// joins rather than re-boots.
#[test]
fn boot_standing_gateway_boots_with_no_egress_registration() {
    let id = unique_id();
    let topic = format!("/sgw/later/{id}");
    for attempt in 0..3 {
        let (plane, port) = robot_shaped_plane("boot", &id, attempt);

        // The earlier PREMISE, asserted: a plane with no egress plan registered
        // is unbooted, and the beacon has not even been consulted.
        assert!(
            !plane.gateway_running(),
            "no plan registered ⇒ no gateway (the lazy pre-1541 posture)"
        );
        assert_eq!(
            plane.mdns_beacon_decision(),
            None,
            "the beacon is consulted only by a gateway boot"
        );

        // A bind steal (probe→rebind race) surfaces here → retry on a fresh port.
        match plane.boot_standing_gateway(&SchemaServing::default()) {
            Ok(booted) => {
                assert!(booted, "the standing boot is the FIRST boot on this plane");
            }
            Err(e) => {
                eprintln!("attempt {attempt}: standing boot failed (port {port}): {e}");
                continue;
            }
        }
        assert!(
            plane.gateway_running(),
            "the standing boot started the gateway drive thread with NO plan registered"
        );
        // The beacon's port-bound precondition held: `new_embedded` opened the
        // session (binding the listen locator) BEFORE the beacon was consulted.
        assert_eq!(
            plane.mdns_beacon_decision(),
            Some(BeaconDecision::Advertise {
                robot: "sgwrobot".to_string(),
                port,
            }),
            "a LISTEN-configured plane decides Advertise on its bound port at the start boot"
        );

        // A LATER egress plan JOINS the running gateway (Ok(false)) — it does not
        // re-boot — and its topic still reaches the shared bridge manager via the
        // reg-channel the start-booted gateway is already draining.
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        let joined = plane
            .register_egress(0, &plan, &SchemaServing::default(), None)
            .expect("register_egress after the standing boot");
        assert!(
            !joined,
            "register_egress after a standing boot joins the running gateway (Ok(false))"
        );
        let landed = wait_until(
            || {
                plane
                    .manager()
                    .bridge_manager()
                    .is_registered(&topic)
                    .expect("read the registered set")
            },
            Duration::from_secs(20),
        );
        assert!(
            landed,
            "the plan's topic must reach the start-booted gateway over the reg-channel"
        );
        return;
    }
    panic!("could not boot the standing gateway in 3 attempts");
}

/// A mirror plane that is never demanded — the lifecycle arm needs a daemon, not
/// a mirror.
struct NoopMirrorPlane;

impl MirrorPlane for NoopMirrorPlane {
    fn ensure_mirror(&self, _key: &TopicKey, _schema_hash: u64) -> Result<(), MirrorError> {
        Ok(())
    }
    fn release_mirror(&self, _key: &TopicKey) -> MirrorRelease {
        MirrorRelease::Retired
    }
}

fn unique_socket(tag: &str) -> (PathBuf, PathBuf) {
    let dir = std::env::temp_dir().join(format!("cer_netd_sgw_{tag}_{}", unique_id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    (dir.clone(), dir.join("netd.sock"))
}

/// Arm 2: the lifecycle rule and its anti-tautology pair.
#[test]
fn a_standing_daemon_never_idle_self_exits_and_the_default_pair_does() {
    let grace = Duration::from_millis(150);
    let poll = Duration::from_millis(25);

    // STANDING: idle self-exit disabled ⇒ no self-exit across many graces, and
    // the control socket stays bound (the idle-watch is what unlinks it).
    let (dir, sock) = unique_socket("standing");
    let netd = daemon::start(
        sock.clone(),
        Arc::new(NoopMirrorPlane),
        NetdConfig {
            idle_grace: grace,
            idle_watch_poll: poll,
            idle_self_exit: false,
            ..NetdConfig::default()
        },
    )
    .expect("daemon start (standing)");
    std::thread::sleep(grace * 8);
    assert!(
        !netd.self_exit_requested(),
        "a standing daemon (idle_self_exit: false) must never commit self-exit"
    );
    assert!(
        sock.exists(),
        "a standing daemon keeps its control socket bound (nothing unlinks it)"
    );
    assert!(
        netd.is_idle(),
        "it IS idle by the registry's account — the rule holds it open anyway"
    );
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);

    // THE PAIR (byte-identical to today): the default config self-exits after one
    // grace. Without this arm a daemon that ignored the flag in BOTH directions
    // (never exiting) would pass the standing arm.
    let (dir, sock) = unique_socket("default");
    let netd = daemon::start(
        sock.clone(),
        Arc::new(NoopMirrorPlane),
        NetdConfig {
            idle_grace: grace,
            idle_watch_poll: poll,
            ..NetdConfig::default()
        },
    )
    .expect("daemon start (default)");
    assert!(
        NetdConfig::default().idle_self_exit,
        "the DEFAULT is the self-exit lifecycle (a desk is byte-identical to today)"
    );
    assert!(
        wait_until(|| netd.self_exit_requested(), Duration::from_secs(3)),
        "the default daemon self-exits after the idle grace"
    );
    assert!(
        wait_until(|| !sock.exists(), Duration::from_secs(3)),
        "the self-exit commit unlinks the socket"
    );
    drop(netd);
    let _ = std::fs::remove_dir_all(&dir);
}

/// Arm 3: over the REAL binary, the kill switch wins — LISTEN set + local-only ⇒
/// nothing boots and the daemon idle-self-exits exactly as today.
#[test]
fn real_binary_local_only_wins_over_listen_and_still_idle_exits() {
    let (dir, sock) = unique_socket("bin");
    let mut child = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"))
        .env("CERULION_HOME", dir.join("empty-login"))
        .env("CERULION_NETD_SOCK", &sock)
        .env("CERULION_NETD_LISTEN", "tcp/127.0.0.1:0")
        .env("CERULION_NETD_NETWORK", "off")
        .env("CERULION_NETD_IDLE_GRACE_MS", "300")
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .expect("spawn cerulion-netd");
    let booted = wait_until(|| sock.exists(), Duration::from_secs(8));
    if !booted {
        let _ = child.kill();
        let _ = child.wait();
        let _ = std::fs::remove_dir_all(&dir);
        panic!("the real daemon never bound its control socket");
    }
    let exited = wait_until(
        || matches!(child.try_wait(), Ok(Some(_))),
        Duration::from_secs(10),
    );
    if !exited {
        let _ = child.kill();
        let _ = child.wait();
        let _ = std::fs::remove_dir_all(&dir);
        panic!(
            "LISTEN set + CERULION_NETD_NETWORK=off must still idle-self-exit — the kill switch \
             resolves the config to local-only, so no standing gateway (and no standing \
             lifecycle) applies"
        );
    }
    let status = child.wait().expect("reap");
    assert_eq!(status.code(), Some(0), "a clean idle self-exit returns 0");
    assert!(!sock.exists(), "the self-exit unlinked the socket");
    let _ = std::fs::remove_dir_all(&dir);
}

/// The custom-type serving a graph registration hands: one topic→name binding,
/// one hash→name binding, one served doc — the desk-side trio a custom type
/// needs to catalog by name and resolve. Hand-built (any hash exercises the
/// path; the schema fetch is by NAME).
const WIDGET_Q: &str = "probe_msgs/Widget";
const WIDGET_HASH: u64 = 0x1541_0002_0000_0001;
const WIDGET_TEXT: &str = "int32 count\nfloat64 v\n";

fn widget_serving(topic: &str) -> SchemaServing {
    SchemaServing {
        topic_schemas: vec![TopicSchema {
            topic: topic.to_string(),
            schema_name: WIDGET_Q.to_string(),
        }],
        schema_docs: vec![SchemaDoc {
            qualified: WIDGET_Q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: WIDGET_TEXT.to_string(),
            deps: vec![],
        }],
        schema_hashes: vec![SchemaHashName {
            schema_hash: WIDGET_HASH,
            qualified: WIDGET_Q.to_string(),
        }],
    }
}

/// Poll the plane's REAL remote query surface (a desk session over loopback TCP)
/// until `topic` catalogs with the `q` name AND the `q` doc serves with `text`
/// — then assert against the hand oracle. Bounded; panics with the LAST
/// observed state on timeout.
///
/// `expect_hash`: `Some` asserts the catalog row's hash column; `None` tolerates
/// it — used ONLY for a topic registered AFTER a gateway re-boot, whose
/// reg-channel hash lands in the REPLACEMENT gateway's table while the LATCHED
/// query-surface callback reads the first boot's (a pre-existing re-boot
/// residual, documented on `GatewayEgressPlane::callback_serving`; NAME + doc —
/// what a desk consumer needs — are what this helper always pins).
fn assert_type_resolves_remotely(
    port: u16,
    topic: &str,
    q: &str,
    expect_hash: Option<u64>,
    text: &str,
) {
    let desk = NetworkManager::new(NetworkConfig {
        connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
        ..NetworkConfig::default()
    });
    let session = desk.session().expect("desk session");
    let mut last_name: Option<String> = None;
    for _ in 0..40 {
        if let Some(catalog) = query_robot_catalog(session, "sgwrobot", Duration::from_millis(300))
        {
            if let Some(e) = catalog.entries.iter().find(|e| e.topic == topic) {
                last_name = e.schema_name.clone();
                if e.schema_name.as_deref() == Some(q) {
                    if let Some(hash) = expect_hash {
                        assert_eq!(
                            e.schema_hash,
                            Some(hash),
                            "the catalog carries the registration's hash beside the name"
                        );
                    }
                    // The doc serves too — the desk's `SchemaUnavailable` needs BOTH.
                    let reply =
                        query_robot_schema(session, "sgwrobot", q, Duration::from_millis(300))
                            .expect("schema reply");
                    assert_eq!(reply.docs.len(), 1, "the {q} doc serves");
                    assert_eq!(reply.docs[0].qualified, q);
                    assert_eq!(reply.docs[0].text, text, "verbatim text");
                    return;
                }
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    panic!(
        "'{topic}' never catalogued as {q} over the remote query surface \
         (last observed schema_name: {last_name:?} — the frozen-boot-serving defect)"
    );
}

/// The Widget shape at its own oracle values (hash asserted).
fn assert_widget_resolves_remotely(port: u16, topic: &str) {
    assert_type_resolves_remotely(port, topic, WIDGET_Q, Some(WIDGET_HASH), WIDGET_TEXT);
}

/// Arm 6 (the backfill shape): the standing boot brings the gateway up with
/// NO custom serving; a LATER `register_egress` carries a custom type. Without the merge
/// the boot serving is FROZEN — the topic catalogues `schema_name: None` with
/// zero served docs forever (desk: `SchemaUnavailable`); the merge seam is what
/// this pins. Dropping the `g.serving.merge(serving)` call in
/// `register_egress` fails this arm with `schema_name: None`.
#[test]
fn a_custom_registration_after_the_standing_boot_backfills_the_serving() {
    let id = unique_id();
    let topic = format!("/sgw/backfill/{id}");
    for attempt in 0..3 {
        let (plane, port) = robot_shaped_plane("backfill", &id, attempt);
        // The standing boot: NO custom serving (production hands the built-in
        // corpus, which contains no custom either — an empty serving makes the
        // merge the ONLY possible source of the Widget bindings, the sharper pin).
        match plane.boot_standing_gateway(&SchemaServing::default()) {
            Ok(booted) => assert!(booted, "the standing boot is first"),
            Err(e) => {
                eprintln!("attempt {attempt}: standing boot failed (port {port}): {e}");
                continue;
            }
        }
        // The later registration: a graph pushes its plan + CUSTOM serving.
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        let joined = plane
            .register_egress(0, &plan, &widget_serving(&topic), None)
            .expect("register_egress after the standing boot");
        assert!(!joined, "joins the standing gateway — no re-boot");
        assert_widget_resolves_remotely(port, &topic);
        return;
    }
    panic!("could not boot the standing gateway in 3 attempts");
}

/// Arm 7 (the control for arm 6): the same serving handed at the boot
/// registration resolves identically — the merge seam did not displace the
/// boot-time path, and the two boot orders now compose to the same catalog.
#[test]
fn a_first_registration_boot_with_serving_resolves() {
    let id = unique_id();
    let topic = format!("/sgw/control/{id}");
    for attempt in 0..3 {
        let (plane, port) = robot_shaped_plane("control", &id, attempt);
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![topic.clone()],
            ingress: vec![],
        };
        match plane.register_egress(0, &plan, &widget_serving(&topic), None) {
            Ok(booted) => assert!(booted, "the first registration boots"),
            Err(e) => {
                eprintln!("attempt {attempt}: boot failed (port {port}): {e}");
                continue;
            }
        }
        assert_widget_resolves_remotely(port, &topic);
        return;
    }
    panic!("could not boot the gateway in 3 attempts");
}

/// Arm 8 (the replacement race class, made deterministic): a gateway replaced
/// after a drive-thread death re-seeds from the plane's ACCUMULATED serving, so
/// a type registered BEFORE the death still catalogs + serves AFTER it.
///
/// The threaded window itself is closed STRUCTURALLY — the merge happens
/// inside `ensure_gateway_booted`, under the same slot-lock acquisition that
/// resolved (or replaced) the gateway, so there is no boot-resolution→merge gap
/// for a concurrent replacement to land in and no clean seam to schedule one
/// through. What remains OBSERVABLE is the race's harm class: a replacement
/// gateway that forgot previously-merged servings. The fault seam
/// (`fault_inject_drive_failure_for_test`) makes exactly that replacement happen
/// deterministically. Dropping the re-boot FOLD
/// (`Some(first) => first.merge(&accumulated)` in `ensure_gateway_booted`) fails
/// this arm with the Gizmo's `schema_name: None` — the registration that CAUSED
/// the re-boot takes the boot arm, so the fold is the only path its serving has
/// into the maps the latched callback reads. (The Widget survives the death via
/// the callback handles themselves, which outlive the gateway.)
#[test]
fn a_rebooted_gateway_retains_every_previously_registered_serving() {
    const GIZMO_Q: &str = "probe_msgs/Gizmo";
    const GIZMO_HASH: u64 = 0x1541_0002_0000_0002;
    const GIZMO_TEXT: &str = "float64 spin\n";
    let id = unique_id();
    let widget_topic = format!("/sgw/reboot/widget/{id}");
    let gizmo_topic = format!("/sgw/reboot/gizmo/{id}");
    for attempt in 0..3 {
        let (plane, port) = robot_shaped_plane("reboot", &id, attempt);
        // Registration 1 boots the gateway and carries the Widget serving.
        let plan_a = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![widget_topic.clone()],
            ingress: vec![],
        };
        match plane.register_egress(0, &plan_a, &widget_serving(&widget_topic), None) {
            Ok(booted) => assert!(booted, "the first registration boots"),
            Err(e) => {
                eprintln!("attempt {attempt}: boot failed (port {port}): {e}");
                continue;
            }
        }
        // PREMISE: the Widget resolves on the FIRST gateway (so a post-reboot
        // failure below is attributable to the replacement, not to the serving
        // never having worked).
        assert_widget_resolves_remotely(port, &widget_topic);

        // Kill the drive thread (the crash → dead → re-boot path) and wait for
        // the death to be RECORDED.
        plane.fault_inject_drive_failure_for_test();
        assert!(
            wait_until(
                || plane.gateway_drive_died_for_test(),
                Duration::from_secs(3)
            ),
            "the faulted drive thread marks the slot dead"
        );

        // Registration 2 REPLACES the dead gateway, carrying only the Gizmo
        // serving. The replacement must seed from the ACCUMULATOR — this
        // caller's serving alone knows nothing of the Widget.
        let plan_b = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![gizmo_topic.clone()],
            ingress: vec![],
        };
        let gizmo_serving = SchemaServing {
            topic_schemas: vec![TopicSchema {
                topic: gizmo_topic.clone(),
                schema_name: GIZMO_Q.to_string(),
            }],
            schema_docs: vec![SchemaDoc {
                qualified: GIZMO_Q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: GIZMO_TEXT.to_string(),
                deps: vec![],
            }],
            schema_hashes: vec![SchemaHashName {
                schema_hash: GIZMO_HASH,
                qualified: GIZMO_Q.to_string(),
            }],
        };
        let rebooted = plane
            .register_egress(1, &plan_b, &gizmo_serving, None)
            .expect("re-register after the drive thread died");
        assert!(rebooted, "a dead gateway RE-BOOTS on the next registration");

        // THE PIN: BOTH types resolve on the replacement gateway — the Gizmo
        // (this registration's serving, reaching the latched callback's maps
        // ONLY through the re-boot fold) and the Widget (registered before the
        // death). The Gizmo's HASH column is deliberately not asserted: its
        // reg-channel hash lands in the replacement gateway's table, which the
        // LATCHED callback does not read — a pre-existing re-boot residual
        // (documented on `callback_serving`), orthogonal to the serving merge;
        // name + doc are what a desk consumer needs.
        assert_type_resolves_remotely(port, &gizmo_topic, GIZMO_Q, None, GIZMO_TEXT);
        assert_widget_resolves_remotely(port, &widget_topic);
        return;
    }
    panic!("could not boot the gateway in 3 attempts");
}

/// Arm 9 (the partial-boot shape): a
/// boot whose construction STARTS the query surface and then FAILS leaves the
/// LATCHED surface reading the dropped construction's maps. The retry must bind
/// the plane's `callback_serving` to the SURFACE's recorded captures
/// (`NetworkManager::query_surface_serving_handles`), never to the retry
/// gateway's own handles — those name maps the callback never reads, so every
/// later custom schema would land where nobody serves it, despite the retry
/// "succeeding". Recording `gateway.schema_serving_handles()` instead
/// of the surface's captures fails BOTH resolves below with
/// `schema_name: None`.
///
/// The Widget's HASH column is not asserted: after the partial boot, reg-channel
/// hashes drain into the RETRY construction's table while the latched callback
/// reads the failed construction's — the same pre-existing residual arm 8
/// documents (see `callback_serving`); name + doc are what a desk needs.
#[test]
fn a_partial_boot_does_not_strand_the_query_surface_on_dropped_maps() {
    const POST_Q: &str = "probe_msgs/PostRetry";
    const POST_HASH: u64 = 0x1541_0002_0000_0003;
    const POST_TEXT: &str = "bool armed\n";
    let id = unique_id();
    let widget_topic = format!("/sgw/partial/widget/{id}");
    let post_topic = format!("/sgw/partial/post/{id}");
    for attempt in 0..3 {
        let (plane, port) = robot_shaped_plane("partial", &id, attempt);
        // The PARTIAL boot: the construction starts (and latches) the query
        // surface, then the boot fails (one-shot seam). The gateway drops; the
        // surface keeps its maps.
        plane.fault_inject_boot_failure_after_surface_for_test();
        let plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![widget_topic.clone()],
            ingress: vec![],
        };
        let err = plane
            .register_egress(0, &plan, &widget_serving(&widget_topic), None)
            .expect_err("the faulted boot must FAIL (the partial-boot premise)");
        assert!(
            err.to_string()
                .contains("boot fault after the query surface"),
            "the failure is the injected partial-boot fault, not a bind error: {err}"
        );
        assert!(
            !plane.gateway_running(),
            "a failed boot leaves no gateway (the retry re-boots)"
        );

        // The RETRY (same registration, fault consumed) succeeds...
        match plane.register_egress(0, &plan, &widget_serving(&widget_topic), None) {
            Ok(booted) => assert!(booted, "the retry boots the gateway"),
            Err(e) => {
                // A genuine bind race on this port — rebuild the whole fixture.
                eprintln!("attempt {attempt}: retry boot failed (port {port}): {e}");
                continue;
            }
        }
        // ...and the Widget resolves over the surface the FAILED construction
        // latched — reachable only through the surface's own recorded captures.
        assert_type_resolves_remotely(port, &widget_topic, WIDGET_Q, None, WIDGET_TEXT);

        // A LATER registration (the alive arm) must land in the same true maps —
        // "later custom schemas never appear" is the MAJOR's harm.
        let post_plan = GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowAll,
            announce: vec![post_topic.clone()],
            ingress: vec![],
        };
        let post_serving = SchemaServing {
            topic_schemas: vec![TopicSchema {
                topic: post_topic.clone(),
                schema_name: POST_Q.to_string(),
            }],
            schema_docs: vec![SchemaDoc {
                qualified: POST_Q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: POST_TEXT.to_string(),
                deps: vec![],
            }],
            schema_hashes: vec![SchemaHashName {
                schema_hash: POST_HASH,
                qualified: POST_Q.to_string(),
            }],
        };
        let joined = plane
            .register_egress(1, &post_plan, &post_serving, None)
            .expect("a later registration joins the retried gateway");
        assert!(!joined, "joins — no re-boot");
        assert_type_resolves_remotely(port, &post_topic, POST_Q, None, POST_TEXT);
        return;
    }
    panic!("could not complete the partial-boot fixture in 3 attempts");
}

/// Strip `//` line comments and (nesting) `/* */` block comments so a mention in
/// prose cannot satisfy — or a commented-out call cannot dodge — the wiring walk.
fn code_only(src: &str) -> String {
    let bytes = src.as_bytes();
    let mut out = String::with_capacity(src.len());
    let mut i = 0usize;
    let mut depth = 0usize;
    while i < bytes.len() {
        if depth == 0 && bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'/') {
            while i < bytes.len() && bytes[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if bytes[i] == b'/' && bytes.get(i + 1) == Some(&b'*') {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && bytes[i] == b'*' && bytes.get(i + 1) == Some(&b'/') {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            out.push(bytes[i] as char);
        }
        i += 1;
    }
    assert_eq!(
        depth, 0,
        "unterminated block comment — the walk fails CLOSED"
    );
    out
}

/// The brace-balanced block that follows `open` (the byte index of a `{`):
/// returns the inner text, or `None` if the braces never balance. Runs over a
/// comment-stripped view, so a brace in a comment cannot skew the count (string
/// literals with braces would — none of the wiring under test carries one).
fn brace_block(code: &str, open: usize) -> Option<&str> {
    let bytes = code.as_bytes();
    assert_eq!(bytes[open], b'{', "brace_block must start at a `{{`");
    let mut depth = 0usize;
    for (i, b) in bytes.iter().enumerate().skip(open) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&code[open + 1..i]);
                }
            }
            _ => {}
        }
    }
    None
}

/// The chain `main()` must wire, located STRUCTURALLY (byte offsets over a
/// comment-stripped view), so "the strings are all there somewhere"
/// cannot satisfy it. Pure over the source text — the mutation arm below drives it
/// with hand-written shapes.
struct WiringChain {
    /// Offset of the gate binding `let standing_gateway = net::standing_gateway_requested(`.
    gate: usize,
    /// Offset of the `if standing_gateway {` that follows the gate.
    if_open: usize,
    /// The brace-balanced body of that `if`.
    block: String,
}

fn locate_chain(code: &str) -> Result<WiringChain, String> {
    let gate_needle = "let standing_gateway = net::standing_gateway_requested(network.as_ref())";
    let gate = code
        .find(gate_needle)
        .ok_or_else(|| format!("no gate binding {gate_needle:?}"))?;
    let if_needle = "if standing_gateway {";
    let if_rel = code[gate..]
        .find(if_needle)
        .ok_or_else(|| format!("no {if_needle:?} after the gate binding"))?;
    let if_open = gate + if_rel + if_needle.len() - 1;
    let block = brace_block(code, if_open)
        .ok_or_else(|| "the `if standing_gateway` block never balances".to_string())?
        .to_string();
    Ok(WiringChain {
        gate,
        if_open,
        block,
    })
}

/// The RELATIONSHIP, not the substrings: exactly one `.boot_standing_gateway(` call
/// in the file, and it sits INSIDE the `if standing_gateway { … }` block that
/// follows the gate binding — never before the gate, never after the block; and
/// the lifecycle flag derives from the SAME binding, after it.
fn check_chain(code: &str) -> Result<(), String> {
    let chain = locate_chain(code)?;
    let boot = ".boot_standing_gateway(";
    let calls: Vec<usize> = code.match_indices(boot).map(|(i, _)| i).collect();
    if calls.len() != 1 {
        return Err(format!(
            "expected exactly ONE {boot:?} call, found {}",
            calls.len()
        ));
    }
    let call = calls[0];
    if call < chain.gate {
        return Err(format!("{boot:?} appears BEFORE the gate binding"));
    }
    let block_start = chain.if_open + 1;
    let block_end = block_start + chain.block.len();
    if !(block_start..block_end).contains(&call) {
        return Err(format!(
            "{boot:?} is not INSIDE the `if standing_gateway {{ … }}` block (call at {call}, \
             block {block_start}..{block_end})"
        ));
    }
    // The lifecycle flag must be bound INSIDE the daemon-config literal that
    // follows the gate — a textual occurrence anywhere later is not wiring (the
    // pre-NIT walk accepted one, so `idle_self_exit: true` in the real literal
    // plus the needle in stray text passed).
    let cfg_needle = "NetdConfig {";
    let cfg_rel = code[chain.gate..]
        .find(cfg_needle)
        .ok_or_else(|| "no `NetdConfig {` literal after the gate binding".to_string())?;
    let cfg_open = chain.gate + cfg_rel + cfg_needle.len() - 1;
    let cfg_block = brace_block(code, cfg_open)
        .ok_or_else(|| "the `NetdConfig` literal never balances".to_string())?;
    let flag = "idle_self_exit: !standing_gateway";
    if !cfg_block.contains(flag) {
        return Err(format!(
            "{flag:?} is not bound INSIDE the `NetdConfig {{ … }}` literal that follows the gate"
        ));
    }
    // Shadow guard: exactly ONE binding/assignment of `standing_gateway` in the
    // file — a shadowing `let standing_gateway = false;` (or a reassignment)
    // between the gate and the literal disables the whole chain while every
    // needle above still matches. (`matches` counts substrings, so a
    // `standing_gateway ==` comparison would also count here — none exists, and
    // a false positive fails LOUDLY rather than silently passing.)
    let bindings = code.matches("let standing_gateway").count();
    if bindings != 1 {
        return Err(format!(
            "expected exactly ONE `let standing_gateway` binding (the gate's), found {bindings} — \
             a shadow rebinding disables the chain"
        ));
    }
    let assigns = code.matches("standing_gateway =").count();
    if assigns != 1 {
        return Err(format!(
            "expected exactly ONE `standing_gateway =` (the gate binding), found {assigns}"
        ));
    }
    Ok(())
}

/// Arm 4: `main()` wires the gate → boot → lifecycle chain, STRUCTURALLY.
#[test]
fn main_rs_wires_the_standing_boot_inside_the_gate_and_the_idle_exit_flag() {
    let main_rs = concat!(env!("CARGO_MANIFEST_DIR"), "/src/main.rs");
    let src = std::fs::read_to_string(main_rs).expect("read src/main.rs");
    let code = code_only(&src);
    if let Err(why) = check_chain(&code) {
        panic!(
            "src/main.rs (comment-stripped) does not wire the standing-gateway chain \
             gate → `if standing_gateway {{ boot }}` → idle_self_exit: {why}"
        );
    }
}

/// The checker's own oracle — hand-written shapes, so the arm above cannot pass
/// on a checker that accepts anything. Includes the exact variant the arm exists
/// to catch: the call MOVED OUT of the gate's block while every substring survives.
#[test]
fn the_wiring_checker_accepts_only_a_call_inside_the_gates_block() {
    let good = "\
let standing_gateway = net::standing_gateway_requested(network.as_ref());
let cfg = NetdConfig { idle_self_exit: !standing_gateway, ..NetdConfig::default() };
if standing_gateway {
    let serving = build();
    if let Err(e) = plane.boot_standing_gateway(&serving) { warn(e); }
}
run();
";
    assert_eq!(check_chain(good), Ok(()));

    // The regression: the call hoisted OUT of the block (unconditional boot) — every
    // substring still present.
    let hoisted = "\
let standing_gateway = net::standing_gateway_requested(network.as_ref());
let cfg = NetdConfig { idle_self_exit: !standing_gateway, ..NetdConfig::default() };
let _ = plane.boot_standing_gateway(&serving);
if standing_gateway {
    let serving = build();
}
";
    assert!(
        check_chain(hoisted).is_err(),
        "a call outside the block must fail"
    );

    // The call BEFORE the gate is even computed.
    let early = "\
let _ = plane.boot_standing_gateway(&serving);
let standing_gateway = net::standing_gateway_requested(network.as_ref());
let cfg = NetdConfig { idle_self_exit: !standing_gateway, ..NetdConfig::default() };
if standing_gateway { }
";
    assert!(check_chain(early).is_err());

    // Two calls (one inside, one outside) — the outside one is still a boot.
    let twice = "\
let standing_gateway = net::standing_gateway_requested(network.as_ref());
let cfg = NetdConfig { idle_self_exit: !standing_gateway, ..NetdConfig::default() };
if standing_gateway { plane.boot_standing_gateway(&s); }
plane.boot_standing_gateway(&s);
";
    assert!(check_chain(twice).is_err());

    // No lifecycle flag, or the flag not derived from the gate.
    let no_flag = "\
let standing_gateway = net::standing_gateway_requested(network.as_ref());
if standing_gateway { plane.boot_standing_gateway(&s); }
";
    assert!(check_chain(no_flag).is_err());

    // A SHADOW rebinding between the gate and the config literal — every needle
    // still present, the chain silently disabled. (The NIT the literal-bound
    // walk exists for, half 1.)
    let shadowed = "\
let standing_gateway = net::standing_gateway_requested(network.as_ref());
let standing_gateway = false;
let cfg = NetdConfig { idle_self_exit: !standing_gateway, ..NetdConfig::default() };
if standing_gateway { plane.boot_standing_gateway(&s); }
";
    assert!(
        check_chain(shadowed).is_err(),
        "a shadow `let standing_gateway = false;` must fail the walk"
    );

    // The REAL literal binds `idle_self_exit: true` while the good needle text
    // occurs LATER, outside it — the pre-NIT anywhere-later walk PASSED this.
    // (Half 2: the flag must be inside the literal that actually builds the
    // daemon's config.)
    let flag_elsewhere = "\
let standing_gateway = net::standing_gateway_requested(network.as_ref());
let cfg = NetdConfig { idle_self_exit: true, ..NetdConfig::default() };
if standing_gateway { plane.boot_standing_gateway(&s); }
let stray = NetdConfig { idle_self_exit: !standing_gateway, ..NetdConfig::default() };
";
    assert!(
        check_chain(flag_elsewhere).is_err(),
        "`idle_self_exit: true` in the daemon's literal must fail even when the good needle \
         appears later in stray text"
    );

    // A comment-only mention satisfies nothing (the stripper is load-bearing).
    let commented = code_only(
        "// let standing_gateway = net::standing_gateway_requested(network.as_ref());\nfn f() {}",
    );
    assert!(check_chain(&commented).is_err());
}
