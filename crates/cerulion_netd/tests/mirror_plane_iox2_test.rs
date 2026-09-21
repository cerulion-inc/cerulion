// SPDX-License-Identifier: AGPL-3.0-only
//! The PRODUCTION mirror-plane path over REAL iceoryx2 + zenoh —
//! proof that `GatewayMirrorPlane` is not inert (the "no inert shipping" rule).
//!
//! Builds a network-configured `TransportManager` over an ISOLATED per-test SHM
//! root (`init_for_test` + `iceoryx_test_config`) with a scouting-OFF,
//! `robot_identity: None` desk `NetworkConfig` (netd's ingress-only shape) — the
//! `network_ingress_test.rs` / `gateway_iox2_test.rs` hermetic pattern. No remote
//! peer is needed: `register_ingress_topic` opens a local-only zenoh session
//! (scouting off, no endpoints) and creates the local SHM mirror publisher, which
//! a local subscriber can then open — proving the demand→mirror wiring is real.
//!
//! The HEADLINE pin (`demand_release_redemand_reuses_the_bridge_over_real_transport`)
//! drives the REAL daemon + the production plane over real `register_ingress_topic`
//! and proves the sequential demand→release→re-demand path SUCCEEDS on the same key
//! — the lingering-reuse fix (a re-demand of a released mirror reuses the existing
//! bridge instead of re-registering it, which the transport would REFUSE).
//!
//! Parallel-safe (per-test SHM roots + scouting-off local sessions + unique
//! sockets), so NOT `#[serial]` — mirrors the `network_ingress_test.rs` convention.

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::mirror_registry::MirrorRecord;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};

use cerulion_netd::daemon::{self, NetdConfig};
use cerulion_netd::mirror::{GatewayMirrorPlane, MirrorPlane, MirrorRelease};
use cerulion_netd::protocol::{Hello, Request, Response};
use cerulion_netd::registry::TopicKey;

/// An arbitrary wire schema hash — registration stores it to validate INBOUND
/// frames (none arrive in this hermetic test), so any value exercises the path.
const PROBE_HASH: u64 = 0x0BAD_F00D_DEAD_BEEF;

/// Gather mirror provenance until it contains `want`, or the budget elapses (the
/// live registry republishes ~every 150ms; the in-process pub↔sub takes a pass or
/// two — the `gather_until` pattern). Returns whether it appeared.
fn gather_until_contains(
    manager: &TransportManager,
    want: &MirrorRecord,
    budget: Duration,
) -> bool {
    let deadline = Instant::now() + budget;
    loop {
        let got = manager
            .gather_mirror_provenance(Duration::from_millis(250))
            .expect("gather provenance");
        if got.contains(want) {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
    }
}

/// Gather mirror provenance until it NO LONGER contains `gone`, or the budget
/// elapses (after `unregister_mirror_provenance` the record stops being
/// republished, so a fresh gather window ages it out). Returns whether it became
/// absent.
fn gather_until_absent(manager: &TransportManager, gone: &MirrorRecord, budget: Duration) -> bool {
    let deadline = Instant::now() + budget;
    loop {
        let got = manager
            .gather_mirror_provenance(Duration::from_millis(250))
            .expect("gather provenance");
        if !got.contains(gone) {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
    }
}

/// Build a network-configured (scouting-off, ingress-only) manager over an
/// isolated per-test SHM root.
fn networked_test_manager(node: &str) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node.to_string(),
            // NetworkConfig::default() is scouting-OFF + robot_identity None —
            // exactly netd's ingress-only desk shape, and hermetic (no multicast).
            network: Some(NetworkConfig::default()),
            ..TransportConfig::default()
        },
        iceoryx_test_config(),
    )
    .expect("init networked test manager")
}

#[test]
fn ensure_mirror_registers_a_real_shared_mirror_and_release_tears_down() {
    let manager = networked_test_manager("netd_mirror_ok");
    // Before any demand the lazy session has not opened.
    assert!(
        !manager.network().expect("has network").is_active(),
        "the zenoh session is LAZY — nothing opened before the first demand"
    );

    let plane = GatewayMirrorPlane::new(Arc::clone(&manager));
    let key = TopicKey::new("ubuntu", "/netd/mirror/probe");

    // FIRST demand → register the shared mirror. This is the exact production call
    // the daemon makes on a refcount 0→1 transition.
    plane
        .ensure_mirror(&key, PROBE_HASH)
        .expect("ensure_mirror registers the shared mirror");

    // The mirror is REAL: the local SHM data service now exists at the CANONICAL
    // topic name, so a normal consumer can open it (this is how vizd / topic echo
    // / a user graph subscribe zero-copy to the one mirror).
    assert!(
        manager.create_subscriber_open_only(&key.topic).is_ok(),
        "the mirror's local data service exists at the canonical topic name"
    );
    // And the lazy zenoh session opened on that first register.
    assert!(
        manager.network().expect("has network").is_active(),
        "the first ensure_mirror opened the one zenoh session"
    );
    // The mirror topic is in the self_ingress exclusion set
    // (register_ingress_topic inserted it) — the pre-teardown anchor for the
    // removal assert below.
    assert!(
        manager.network().expect("net").is_self_ingress(&key.topic),
        "the mirror topic is in the self_ingress exclusion set while ingressed"
    );

    // Ensure_mirror ALSO registered the mirror's PROVENANCE at the
    // re-injection point, so `topic list` folds this netd-mirrored topic into the
    // REMOTE section (● streaming {robot}) instead of showing it as a phantom
    // LOCAL. Gather it back (best-effort; the live registry republishes ~150ms).
    let record = MirrorRecord {
        topic: key.topic.clone(),
        origin_robot: key.robot.clone(),
    };
    assert!(
        gather_until_contains(&manager, &record, Duration::from_secs(5)),
        "ensure_mirror registered the mirror provenance ({} → {}) so topic list folds it into REMOTE",
        key.topic,
        key.robot,
    );

    // A SECOND, distinct topic registers its own mirror on the SAME session — the
    // isolation control (tearing down `key` must not disturb `key2`).
    let key2 = TopicKey::new("ubuntu", "/netd/mirror/probe2");
    plane
        .ensure_mirror(&key2, PROBE_HASH)
        .expect("a second distinct topic registers on the one session");
    assert!(manager.create_subscriber_open_only(&key2.topic).is_ok());

    // Release_mirror performs the REAL teardown — unregister the
    // ingress bridge + best-effort remove provenance — and returns Retired.
    assert_eq!(
        plane.release_mirror(&key),
        MirrorRelease::Retired,
        "release_mirror tears the real bridge down (Retired)"
    );
    // The mirror's local data service was RELEASED — a fresh open now errs (the
    // publisher slot is free for a clean re-create).
    assert!(
        manager.create_subscriber_open_only(&key.topic).is_err(),
        "release_mirror tore the mirror down — no phantom data service survives"
    );
    // The torn-down topic left the self_ingress exclusion set.
    assert!(
        !manager.network().expect("net").is_self_ingress(&key.topic),
        "teardown removed the topic from the self_ingress set"
    );
    // The mirror provenance was removed — `topic list` no longer folds it into REMOTE.
    assert!(
        gather_until_absent(&manager, &record, Duration::from_secs(5)),
        "release_mirror removed the mirror provenance so topic list stops folding it into REMOTE"
    );
    // Isolation: key2's mirror + self_ingress entry are UNTOUCHED.
    assert!(
        manager.create_subscriber_open_only(&key2.topic).is_ok(),
        "the sibling mirror survives the teardown of key"
    );
    assert!(
        manager.network().expect("net").is_self_ingress(&key2.topic),
        "the sibling's self_ingress entry survives"
    );
}

#[test]
fn ensure_mirror_without_network_errors_and_creates_no_phantom_mirror() {
    // A local-only manager (CERULION_NETD_NETWORK=off shape: network None).
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: "netd_mirror_nonet".to_string(),
            ..TransportConfig::default() // network: None
        },
        iceoryx_test_config(),
    )
    .expect("init local-only manager");
    assert!(
        manager.network().is_none(),
        "local-only manager has no network"
    );

    let plane = GatewayMirrorPlane::new(Arc::clone(&manager));
    let key = TopicKey::new("ubuntu", "/netd/mirror/nonet");

    // ensure_mirror errors LOUDLY (the explicit not-network-configured residual),
    // naming the offending topic.
    let err = plane
        .ensure_mirror(&key, PROBE_HASH)
        .expect_err("no network → ensure_mirror errors");
    let msg = err.to_string();
    assert!(msg.contains("/netd/mirror/nonet"), "names the topic: {msg}");
    assert_eq!(err.key().topic, "/netd/mirror/nonet");

    // No phantom mirror was created — opening the topic still errors (nothing to open).
    assert!(
        manager.create_subscriber_open_only(&key.topic).is_err(),
        "a failed ensure_mirror leaves no local data service behind"
    );
}

/// A thread-safe in-memory `std::io::Write` for a scoped `tracing_subscriber::fmt`
/// subscriber — captures log lines synchronously into a shared buffer. Used to
/// assert the production teardown-FAILURE `error!` fired, WITHOUT depending on any
/// process-global subscriber (a scoped `with_default` thread-local dispatcher
/// always wins for events emitted synchronously on the calling thread).
#[derive(Clone)]
struct SharedBuf(Arc<std::sync::Mutex<Vec<u8>>>);
impl std::io::Write for SharedBuf {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl tracing_subscriber::fmt::MakeWriter<'_> for SharedBuf {
    type Writer = SharedBuf;
    fn make_writer(&self) -> Self::Writer {
        self.clone()
    }
}

#[test]
fn second_release_hits_the_real_err_lingering_teardown_failure_arm() {
    // Pin the production `release_mirror` Err→Lingering
    // teardown-FAILURE arm over REAL transport — deterministically, no mock. After
    // a successful Retired teardown the mirror is gone from the ingress map, so a
    // SECOND `release_mirror` drives `unregister_ingress_topic` into its
    // "no registered network ingress bridge" error, which is exactly the Err(e)
    // arm: it emits the loud `error!` and returns Lingering (keep the entry
    // reusable rather than half-torn-down). The daemon_e2e SpyPlane's
    // unconditional-Lingering tests only pin the daemon's ROUTING of a Lingering
    // result — this pins the real arm inside the production plane.
    let manager = networked_test_manager("netd_mirror_err_arm");
    let plane = GatewayMirrorPlane::new(Arc::clone(&manager));
    let key = TopicKey::new("ubuntu", "/netd/mirror/errarm");

    plane
        .ensure_mirror(&key, PROBE_HASH)
        .expect("ensure the mirror");
    // First release → REAL teardown → Retired (the mirror is gone).
    assert_eq!(
        plane.release_mirror(&key),
        MirrorRelease::Retired,
        "the first release tears the real bridge down"
    );
    assert!(
        manager.create_subscriber_open_only(&key.topic).is_err(),
        "the bridge is gone after the first (Retired) release"
    );

    // SECOND release → unregister_ingress_topic errors (nothing to tear down) →
    // the real Err→Lingering fallback arm fires with the loud teardown-failure
    // `error!`. Capture it via a SCOPED subscriber around exactly this call (the
    // error! is emitted synchronously on this thread, so the thread-local default
    // dispatcher captures it deterministically).
    let buf = SharedBuf(Arc::new(std::sync::Mutex::new(Vec::new())));
    let subscriber = tracing_subscriber::fmt()
        .with_writer(buf.clone())
        .with_max_level(tracing::Level::TRACE)
        .without_time()
        .finish();
    let released = tracing::subscriber::with_default(subscriber, || plane.release_mirror(&key));
    assert_eq!(
        released,
        MirrorRelease::Lingering,
        "a failed teardown returns Lingering (the fallback, not Retired)"
    );
    let captured = String::from_utf8_lossy(&buf.0.lock().unwrap()).to_string();
    assert!(
        captured.contains("network mirror teardown FAILED"),
        "the Err arm emitted the loud teardown-failure error!; captured logs:\n{captured}"
    );
    // And ONLY the error! fired on this arm — no spurious success info!.
    assert!(
        !captured.contains("mirror torn down"),
        "the Err arm must NOT log the success line; captured:\n{captured}"
    );
}

#[test]
fn ensure_mirror_arms_the_demand_keepalive_release_disarms_it() {
    // WIRING pin (fast, hermetic — no remote peer): `ensure_mirror` arms the
    // self-heal demand-GET keepalive for the topic (observable via the manager's
    // keepalive topic count), and a Retired `release_mirror` disarms it. This is the
    // wiring the strict-link self-heal rides (the restart e2e's `_suppressed` twin
    // proves it is load-bearing over real zenoh); pinning the count here catches a
    // regression that forgot to arm/disarm without needing the full restart flow.
    let manager = networked_test_manager("netd_keepalive_wiring");
    let plane = GatewayMirrorPlane::new(Arc::clone(&manager));
    assert_eq!(
        manager.ingress_demand_keepalive_topic_count(),
        0,
        "no demand keepalive before any mirror"
    );

    let key1 = TopicKey::new("ubuntu", "/netd/keepalive/one");
    let key2 = TopicKey::new("ubuntu", "/netd/keepalive/two");
    plane.ensure_mirror(&key1, PROBE_HASH).expect("ensure key1");
    assert_eq!(
        manager.ingress_demand_keepalive_topic_count(),
        1,
        "ensure_mirror armed the keepalive for key1"
    );
    plane.ensure_mirror(&key2, PROBE_HASH).expect("ensure key2");
    assert_eq!(
        manager.ingress_demand_keepalive_topic_count(),
        2,
        "a second mirror grows the keepalive set"
    );

    // A Retired teardown disarms ONLY that topic (the sibling keeps self-healing).
    assert_eq!(plane.release_mirror(&key1), MirrorRelease::Retired);
    assert_eq!(
        manager.ingress_demand_keepalive_topic_count(),
        1,
        "release_mirror disarmed key1's keepalive; key2's survives"
    );
    assert_eq!(plane.release_mirror(&key2), MirrorRelease::Retired);
    assert_eq!(
        manager.ingress_demand_keepalive_topic_count(),
        0,
        "the last release disarms the last keepalive"
    );
}

// --------------------------------------------------------------------------
// The headline teardown pin: the FULL demand→release→re-demand cycle over the REAL
// daemon + production plane + real (un)register_ingress_topic — the mirror is
// torn down on the last release and cleanly RE-CREATED on re-demand.
// --------------------------------------------------------------------------

/// A minimal UDS client for the daemon control protocol.
struct Client {
    stream: UnixStream,
    reader: BufReader<UnixStream>,
}

impl Client {
    fn connect(sock: &Path) -> Client {
        let stream = UnixStream::connect(sock).expect("connect");
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let reader = BufReader::new(stream.try_clone().expect("clone"));
        let mut c = Client { stream, reader };
        // Consume the Hello banner.
        let _: Hello = serde_json::from_str(c.read_line().trim()).expect("hello parses");
        c
    }
    fn read_line(&mut self) -> String {
        let mut line = String::new();
        let n = self.reader.read_line(&mut line).expect("read");
        assert!(n > 0, "server closed unexpectedly");
        line
    }
    fn request(&mut self, req: &Request) -> Response {
        writeln!(self.stream, "{}", req.to_json_line()).expect("send");
        self.stream.flush().expect("flush");
        serde_json::from_str(self.read_line().trim()).expect("resp parses")
    }
}

fn unique_socket(tag: &str) -> (PathBuf, PathBuf) {
    static N: AtomicU64 = AtomicU64::new(0);
    let n = N.fetch_add(1, Ordering::Relaxed);
    let dir = std::env::temp_dir().join(format!("cer_netd_reuse_{tag}_{}_{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mk tempdir");
    (dir.clone(), dir.join("netd.sock"))
}

#[test]
fn demand_release_redemand_recreates_the_bridge_over_real_transport() {
    // End-to-end over REAL (un)register_ingress_topic: demand → mirror up →
    // last release → REAL teardown (Retired) → re-demand → mirror RE-CREATED. The
    // last release RETIRES the registry entry (no lingering), and the re-demand
    // classifies as a fresh FirstDemand that cleanly re-registers the bridge — the
    // full cycle the "obtain the copy once, tear it down when nobody
    // wants it, re-obtain on demand" model requires.
    let (dir, sock) = unique_socket("live");
    let manager = networked_test_manager("netd_recreate_daemon");
    let plane: Arc<dyn MirrorPlane> = Arc::new(GatewayMirrorPlane::new(Arc::clone(&manager)));
    let mut netd = daemon::start(
        sock.clone(),
        plane,
        NetdConfig {
            idle_grace: Duration::from_secs(3600), // never self-exit mid-test
            idle_watch_poll: Duration::from_millis(50),
            ..NetdConfig::default()
        },
    )
    .expect("daemon start");

    let mut c = Client::connect(&sock);
    let topic = "/netd/recreate/tf";

    // (1) First demand → registers the real bridge.
    match c.request(&Request::Demand {
        id: 1,
        robot: "ubuntu".to_string(),
        topic: topic.to_string(),
        schema_hash: PROBE_HASH,
    }) {
        Response::Demand(d) => assert!(d.mirror_created && d.refcount == 1, "{d:?}"),
        other => panic!("first demand should succeed: {other:?}"),
    }
    // The real mirror exists.
    assert!(manager.create_subscriber_open_only(topic).is_ok());

    // (2) Release → last demander leaves; the plane TEARS THE BRIDGE DOWN and RETIRES the
    // registry entry (no lingering).
    match c.request(&Request::Release {
        id: 2,
        robot: "ubuntu".to_string(),
        topic: topic.to_string(),
    }) {
        Response::Release(r) => assert!(r.last_release && r.refcount == 0, "{r:?}"),
        other => panic!("release should succeed: {other:?}"),
    }
    assert_eq!(
        netd.lingering_count(),
        0,
        "the last release RETIRES the entry, it does not linger"
    );
    assert_eq!(netd.mirror_count(), 0, "the registry entry is gone");
    assert!(
        manager.create_subscriber_open_only(topic).is_err(),
        "the mirror's data service was released by the teardown"
    );

    // (3) RE-DEMAND the SAME topic → a fresh FirstDemand that RE-CREATES the mirror
    // (mirror_created == true). The teardown at the last release RETIRED it, and
    // that teardown freed the single-writer slot so this re-register
    // SUCCEEDS (a non-torn-down bridge would refuse it).
    match c.request(&Request::Demand {
        id: 3,
        robot: "ubuntu".to_string(),
        topic: topic.to_string(),
        schema_hash: PROBE_HASH,
    }) {
        Response::Demand(d) => {
            assert!(
                d.mirror_created,
                "re-demand RE-CREATES the mirror (a fresh FirstDemand after the teardown)"
            );
            assert_eq!(d.refcount, 1);
        }
        Response::Error(e) => panic!(
            "re-demand FAILED: {} — the teardown did not free the mirror for re-creation",
            e.error
        ),
        other => panic!("unexpected: {other:?}"),
    }
    // The mirror is back up as the ONE real bridge, openable again.
    assert_eq!(netd.mirror_count(), 1);
    assert_eq!(netd.active_demand_count(), 1);
    assert!(manager.create_subscriber_open_only(topic).is_ok());

    netd.shutdown();
    let _ = std::fs::remove_dir_all(&dir);
}
