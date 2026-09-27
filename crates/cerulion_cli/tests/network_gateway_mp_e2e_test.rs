// SPDX-License-Identifier: AGPL-3.0-only
//! The HEADLINE STRICT networked MULTI-PROCESS acceptance over the
//! REAL `cerulion` binary — networked multi-process WORKS (the v1
//! refusal is deleted; workers carry no zenoh session, the single per-run
//! GATEWAY process owns the whole network plane).
//!
//! A 2-group `process_groups:` graph (`p0:[ticker,relay]` rank 0 /
//! `p1:[sink]` rank 1) with an explicit `network:` block egress-listing the
//! rank-1-produced topic (`/gwmp/telemetry`, an absolute output override on
//! `sink`) is run under the REAL supervisor, which spawns the workers and ONE
//! gateway. SEQUENCING: the gateway spawns AFTER the GO gate and binds its
//! listen port seconds later, so the test first waits (bounded TCP probe) for
//! that listener to ACCEPT — only then does it open the TEST-SIDE
//! `cerulion_core` `TransportManager` on a DISTINCT iceoryx2 SHM root, talking
//! to the gateway ONLY over the explicit 127.0.0.1 zenoh hop (scouting off =
//! hermetic). The link forms at session open and the DEMAND token
//! (`register_ingress_topic`) is declared over the live link, which flips the
//! gateway's egress flag; the gateway taps the rank-1 worker's SHM and
//! forwards each frame to zenoh, and the test-side manager re-injects +
//! receives it. This proves the full worker → SHM → gateway tap → zenoh →
//! external-subscriber chain ACROSS process boundaries.
//!
//! Two independent proofs land per run:
//!  * **delivery count > 0** through the test-side ingress subscriber (the
//!    rank-1 frames really crossed the machine boundary), plus the PAYLOAD
//!    oracle — every frame decodes to exactly the hand-derivable (0.0, 0.0,
//!    0.0) `Vector3` (ticker x=0.0, zero-init y/z, forwarded verbatim) with
//!    header total_size/offset-table/sequence sanity (no mangling anywhere in
//!    the worker → SHM → tap → zenoh → re-inject composition);
//!  * the egress topic's **announce is visible** to a real `cerulion topic
//!    list --connect <gateway-locator>` invocation (the REMOTE TOPICS section).
//!
//! The in-process byte-identity of the P→G→B chain is already pinned by
//! `cerulion_core/tests/network_ingress_e2e_test.rs`; THIS file pins the same
//! chain composed through the real supervisor + real workers + a real gateway
//! subprocess.
//!
//! Port robustness: an explicit `network:` block's listen endpoint is used
//! VERBATIM (Strict — no bind ladder), so a probed port stolen in the
//! probe→gateway-bind window fails the gateway's boot (the run degrades
//! LOCAL-ONLY) — the listener-accept wait then times out and the attempt is
//! retried promptly. The whole setup is wrapped in a bounded retry (fresh
//! port + fresh supervisor) — the `network_yaml_e2e_test.rs` convention
//! lifted to the subprocess boundary.
//!
//! Prerequisites (the repo's fixture pattern — PANICs with the instruction if
//! missing):
//! `cargo build -p test_node_macro_period_cdylib -p test_node_macro_data_trigger_cdylib`
//!
//! GATED `#[cfg(unix)]`, `#[serial]` (real binary + global iceoryx2 namespace).

#![cfg(unix)]

use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serial_test::serial;

use cerulion_core::message::ShmMessage;
use cerulion_core::transport::network::NetworkConfig;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use native_ros2_messages::geometry_msgs::Vector3;

mod mp_support;
mod serving_login_support;
use mp_support::{dylib_file, fixture_cdylib, read_file, send_signal, ChildGuard};

const PERIOD_FIXTURE: &str = "test_node_macro_period_cdylib";
const DATA_FIXTURE: &str = "test_node_macro_data_trigger_cdylib";
/// The absolute egress topic `sink` (rank 1) produces; the gateway announces +
/// taps it, the test-side manager ingresses it.
const EGRESS_TOPIC: &str = "/gwmp/telemetry";
/// How many whole-setup attempts before giving up (a stolen probed port fails
/// the Strict gateway's verbatim bind; a fresh port + supervisor is retried).
const SETUP_ATTEMPTS: usize = 4;

static COUNTER: AtomicU64 = AtomicU64::new(0);

fn unique_suffix() -> String {
    let n = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let c = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{n}_{c}")
}

fn probe_ephemeral_port() -> u16 {
    let l = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral probe");
    let port = l.local_addr().expect("probe local_addr").port();
    drop(l);
    port
}

/// Hand-build the networked multi-process workspace: the 3-node chain
/// (`ticker`→`relay`→`sink`) split `p0:[ticker,relay]` / `p1:[sink]`, with an
/// explicit `network:` block listening on `port` and egress-listing
/// [`EGRESS_TOPIC`] (the absolute output override on `sink`).
fn build_networked_mp_workspace(root: &Path, prefix: &str, port: u16) {
    std::fs::create_dir_all(root.join("graphs")).unwrap();
    std::fs::create_dir_all(root.join("target/debug")).unwrap();
    std::fs::write(
        root.join("Cargo.toml"),
        "[workspace]\nresolver = \"2\"\nmembers = []\n",
    )
    .unwrap();
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("test_fixtures");
    for (node_type, fixture) in [
        ("ticker", PERIOD_FIXTURE),
        ("relay", DATA_FIXTURE),
        ("sink", DATA_FIXTURE),
    ] {
        std::fs::create_dir_all(root.join(format!("nodes/{node_type}/src"))).unwrap();
        std::fs::copy(
            fixtures.join(fixture).join("src/lib.rs"),
            root.join(format!("nodes/{node_type}/src/lib.rs")),
        )
        .expect("copy fixture src");
        std::fs::copy(
            fixture_cdylib(fixture),
            root.join("target/debug").join(dylib_file(node_type)),
        )
        .expect("copy fixture cdylib");
    }
    std::fs::write(
        root.join("graphs/gwmp.yaml"),
        format!(
            "name: gwmp\n\
             prefix: {prefix}\n\
             process_groups:\n\
             \x20 p0:\n\
             \x20 - ticker\n\
             \x20 - relay\n\
             \x20 p1:\n\
             \x20 - sink\n\
             network:\n\
             \x20 mode: peer\n\
             \x20 listen:\n\
             \x20 - tcp/127.0.0.1:{port}\n\
             \x20 egress:\n\
             \x20 - {EGRESS_TOPIC}\n\
             nodes:\n\
             - id: ticker\n\
             \x20 type: ticker\n\
             \x20 inputs: []\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n\
             - id: relay\n\
             \x20 type: relay\n\
             \x20 inputs:\n\
             \x20 - name: trigger_in\n\
             \x20\x20\x20 source: ticker/cmd\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n\
             - id: sink\n\
             \x20 type: sink\n\
             \x20 inputs:\n\
             \x20 - name: trigger_in\n\
             \x20\x20\x20 source: relay/cmd\n\
             \x20 outputs:\n\
             \x20 - name: cmd\n\
             \x20\x20\x20 schema: geometry_msgs/Vector3\n\
             \x20\x20\x20 topic: {EGRESS_TOPIC}\n"
        ),
    )
    .unwrap();
}

/// Spawn `cerulion graph run gwmp --no-validate` (mp via `process_groups:`;
/// STRICT network from the explicit block — NOT permissive, so no
/// `CERULION_GATEWAY_PORT`). NOT `CERULION_NETWORK=off`: this test's PURPOSE
/// is the networked run.
fn spawn_supervisor(root: &Path) -> (ChildGuard, PathBuf, PathBuf) {
    let login_home = serving_login_support::expired_login(root);
    let stdout_path = root.join("run.stdout");
    let stderr_path = root.join("run.stderr");
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_cerulion"));
    cmd.args(["graph", "run", "gwmp", "--no-validate"])
        .current_dir(root)
        .env("CERULION_HOME", &login_home)
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_NETWORK")
        .env_remove("CARGO_TARGET_DIR")
        .env("RUST_LOG", "cerulion=info,cerulion_cli_engine=info")
        .env("NO_COLOR", "1")
        .stdin(Stdio::null())
        .stdout(Stdio::from(std::fs::File::create(&stdout_path).unwrap()))
        .stderr(Stdio::from(std::fs::File::create(&stderr_path).unwrap()));
    let guard = ChildGuard::spawn_group_leader(&mut cmd)
        .expect("spawn cerulion graph run (networked multi-process)");
    (guard, stdout_path, stderr_path)
}

/// True once the supervisor log shows the gateway DEGRADED to local-only (its
/// verbatim Strict bind lost the probe→bind port race, or it could not spawn).
/// Detecting it lets the retry re-attempt promptly rather than burning the full
/// delivery deadline.
fn gateway_degraded(stdout_path: &Path, stderr_path: &Path) -> bool {
    let log = format!("{}{}", read_file(stdout_path), read_file(stderr_path));
    log.contains("run continues LOCAL-ONLY") || log.contains("could not spawn the network gateway")
}

/// Build a test-side manager on a DISTINCT SHM root that connects to the
/// gateway (`tcp/127.0.0.1:port`), subscribes to [`EGRESS_TOPIC`], and declares
/// the DEMAND token. Returns the manager (owns the session) + its subscriber.
fn build_ingress_manager(
    port: u16,
    tag: &str,
) -> (
    Arc<TransportManager>,
    cerulion_core::transport::subscriber::CerulionSubscriber,
) {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("gwmp_b_{tag}"),
            network: Some(NetworkConfig {
                connect_endpoints: vec![format!("tcp/127.0.0.1:{port}")],
                // Scouting OFF — reach exactly the gateway locator (hermetic).
                ..NetworkConfig::default()
            }),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init test-side ingress manager");
    let sub = mgr
        .create_subscriber(EGRESS_TOPIC)
        .expect("test-side subscriber");
    mgr.register_ingress_topic(
        EGRESS_TOPIC,
        Vector3::SCHEMA_HASH,
        MaxSliceLen::const_new(256),
    )
    .expect("register the demand token for the egress topic");
    (mgr, sub)
}

/// One frame captured off the test-side ingress subscriber: the full wire
/// header fields the oracle checks plus the PAYLOAD BYTES (the mangling-proof
/// half — count + schema hash alone would pass a byte-corrupting composition
/// bug in the worker→SHM→tap→to_vec→zenoh→re-inject chain).
#[derive(Debug)]
struct GotFrame {
    schema_hash: u64,
    sequence: u32,
    total_size: u32,
    offset_table_offset: u32,
    offset_table_count: u32,
    payload: Vec<u8>,
}

/// Poll the test-side subscriber up to `timeout`, returning the captured
/// frames, or bail early if the supervisor reports the gateway degraded to
/// local-only.
fn collect_ingress_frames(
    sub: &cerulion_core::transport::subscriber::CerulionSubscriber,
    stdout_path: &Path,
    stderr_path: &Path,
    timeout: Duration,
) -> Vec<GotFrame> {
    let mut got: Vec<GotFrame> = Vec::new();
    let deadline = Instant::now() + timeout;
    while got.is_empty() && Instant::now() < deadline {
        sub.try_receive(|msg| {
            let h = msg.header();
            got.push(GotFrame {
                schema_hash: h.schema_hash,
                sequence: h.sequence,
                total_size: h.total_size,
                offset_table_offset: h.offset_table_offset,
                offset_table_count: h.offset_table_count,
                payload: msg.payload().to_vec(),
            });
        })
        .expect("test-side try_receive");
        if got.is_empty() {
            if gateway_degraded(stdout_path, stderr_path) {
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
    }
    got
}

/// The REMOTE half of `cerulion topic list --connect <locator>` (everything
/// after the `REMOTE TOPICS` marker), or `""` if the section is absent.
fn remote_topics_section(output: &str) -> String {
    match output.find("REMOTE TOPICS") {
        Some(i) => output[i..].to_string(),
        None => String::new(),
    }
}

/// SIGINT the supervisor and reap it (bounded); its `GatewayChild::Drop` reaps
/// the gateway. Used both on the retry-teardown and the success path.
fn stop_supervisor(guard: &mut ChildGuard) {
    send_signal(guard.id(), libc::SIGINT);
    let _ = guard.wait_bounded(Duration::from_secs(60));
}

/// Bounded wait for the gateway's zenoh listener to ACCEPT on `port`. The
/// gateway is spawned by the supervisor AFTER the GO gate and binds seconds
/// later; the pinned zenoh does not reliably form the link (or propagate a
/// pre-declared demand token) when the test side connects BEFORE the listener
/// exists, so the test-side manager must not open its session until the port
/// accepts. The probe socket is dropped immediately (a probe, not a session).
fn wait_for_gateway_listener(port: u16, timeout: Duration) -> bool {
    let addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));
    let start = Instant::now();
    while start.elapsed() < timeout {
        if let Ok(probe) = std::net::TcpStream::connect_timeout(&addr, Duration::from_millis(200)) {
            drop(probe);
            return true;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    false
}

/// The headline: a networked multi-process run delivers the rank-1 worker's
/// frames to an external test-side ingress subscriber, and the egress topic is
/// visible to a real `topic list --connect`.
#[test]
#[serial]
fn networked_mp_delivers_to_external_ingress_and_announces_topic() {
    let mut established: Option<(ChildGuard, PathBuf, PathBuf, u16, tempfile::TempDir)> = None;
    let mut delivered: Vec<GotFrame> = Vec::new();

    for attempt in 0..SETUP_ATTEMPTS {
        let tmp = tempfile::tempdir().unwrap();
        let port = probe_ephemeral_port();
        let prefix = format!("gwmp{attempt}");
        build_networked_mp_workspace(tmp.path(), &prefix, port);
        let (mut guard, stdout_path, stderr_path) = spawn_supervisor(tmp.path());

        // ORDERING: wait for the gateway's listener to ACCEPT before opening
        // the test-side session — the gateway spawns after GO and binds
        // seconds later, and every green zenoh test connects AFTER the
        // listener exists (a pre-listener connect does not reliably form the
        // link / propagate the demand token).
        if !wait_for_gateway_listener(port, Duration::from_secs(20)) {
            eprintln!(
                "attempt {attempt}: gateway listener never accepted on port {port} \
                 (gateway_degraded={}); retrying with a fresh port",
                gateway_degraded(&stdout_path, &stderr_path)
            );
            stop_supervisor(&mut guard);
            continue;
        }

        // The test-side ingress manager: the link forms at session open (the
        // listener is live), and the demand token is declared over that live
        // link — the gateway's already-running watch sees the fresh
        // declaration and attaches its tap.
        let (_mgr, sub) = build_ingress_manager(port, &unique_suffix());
        let frames =
            collect_ingress_frames(&sub, &stdout_path, &stderr_path, Duration::from_secs(20));

        if !frames.is_empty() {
            delivered = frames;
            established = Some((guard, stdout_path, stderr_path, port, tmp));
            break;
        }
        // Failed attempt — tear the supervisor down (reaps the gateway) and
        // retry with a fresh port. `_mgr`/`sub` drop here (session closed).
        eprintln!(
            "attempt {attempt}: listener accepted but no ingress delivery within the window \
             (gateway_degraded={}); retrying with a fresh port",
            gateway_degraded(&stdout_path, &stderr_path)
        );
        stop_supervisor(&mut guard);
    }

    let (mut guard, _stdout_path, _stderr_path, port, _tmp) = established.unwrap_or_else(|| {
        panic!("networked mp never delivered a frame to the external ingress in {SETUP_ATTEMPTS} attempts")
    });

    // (1) delivery > 0 + the frame is the sink's Vector3 (not noise).
    assert!(
        !delivered.is_empty(),
        "the external ingress subscriber must receive ≥1 rank-1 frame"
    );
    assert!(
        delivered
            .iter()
            .any(|f| f.schema_hash == Vector3::SCHEMA_HASH),
        "a delivered frame must carry the sink's Vector3 schema hash; got {delivered:?}"
    );

    // (1b) PAYLOAD-BYTE oracle (the mangling proof — count + hash alone would
    // pass a byte-corrupting composition bug). The chain's values are fully
    // deterministic BY HAND: the ticker fixture writes `cmd.x = 0.0` and its
    // unwritten `y`/`z` are zero-initialized by the publisher loan (the
    // zero-init prefix covers the whole fixed section — a fixed field the
    // user never writes is observed as exactly 0); the relay and sink
    // fixtures each forward `cmd.x = trigger_in.x`, with their own `y`/`z`
    // likewise zero-init. So EVERY frame on `/gwmp/telemetry` decodes to
    // exactly (0.0, 0.0, 0.0) — asserted via `to_bits() == 0` (exact +0.0,
    // not an epsilon and not -0.0). Header sanity cribs the byte-identity
    // discipline of `network_ingress_e2e_test::oracle_frame`: a fixed
    // Vector3 frame is 32-byte header + 24 payload bytes, offset table
    // empty and sitting at total_size.
    let expected_total = (WireHeader::SIZE + 24) as u32;
    for f in &delivered {
        assert_eq!(
            f.payload.len(),
            24,
            "a Vector3 payload is exactly 3 f64s = 24 bytes; got {f:?}"
        );
        for (i, chunk) in f.payload.as_chunks::<8>().0.iter().enumerate() {
            let v = f64::from_le_bytes(*chunk);
            assert_eq!(
                v.to_bits(),
                0,
                "payload f64 #{i} must be exactly +0.0 (the hand oracle: ticker \
                 x=0.0, zero-init y/z, forwarded verbatim); got {v} in {f:?}"
            );
        }
        assert_eq!(
            f.total_size, expected_total,
            "wire total_size must be header + 24 payload bytes; got {f:?}"
        );
        assert_eq!(
            f.offset_table_offset, expected_total,
            "a fixed schema's (empty) offset table sits at total_size; got {f:?}"
        );
        assert_eq!(
            f.offset_table_count, 0,
            "a fixed schema carries no offset entries; got {f:?}"
        );
    }
    // Sequence sanity: strictly increasing in arrival order (the drop-to-live
    // tap may SKIP sequences under load, so gap-free would over-assert; a
    // duplicate or reorder through the re-inject path still fails here).
    for w in delivered.windows(2) {
        assert!(
            w[0].sequence < w[1].sequence,
            "wire sequences must be strictly increasing in arrival order; got {:?} then {:?}",
            w[0],
            w[1]
        );
    }

    // (2) the egress topic's announce is visible to `topic list --connect`.
    //
    // The peer cache is ISOLATED. `--connect` is ADDITIVE to the
    // automagic scouting-ON default, so this child runs the full discovery
    // ladder, whose cached-peers rung READS AND WRITES `~/.cerulion/peers.json`
    // (`dirs::home_dir()`, i.e. `$HOME`). This run's own gateway mDNS-advertises,
    // so `resolve_write_back` sees a verified mDNS row and `record_confirmed`
    // WROTE `(<this hostname>, tcp/<ip>:<probed EPHEMERAL port>)` into the
    // developer's REAL peer cache with a 7-day TTL — observed first-party on the
    // dev desk as two dead ephemeral-port self-entries. Every later scouting-ON
    // `topic list` then TCP-probes those dead locators for a week. An isolated
    // `HOME` + no `CERULION_PEERS` (the `quiet_cli_logging_e2e_test.rs` precedent)
    // keeps the cache rung inside this test's tempdir; the assertion below is a
    // positive check on the gateway's own explicitly-connected announce, so nothing
    // it pins comes from the cache.
    let cache_home = _tmp.path().join("home");
    std::fs::create_dir_all(&cache_home).expect("mk isolated HOME for the peer cache");
    let list = Command::new(env!("CARGO_BIN_EXE_cerulion"))
        .args([
            "topic",
            "list",
            "--connect",
            &format!("tcp/127.0.0.1:{port}"),
        ])
        .current_dir(_tmp.path())
        .env_remove("CARGO_TARGET_DIR")
        .env("HOME", &cache_home)
        .env_remove("CERULION_PEERS")
        .output()
        .expect("run cerulion topic list --connect");
    let stdout = String::from_utf8_lossy(&list.stdout);
    let remote = remote_topics_section(&stdout);
    assert!(
        remote.contains(EGRESS_TOPIC),
        "the gateway's egress announce must be visible in the REMOTE TOPICS section; \
         topic list stdout:\n{stdout}\nstderr:\n{}",
        String::from_utf8_lossy(&list.stderr)
    );

    // Clean teardown (the gateway is reaped by the supervisor's Drop).
    stop_supervisor(&mut guard);
}
