// SPDX-License-Identifier: AGPL-3.0-only
//! Real daemon/server lifecycle and authenticated delivery on loopback.
//! Prerequisite: cargo build -p cerulion_remoted in the same target/profile.
#![cfg(all(feature = "wan", unix))]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use cerulion_core::testing::IsolatedRoot;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, SchemaServing, TransportConfig, TransportManager,
};
use cerulion_link::{
    alpn, build_endpoint, dial, open_frame_stream, read_frame, write_frame, EndpointAddr,
    EndpointConfig, RelayConfig, DEFAULT_MAX_FRAME_LEN,
};
use cerulion_netd::protocol::{Hello, Request, Response};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PrincipalKind, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::DeviceAccountIndex;
use cerulion_wireclient::protocol::{WireRequest, WireResponse};

const TOPIC: &str = "/automatic/oracle";
const ROBOT_SEED: [u8; 32] = [0x41; 32];
const DESK_SEED: [u8; 32] = [0x42; 32];
const OWNER: AccountId = AccountId([0x43; 32]);
const MAC: [u8; 32] = [0x44; 32];
const LIMIT: Duration = Duration::from_secs(15);
/// How long remoted keeps a refused connection that never sends its control
/// frame before it drops it: remoted's private `HANDSHAKE_DEADLINE`, pinned at
/// its source by `the_handshake_deadline_is_ten_seconds` in its daemon tests.
const REMOTED_HANDSHAKE_DEADLINE: Duration = Duration::from_secs(10);
static PAIRED_DEVICE: std::sync::Mutex<()> = std::sync::Mutex::new(());

struct Daemon {
    child: Child,
    socket: std::path::PathBuf,
    root: tempfile::TempDir,
}

fn children(pid: u32) -> Vec<u32> {
    let output = Command::new("pgrep")
        .args(["-P", &pid.to_string()])
        .output()
        .unwrap();
    assert!(output.status.success() || output.status.code() == Some(1));
    String::from_utf8(output.stdout)
        .unwrap()
        .split_whitespace()
        .map(|pid| pid.parse().unwrap())
        .collect()
}

fn until(mut ready: impl FnMut() -> bool) {
    let deadline = Instant::now() + LIMIT;
    while !ready() {
        assert!(Instant::now() < deadline, "bounded lifecycle wait expired");
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn paired_state(root: &Path) {
    let dir = root.join("robot-state/remoted");
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("device_key"), ROBOT_SEED).unwrap();
    std::fs::write(dir.join("trust_store.mac_key"), MAC).unwrap();
    let key = DeviceIdentity::from_seed(&ROBOT_SEED).public_key();
    let mut store = TrustStore::provision(
        RobotId([0x45; 32]),
        key,
        RootSet::new(vec![DeviceIdentity::from_seed(&[0x46; 32]).public_key()], 1).unwrap(),
        b"physical-pairing-secret",
        0,
    )
    .unwrap()
    .with_path(dir.join("trust_store"));
    store
        .claim(OWNER, b"physical-pairing-secret", PrincipalKind::Human, 0)
        .unwrap();
    store.save(&MAC).unwrap();
    let mut index = DeviceAccountIndex::new().with_path(dir.join("device_index.json"));
    index.bind(DeviceIdentity::from_seed(&DESK_SEED).public_key(), OWNER);
    index.save(&MAC).unwrap();
}

impl Daemon {
    fn start(
        ix: &IsolatedRoot,
        network_off: bool,
        expired_login: bool,
        wan: Option<&str>,
        standing: bool,
    ) -> Self {
        Self::start_with(
            ix,
            network_off,
            expired_login,
            wan,
            standing,
            None,
            |_, _| {},
        )
    }

    fn start_with(
        ix: &IsolatedRoot,
        network_off: bool,
        expired_login: bool,
        wan: Option<&str>,
        standing: bool,
        binary: Option<&Path>,
        configure: impl FnOnce(&mut Command, &Path),
    ) -> Self {
        let root = tempfile::tempdir().unwrap();
        paired_state(root.path());
        if expired_login {
            std::fs::write(root.path().join("auth.json"), br#"{"account_id":"saved-account","session_token":"expired-session","refresh_token":"expired-refresh","expires_at_ns":0,"logged_in_ever":true}"#).unwrap();
        }
        std::fs::create_dir(root.path().join("config")).unwrap();
        std::fs::write(
            root.path().join("config/iceoryx2.toml"),
            format!(
                "[global]\nroot-path = {:?}\nprefix = {:?}\n",
                ix.root, ix.prefix
            ),
        )
        .unwrap();
        let socket = root.path().join("netd.sock");
        let selected = binary
            .map(Path::to_path_buf)
            .unwrap_or_else(test_netd_binary);
        let mut command = Command::new(selected);
        command
            .current_dir(root.path())
            .env("CERULION_HOME", root.path())
            .env("CERULION_NETD_SOCK", &socket)
            .env("CERULION_NETD_IDLE_GRACE_MS", "60000")
            .env("CERULION_NETD_RELAY_DISABLED", "1")
            .env("RUST_LOG", "warn,cerulion_remoted=info")
            .env_remove("CERULION_STATE_ROOT")
            .env_remove("CERULION_NETWORK")
            .env_remove("CERULION_NETD_LISTEN")
            .env_remove("CERULION_NETD_CONNECT")
            .env_remove("CERULION_NETD_WAN_ROBOTS")
            .env_remove("CERULION_NETD_DESK_KEY")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(std::fs::File::create(root.path().join("stderr")).unwrap());
        if network_off {
            command.env("CERULION_NETD_NETWORK", "off");
        } else {
            command.env_remove("CERULION_NETD_NETWORK");
        }
        if standing {
            command.env("CERULION_NETD_LISTEN", "tcp/127.0.0.1:0");
        }
        if let Some(wan) = wan {
            let key = root.path().join("desk.key");
            std::fs::write(&key, DESK_SEED).unwrap();
            command
                .env("CERULION_NETD_WAN_ROBOTS", wan)
                .env("CERULION_NETD_DESK_KEY", key)
                .env_remove("CERULION_NETD_DEVICE_CERT");
        }
        configure(&mut command, root.path());
        let child = command.spawn().unwrap();
        let mut daemon = Self {
            child,
            socket,
            root,
        };
        until(|| {
            assert!(
                daemon.child.try_wait().unwrap().is_none(),
                "{}",
                std::fs::read_to_string(daemon.root.path().join("stderr")).unwrap()
            );
            daemon.socket.exists()
        });
        daemon
    }

    fn connect(&self) -> (UnixStream, BufReader<UnixStream>) {
        let stream = UnixStream::connect(&self.socket).unwrap();
        stream.set_read_timeout(Some(LIMIT)).unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        let _: Hello = serde_json::from_str(&line).unwrap();
        (stream, reader)
    }

    fn stop(&mut self) {
        assert!(Command::new("kill")
            .args(["-TERM", &self.child.id().to_string()])
            .status()
            .unwrap()
            .success());
        until(|| self.child.try_wait().unwrap().is_some());
        assert!(children(self.child.id()).is_empty());
        assert!(!self.socket.exists());
    }
}

impl Drop for Daemon {
    fn drop(&mut self) {
        if std::thread::panicking() {
            eprintln!(
                "{}",
                std::fs::read_to_string(self.root.path().join("stderr")).unwrap_or_default()
            );
        }
        if self.child.try_wait().ok().flatten().is_none() {
            let _ = Command::new("kill")
                .args(["-TERM", &self.child.id().to_string()])
                .stderr(Stdio::null())
                .status();
            let deadline = Instant::now() + LIMIT * 2;
            while self.child.try_wait().ok().flatten().is_none() && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn request(client: &mut (UnixStream, BufReader<UnixStream>), request: Request) -> Response {
    writeln!(client.0, "{}", request.to_json_line()).unwrap();
    let mut line = String::new();
    assert!(client.1.read_line(&mut line).unwrap() > 0);
    serde_json::from_str(&line).unwrap()
}

fn register(id: u64, topic: &str) -> Request {
    Request::RegisterEgress {
        id,
        plan: GatewayPlan {
            egress_policy: GatewayEgressPolicy::AllowList(vec![topic.into()]),
            announce: vec![topic.into()],
            ingress: vec![],
        },
        schema_serving: SchemaServing::default(),
        ix_config_json: None,
    }
}

async fn ops_call(address: EndpointAddr, verb: &str, args: serde_json::Value) -> serde_json::Value {
    let desk = build_endpoint(EndpointConfig::new(DESK_SEED).with_relay(RelayConfig::Disabled))
        .await
        .unwrap();
    let connection = tokio::time::timeout(LIMIT, dial(&desk, address, alpn::OPS))
        .await
        .unwrap()
        .unwrap();
    let (mut send, mut recv) = open_frame_stream(&connection).await.unwrap();
    // Hand-written ops/1 envelopes exercise the real protocol without a client substitute.
    let ack = tokio::time::timeout(
        LIMIT,
        ops_exchange(
            &mut send,
            &mut recv,
            br#"{"min_version":1,"max_version":1}"#,
        ),
    )
    .await
    .unwrap();
    assert_eq!(ack["type"], "ack");
    let request = serde_json::json!({"id":0,"verb":verb,"args":args});
    let result = tokio::time::timeout(
        LIMIT,
        ops_exchange(&mut send, &mut recv, &serde_json::to_vec(&request).unwrap()),
    )
    .await
    .unwrap();
    connection.close(0u32.into(), b"done");
    desk.close().await;
    result
}

async fn ops_exchange(
    send: &mut cerulion_link::SendStream,
    recv: &mut cerulion_link::RecvStream,
    payload: &[u8],
) -> serde_json::Value {
    // ops/1 uses big-endian lengths; wire/2's framing is little-endian.
    send.write_all(&(payload.len() as u32).to_be_bytes())
        .await
        .unwrap();
    send.write_all(payload).await.unwrap();
    let mut length = [0; 4];
    recv.read_exact(&mut length).await.unwrap();
    let length = u32::from_be_bytes(length) as usize;
    assert!(length <= DEFAULT_MAX_FRAME_LEN);
    let mut bytes = vec![0; length];
    recv.read_exact(&mut bytes).await.unwrap();
    serde_json::from_slice(&bytes).unwrap()
}

#[test]
fn no_listen_serves_persisted_pairing_and_expired_login_and_reaps_on_last_release() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let remoted = Path::new(env!("CARGO_BIN_EXE_cerulion-netd")).with_file_name("cerulion-remoted");
    assert!(remoted.is_file(), "build the real remote daemon first");
    for expired in [false, true] {
        let ix = IsolatedRoot::mint("p2");
        let manager =
            TransportManager::init_for_test(TransportConfig::default(), ix.config()).unwrap();
        let mut publisher = manager
            .create_publisher_simple(TOPIC, MaxSliceLen::const_new(64))
            .unwrap();
        let mut daemon = Daemon::start(&ix, false, expired, None, false);
        let mut a = daemon.connect();
        let mut b = daemon.connect();
        assert!(
            children(daemon.child.id()).is_empty(),
            "query-only daemon starts no server"
        );
        assert!(matches!(
            request(&mut a, register(1, TOPIC)),
            Response::Egress(_)
        ));
        until(|| !children(daemon.child.id()).is_empty());
        let server_pid = children(daemon.child.id());
        assert_eq!(server_pid.len(), 1);
        let pid = rustix::process::Pid::from_raw(i32::try_from(server_pid[0]).unwrap()).unwrap();
        assert_eq!(
            rustix::process::getpgid(Some(pid)).unwrap(),
            pid,
            "owned remoted is isolated from netd's terminal process group"
        );
        assert!(matches!(
            request(&mut b, register(1, TOPIC)),
            Response::Egress(_)
        ));
        let facts = daemon
            .root
            .path()
            .join("robot-state/remoted/beacon_facts.json");
        until(|| facts.exists());
        let facts: serde_json::Value =
            serde_json::from_slice(&std::fs::read(facts).unwrap()).unwrap();
        let address = EndpointAddr::new(
            cerulion_link::EndpointId::from_bytes(
                &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
            )
            .unwrap(),
        )
        .with_ip_addr(
            format!("127.0.0.1:{}", facts["iroh_port"].as_u64().unwrap())
                .parse()
                .unwrap(),
        );
        let rt = tokio::runtime::Runtime::new().unwrap();
        rt.block_on(async {
            let desk =
                build_endpoint(EndpointConfig::new(DESK_SEED).with_relay(RelayConfig::Disabled))
                    .await
                    .unwrap();
            let connection = tokio::time::timeout(LIMIT, dial(&desk, address, alpn::WIRE))
                .await
                .unwrap()
                .unwrap();
            let (mut send, mut recv) = open_frame_stream(&connection).await.unwrap();
            let mut catalogs = Vec::new();
            for _ in 0..2 {
                write_frame(
                    &mut send,
                    &serde_json::to_vec(&WireRequest::Catalog).unwrap(),
                )
                .await
                .unwrap();
                let bytes =
                    tokio::time::timeout(LIMIT, read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN))
                        .await
                        .unwrap()
                        .unwrap();
                let WireResponse::Catalog(catalog) = serde_json::from_slice(&bytes).unwrap() else {
                    panic!("authenticated catalog refused")
                };
                assert!(catalog.entries.iter().any(|row| row.topic == TOPIC));
                catalogs.push(bytes);
            }
            assert_eq!(catalogs[0], catalogs[1]);
            connection.close(0u32.into(), b"done");
            desk.close().await;
        });
        let desk_ix = IsolatedRoot::mint("d2");
        let desk_manager =
            TransportManager::init_for_test(TransportConfig::default(), desk_ix.config()).unwrap();
        let wan = format!(
            "producer={}@127.0.0.1:{}",
            facts["eid"].as_str().unwrap(),
            facts["iroh_port"].as_u64().unwrap()
        );
        let mut desk_daemon = Daemon::start(&desk_ix, false, false, Some(&wan), false);
        let mut consumer = desk_daemon.connect();
        let demanded = request(
            &mut consumer,
            Request::Demand {
                id: 1,
                robot: "producer".into(),
                topic: TOPIC.into(),
                schema_hash: 7,
            },
        );
        assert!(matches!(demanded, Response::Demand(_)), "{demanded:?}");
        let mut oracle = [0u8; WireHeader::SIZE + 3];
        let size = oracle.len() as u32;
        oracle[..8].copy_from_slice(&7u64.to_le_bytes());
        oracle[8..12].copy_from_slice(&size.to_le_bytes());
        oracle[12..16].copy_from_slice(&size.to_le_bytes());
        oracle[WireHeader::SIZE..].copy_from_slice(b"p2!");
        let mut tap = None;
        until(|| {
            publisher.publish_raw(&oracle).unwrap();
            tap = desk_manager.create_data_only_subscriber(TOPIC).ok();
            tap.is_some()
        });
        let mut tap = tap.unwrap();
        let mut received = Vec::new();
        until(|| {
            publisher.publish_raw(&oracle).unwrap();
            tap.drain_owned(1, &mut received).unwrap() > 0
        });
        assert_eq!(received[0].payload(), oracle);
        assert!(
            children(desk_daemon.child.id()).is_empty(),
            "desk delivers a remote frame without serving"
        );
        drop(consumer);
        desk_daemon.stop();
        assert!(matches!(
            request(&mut a, Request::ReleaseEgress { id: 2 }),
            Response::EgressRelease(_)
        ));
        assert_eq!(
            children(daemon.child.id()),
            server_pid,
            "second registration retains server"
        );
        let holds = cerulion_remoted::serving_hold::ServingHolds::open(
            &cerulion_remoted::serving_hold::fence_path(&daemon.root.path().join("robot-state")),
        )
        .unwrap();
        let shell = holds
            .acquire(cerulion_remoted::serving_hold::HoldReason::Shell)
            .unwrap();
        drop(b);
        std::thread::sleep(Duration::from_millis(300));
        assert_eq!(
            children(daemon.child.id()),
            server_pid,
            "a live work guard blocks idle reap after last egress disconnect"
        );
        drop(shell);
        until(|| children(daemon.child.id()).is_empty());
        assert!(matches!(
            request(&mut a, register(3, TOPIC)),
            Response::Egress(_)
        ));
        until(|| !children(daemon.child.id()).is_empty());
        assert!(matches!(
            request(&mut a, Request::ReleaseEgress { id: 4 }),
            Response::EgressRelease(_)
        ));
        until(|| children(daemon.child.id()).is_empty());
        assert!(matches!(
            request(&mut a, register(5, TOPIC)),
            Response::Egress(_)
        ));
        until(|| !children(daemon.child.id()).is_empty());
        let restarted_pid = children(daemon.child.id())[0];
        daemon.stop();
        assert!(!Command::new("kill")
            .args(["-0", &restarted_pid.to_string()])
            .stderr(Stdio::null())
            .status()
            .unwrap()
            .success());
    }
}

#[test]
fn local_only_and_query_only_daemons_never_start_the_server() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    for off in [true, false] {
        let ix = IsolatedRoot::mint("q2");
        let mut daemon = Daemon::start(&ix, off, true, None, false);
        let mut client = daemon.connect();
        let _snapshot = request(&mut client, Request::ServingSchemaSnapshot { id: 1 });
        assert!(matches!(
            request(&mut client, register(3, "   ")),
            Response::Error(_)
        ));
        assert!(children(daemon.child.id()).is_empty());
        if off {
            assert!(matches!(
                request(&mut client, register(2, TOPIC)),
                Response::Error(_)
            ));
        } else {
            std::fs::remove_file(daemon.root.path().join("auth.json")).unwrap();
            std::fs::write(
                daemon.root.path().join("robot-state/remoted/device_key"),
                [0x77; 32],
            )
            .unwrap();
            assert!(
                matches!(request(&mut client, register(2, TOPIC)), Response::Error(_)),
                "a mismatched paired key without login must refuse before spawning"
            );
        }
        assert!(children(daemon.child.id()).is_empty());
        daemon.stop();
    }
}

#[test]
fn explicit_listen_retains_the_server_after_last_release_until_shutdown() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let ix = IsolatedRoot::mint("s2");
    let mut daemon = Daemon::start(&ix, false, true, None, true);
    until(|| !children(daemon.child.id()).is_empty());
    let pid = children(daemon.child.id())[0];
    let mut client = daemon.connect();
    assert!(matches!(
        request(&mut client, register(1, TOPIC)),
        Response::Egress(_)
    ));
    assert!(matches!(
        request(&mut client, Request::ReleaseEgress { id: 2 }),
        Response::EgressRelease(_)
    ));
    assert_eq!(children(daemon.child.id()), vec![pid]);
    daemon.stop();
    assert!(!Command::new("kill")
        .args(["-0", &pid.to_string()])
        .stderr(Stdio::null())
        .status()
        .unwrap()
        .success());
}

#[test]
fn engaged_ops_estop_survives_last_egress_disconnect_and_remains_observable() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let ix = IsolatedRoot::mint("e2");
    let mut daemon = Daemon::start(&ix, false, false, None, false);
    let mut producer = daemon.connect();
    assert!(matches!(
        request(&mut producer, register(1, TOPIC)),
        Response::Egress(_)
    ));
    let facts_path = daemon
        .root
        .path()
        .join("robot-state/remoted/beacon_facts.json");
    until(|| facts_path.exists());
    let facts: serde_json::Value =
        serde_json::from_slice(&std::fs::read(facts_path).unwrap()).unwrap();
    let address = EndpointAddr::new(
        cerulion_link::EndpointId::from_bytes(
            &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
        )
        .unwrap(),
    )
    .with_ip_addr(
        format!("127.0.0.1:{}", facts["iroh_port"].as_u64().unwrap())
            .parse()
            .unwrap(),
    );
    let pid = children(daemon.child.id());
    assert_eq!(pid.len(), 1);
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let engaged = runtime.block_on(ops_call(
        address.clone(),
        "engage-estop",
        serde_json::json!({}),
    ));
    assert_eq!(engaged["status"], "ok", "{engaged}");
    assert_eq!(engaged["result"]["engaged"], true);
    drop(producer);
    std::thread::sleep(Duration::from_millis(500));
    assert_eq!(
        children(daemon.child.id()),
        pid,
        "idle cannot clear an engaged latch"
    );
    assert!(
        cerulion_netd::serving_hold::IdleFence::try_acquire(
            &cerulion_remoted::serving_hold::fence_path(&daemon.root.path().join("robot-state"))
        )
        .unwrap()
        .is_none(),
        "the engaged latch independently blocks exclusive idle admission"
    );
    // The shipped act verb reads the ops latch before opening any SHM publisher.
    for _ in 0..16 {
        let status = runtime.block_on(ops_call(
            address.clone(),
            "topic-publish",
            serde_json::json!({"topic":TOPIC,"type":"unknown_msgs/msg/Unknown","message":{}}),
        ));
        assert_eq!(status["status"], "err", "{status}");
        assert_eq!(
            status["message"],
            "verb error: unavailable: the robot's e-stop is engaged, so topic-publish is refused"
        );
    }
    daemon.stop();
}

#[test]
fn a_stalled_tls_handshake_cannot_block_a_second_clients_estop_delivery() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let ix = IsolatedRoot::mint("h2");
    let mut daemon = Daemon::start(&ix, false, false, None, false);
    let mut producer = daemon.connect();
    assert!(matches!(
        request(&mut producer, register(1, TOPIC)),
        Response::Egress(_)
    ));
    let facts_path = daemon
        .root
        .path()
        .join("robot-state/remoted/beacon_facts.json");
    until(|| facts_path.exists());
    let facts: serde_json::Value =
        serde_json::from_slice(&std::fs::read(facts_path).unwrap()).unwrap();
    let robot: std::net::SocketAddr = format!("127.0.0.1:{}", facts["iroh_port"].as_u64().unwrap())
        .parse()
        .unwrap();
    let id = cerulion_link::EndpointId::from_bytes(
        &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
    )
    .unwrap();
    // Forward genuine client datagrams but withhold the robot's replies. Seeing
    // a reply proves the real server began TLS; no protocol bytes are fabricated.
    let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
    let proxy_address = socket.local_addr().unwrap();
    socket
        .set_read_timeout(Some(Duration::from_millis(50)))
        .unwrap();
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let replied = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    struct Proxy {
        running: std::sync::Arc<std::sync::atomic::AtomicBool>,
        thread: Option<std::thread::JoinHandle<()>>,
    }
    impl Drop for Proxy {
        fn drop(&mut self) {
            self.running
                .store(false, std::sync::atomic::Ordering::Release);
            if let Some(thread) = self.thread.take() {
                thread.join().unwrap();
            }
        }
    }
    let live = std::sync::Arc::clone(&running);
    let response_seen = std::sync::Arc::clone(&replied);
    let proxy = Proxy {
        running,
        thread: Some(std::thread::spawn(move || {
            let mut packet = [0; 65536];
            while live.load(std::sync::atomic::Ordering::Acquire) {
                match socket.recv_from(&mut packet) {
                    Ok((_, from)) if from == robot => {
                        response_seen.store(true, std::sync::atomic::Ordering::Release)
                    }
                    Ok((len, _)) => {
                        socket.send_to(&packet[..len], robot).unwrap();
                    }
                    Err(error)
                        if matches!(
                            error.kind(),
                            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                        ) => {}
                    Err(error) => panic!("handshake proxy failed: {error}"),
                }
            }
        })),
    };
    let runtime = tokio::runtime::Runtime::new().unwrap();
    runtime.block_on(async {
        let stalled_peer =
            build_endpoint(EndpointConfig::new([0x91; 32]).with_relay(RelayConfig::Disabled))
                .await
                .unwrap();
        let peer = stalled_peer.clone();
        let stalled = tokio::spawn(async move {
            dial(
                &peer,
                EndpointAddr::new(id).with_ip_addr(proxy_address),
                alpn::OPS,
            )
            .await
        });
        tokio::time::timeout(LIMIT, async {
            while !replied.load(std::sync::atomic::Ordering::Acquire) {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .unwrap();
        let desk = build_endpoint(EndpointConfig::new(DESK_SEED).with_relay(RelayConfig::Disabled))
            .await
            .unwrap();
        let delivered = tokio::time::timeout(Duration::from_secs(2), async {
            let connection = dial(&desk, EndpointAddr::new(id).with_ip_addr(robot), alpn::OPS)
                .await
                .unwrap();
            let (mut send, mut recv) = open_frame_stream(&connection).await.unwrap();
            let ack = ops_exchange(
                &mut send,
                &mut recv,
                br#"{"min_version":1,"max_version":1}"#,
            )
            .await;
            assert_eq!(ack["type"], "ack");
            let engaged = ops_exchange(
                &mut send,
                &mut recv,
                br#"{"id":0,"verb":"engage-estop","args":{}}"#,
            )
            .await;
            (engaged, connection, send, recv)
        })
        .await;
        assert!(
            !stalled.is_finished(),
            "first peer is still waiting for TLS"
        );
        stalled.abort();
        let _ = stalled.await;
        stalled_peer.close().await;
        let (engaged, connection, _send, _recv) =
            delivered.expect("a pending TLS peer cannot delay e-stop delivery beyond two seconds");
        assert_eq!(engaged["status"], "ok", "{engaged}");
        assert_eq!(engaged["result"]["engaged"], true);
        connection.close(0u32.into(), b"done");
        desk.close().await;
    });
    drop(proxy);
    drop(producer);
    daemon.stop();
}

#[test]
fn a_stalled_flood_cannot_refuse_an_estop_dial() {
    use std::sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    };
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let ix = IsolatedRoot::mint("flood");
    // This also raises this process's file limit before creating 33 endpoints.
    let _manager =
        TransportManager::init_for_test(TransportConfig::default(), ix.config()).unwrap();
    let mut daemon = Daemon::start(&ix, false, false, None, false);
    let mut producer = daemon.connect();
    assert!(matches!(
        request(&mut producer, register(1, TOPIC)),
        Response::Egress(_)
    ));
    let facts = daemon
        .root
        .path()
        .join("robot-state/remoted/beacon_facts.json");
    until(|| facts.exists());
    let facts: serde_json::Value = serde_json::from_slice(&std::fs::read(facts).unwrap()).unwrap();
    let robot: std::net::SocketAddr = format!("127.0.0.1:{}", facts["iroh_port"].as_u64().unwrap())
        .parse()
        .unwrap();
    let id = cerulion_link::EndpointId::from_bytes(
        &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
    )
    .unwrap();
    struct WithheldReplies {
        live: Arc<AtomicBool>,
        replied: Arc<AtomicBool>,
        address: std::net::SocketAddr,
        thread: Option<std::thread::JoinHandle<()>>,
    }
    impl WithheldReplies {
        fn new(robot: std::net::SocketAddr) -> Self {
            let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            socket
                .set_read_timeout(Some(Duration::from_millis(20)))
                .unwrap();
            let address = socket.local_addr().unwrap();
            let live = Arc::new(AtomicBool::new(true));
            let replied = Arc::new(AtomicBool::new(false));
            let running = Arc::clone(&live);
            let response_seen = Arc::clone(&replied);
            let thread = std::thread::spawn(move || {
                let mut packet = [0; 65536];
                while running.load(Ordering::Acquire) {
                    match socket.recv_from(&mut packet) {
                        Ok((_, sender)) if sender == robot => {
                            response_seen.store(true, Ordering::Release);
                        }
                        Ok((len, _)) => {
                            socket.send_to(&packet[..len], robot).unwrap();
                        }
                        Err(error)
                            if matches!(
                                error.kind(),
                                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                            ) => {}
                        Err(error) => panic!("reply-withholding proxy failed: {error}"),
                    }
                }
            });
            Self {
                live,
                replied,
                address,
                thread: Some(thread),
            }
        }
    }
    impl Drop for WithheldReplies {
        fn drop(&mut self) {
            self.live.store(false, Ordering::Release);
            self.thread.take().unwrap().join().unwrap();
        }
    }
    let proxies: Vec<_> = (0..33).map(|_| WithheldReplies::new(robot)).collect();
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            let mut builders = tokio::task::JoinSet::new();
            for seed in 0x80..=0xa0 {
                builders.spawn(build_endpoint(
                    EndpointConfig::new([seed; 32]).with_relay(RelayConfig::Disabled),
                ));
            }
            let mut peers = Vec::new();
            while let Some(peer) = builders.join_next().await {
                peers.push(peer.unwrap().unwrap());
            }
            assert_eq!(peers.len(), 33);
            let desk =
                build_endpoint(EndpointConfig::new(DESK_SEED).with_relay(RelayConfig::Disabled))
                    .await
                    .unwrap();
            let mut stalled = tokio::task::JoinSet::new();
            for (peer, proxy) in peers.iter().zip(&proxies) {
                let peer = peer.clone();
                let address = EndpointAddr::new(id).with_ip_addr(proxy.address);
                stalled.spawn(async move { dial(&peer, address, alpn::OPS).await });
            }
            // All 33 genuine Initials have reached remoted and received a response.
            // Their replies are withheld, so none can complete TLS or address retry.
            tokio::time::timeout(Duration::from_secs(5), async {
                while !proxies
                    .iter()
                    .all(|proxy| proxy.replied.load(Ordering::Acquire))
                {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .expect("all stalled dials must reach the real robot before its TLS deadline");
            let started = Instant::now();
            let delivered = tokio::time::timeout(Duration::from_secs(5), async {
                let connection = dial(&desk, EndpointAddr::new(id).with_ip_addr(robot), alpn::OPS)
                    .await
                    .unwrap();
                let (mut send, mut recv) = open_frame_stream(&connection).await.unwrap();
                let ack = ops_exchange(
                    &mut send,
                    &mut recv,
                    br#"{"min_version":1,"max_version":1}"#,
                )
                .await;
                assert_eq!(ack["type"], "ack");
                let engaged = ops_exchange(
                    &mut send,
                    &mut recv,
                    br#"{"id":0,"verb":"engage-estop","args":{}}"#,
                )
                .await;
                assert_eq!(engaged["status"], "ok", "{engaged}");
                assert_eq!(engaged["result"]["engaged"], true);
                (connection, send, recv)
            })
            .await
            .expect("33 stalled dials cannot refuse or delay a paired e-stop beyond five seconds");
            assert!(started.elapsed() < Duration::from_secs(5));
            // These are our client tasks; the server never evicts pending TLS work.
            stalled.shutdown().await;
            for peer in peers {
                peer.close().await;
            }
            let (connection, _send, _recv) = delivered;
            connection.close(0u32.into(), b"done");
            desk.close().await;
        });
    drop(proxies);
    drop(producer);
    daemon.stop();
}

fn test_netd_binary() -> std::path::PathBuf {
    std::env::var_os("CERULION_TEST_NETD_BINARY")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| env!("CARGO_BIN_EXE_cerulion-netd").into())
}

struct ServerFixture {
    root: tempfile::TempDir,
    netd: std::path::PathBuf,
}

impl ServerFixture {
    fn new(script: &str) -> Self {
        use std::os::unix::fs::PermissionsExt;
        let root = tempfile::tempdir().unwrap();
        let netd = root.path().join("cerulion-netd");
        std::fs::copy(test_netd_binary(), &netd).unwrap();
        let remoted = root.path().join("cerulion-remoted");
        std::fs::write(&remoted, script).unwrap();
        std::fs::set_permissions(remoted, std::fs::Permissions::from_mode(0o700)).unwrap();
        Self { root, netd }
    }
}

#[test]
fn registration_during_a_six_second_reap_replies_promptly_and_serves_one_replacement() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    // The wrapper delays TERM forwarding to its own real remoted child. Its
    // marker proves the parent has committed a stop, without guessed timing.
    let fixture = ServerFixture::new(
        r#"#!/bin/sh
rm -f "$CERULION_TEST_FACTS"
"$CERULION_TEST_REAL_REMOTED" "$@" &
server_pid=$!
printf '%s' "$server_pid" > "$CERULION_TEST_CHILD_PID"
trap '
    : > "$CERULION_TEST_STOP_ENTERED"
    sleep 6
    kill -TERM "$server_pid"
    wait "$server_pid"
    exit 0
' TERM
wait "$server_pid"
"#,
    );
    let ix = IsolatedRoot::mint("reap");
    let manager = TransportManager::init_for_test(TransportConfig::default(), ix.config()).unwrap();
    let next_topic = "/automatic/replacement";
    let _publisher = manager
        .create_publisher_simple(next_topic, MaxSliceLen::const_new(64))
        .unwrap();
    let remoted = test_netd_binary().with_file_name("cerulion-remoted");
    let mut daemon = Daemon::start_with(
        &ix,
        false,
        false,
        None,
        false,
        Some(&fixture.netd),
        |command, root| {
            command
                .env("CERULION_TEST_REAL_REMOTED", remoted)
                .env("CERULION_TEST_STOP_ENTERED", root.join("stop-entered"))
                .env("CERULION_TEST_CHILD_PID", root.join("child-pid"))
                .env(
                    "CERULION_TEST_FACTS",
                    root.join("robot-state/remoted/beacon_facts.json"),
                )
                .env("CERULION_REMOTED_HARD_EXIT_MS", "9000");
        },
    );
    let mut first = daemon.connect();
    let mut next = daemon.connect();
    assert!(matches!(
        request(&mut first, register(1, TOPIC)),
        Response::Egress(_)
    ));
    let facts = daemon
        .root
        .path()
        .join("robot-state/remoted/beacon_facts.json");
    let child_pid = daemon.root.path().join("child-pid");
    until(|| facts.exists() && child_pid.exists());
    let old = children(daemon.child.id());
    assert_eq!(old.len(), 1);
    let old_actual = std::fs::read_to_string(&child_pid).unwrap();
    assert!(matches!(
        request(&mut first, Request::ReleaseEgress { id: 2 }),
        Response::EgressRelease(_)
    ));
    until(|| daemon.root.path().join("stop-entered").exists());
    next.1
        .get_mut()
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let started = Instant::now();
    assert!(matches!(
        request(&mut next, register(3, next_topic)),
        Response::Egress(_)
    ));
    assert!(
        started.elapsed() < Duration::from_secs(5),
        "registration exceeded the control roundtrip budget"
    );
    assert_eq!(
        children(daemon.child.id()),
        old,
        "a replacement cannot overlap the stopping owner"
    );
    until(|| {
        let current = children(daemon.child.id());
        current.len() == 1
            && current != old
            && std::fs::read_to_string(&child_pid).is_ok_and(|pid| pid != old_actual)
            && facts.exists()
    });
    let replacement = children(daemon.child.id());
    assert_eq!(replacement.len(), 1);
    assert_eq!(
        children(replacement[0]).len(),
        1,
        "exactly one real remoted serves behind the wrapper"
    );
    let facts: serde_json::Value = serde_json::from_slice(&std::fs::read(facts).unwrap()).unwrap();
    let address = EndpointAddr::new(
        cerulion_link::EndpointId::from_bytes(
            &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
        )
        .unwrap(),
    )
    .with_ip_addr(
        format!("127.0.0.1:{}", facts["iroh_port"].as_u64().unwrap())
            .parse()
            .unwrap(),
    );
    tokio::runtime::Runtime::new().unwrap().block_on(async {
        let desk = build_endpoint(EndpointConfig::new(DESK_SEED).with_relay(RelayConfig::Disabled))
            .await
            .unwrap();
        let connection = tokio::time::timeout(LIMIT, dial(&desk, address, alpn::WIRE))
            .await
            .unwrap()
            .unwrap();
        let (mut send, mut recv) = open_frame_stream(&connection).await.unwrap();
        write_frame(
            &mut send,
            &serde_json::to_vec(&WireRequest::Catalog).unwrap(),
        )
        .await
        .unwrap();
        let bytes = tokio::time::timeout(LIMIT, read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN))
            .await
            .unwrap()
            .unwrap();
        let WireResponse::Catalog(catalog) = serde_json::from_slice(&bytes).unwrap() else {
            panic!("replacement refused authenticated catalog")
        };
        assert!(catalog
            .entries
            .iter()
            .any(|entry| entry.topic == next_topic));
        connection.close(0u32.into(), b"done");
        desk.close().await;
    });
    daemon.stop();
    drop(fixture.root);
}

#[test]
fn registration_replies_while_endpoint_transfer_waits_on_the_controller() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let fixture_root = tempfile::tempdir().unwrap();
    let netd = fixture_root.path().join("cerulion-netd");
    std::fs::copy(std::env::current_exe().unwrap(), &netd).unwrap();
    std::os::unix::fs::symlink(
        test_netd_binary().with_file_name("cerulion-remoted"),
        fixture_root.path().join("cerulion-remoted"),
    )
    .unwrap();
    let ix = IsolatedRoot::mint("busy");
    let mut daemon =
        Daemon::start_with(&ix, false, false, None, false, Some(&netd), |command, _| {
            command
                .args([
                    "--ignored",
                    "--exact",
                    "endpoint_controller_fixture_process",
                ])
                .env(
                    "CERULION_TEST_IX_CONFIG",
                    serde_json::to_string(&ix.config()).unwrap(),
                );
        });
    let mut first = daemon.connect();
    let mut second = daemon.connect();
    first
        .1
        .get_mut()
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    second
        .1
        .get_mut()
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    writeln!(
        first.0,
        "{}",
        serde_json::to_string(&register(1, TOPIC)).unwrap()
    )
    .unwrap();
    until(|| daemon.root.path().join("transfer-entered").exists());
    let mut line = String::new();
    first.1.read_line(&mut line).unwrap();
    assert!(matches!(
        serde_json::from_str::<Response>(&line).unwrap(),
        Response::Egress(_)
    ));
    let started = Instant::now();
    assert!(matches!(
        request(&mut second, register(2, TOPIC)),
        Response::Egress(_)
    ));
    assert!(started.elapsed() < Duration::from_secs(5));
    assert!(!daemon.root.path().join("release-controller").exists());
    assert!(children(daemon.child.id()).is_empty());
    std::fs::write(daemon.root.path().join("release-controller"), b"").unwrap();
    until(|| {
        children(daemon.child.id()).len() == 1
            && daemon
                .root
                .path()
                .join("robot-state/remoted/beacon_facts.json")
                .exists()
    });
    daemon.stop();
}

#[test]
#[ignore = "subprocess fixture driven by registration_replies_while_endpoint_transfer_waits_on_the_controller"]
fn endpoint_controller_fixture_process() {
    use cerulion_netd::mirror::{GatewayMirrorPlane, MirrorError, MirrorPlane, MirrorRelease};
    use cerulion_netd::registry::TopicKey;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    struct HeldController {
        inner: GatewayMirrorPlane,
        gate: Arc<Mutex<()>>,
        root: std::path::PathBuf,
    }
    impl MirrorPlane for HeldController {
        fn begin_serving(&self) -> Result<(), String> {
            std::fs::write(self.root.join("transfer-entered"), b"").unwrap();
            let _controller = self.gate.lock().unwrap();
            self.inner.begin_serving()
        }
        fn ensure_mirror(&self, key: &TopicKey, schema_hash: u64) -> Result<(), MirrorError> {
            self.inner.ensure_mirror(key, schema_hash)
        }
        fn release_mirror(&self, key: &TopicKey) -> MirrorRelease {
            self.inner.release_mirror(key)
        }
    }
    let root = std::path::PathBuf::from(std::env::var_os("CERULION_HOME").unwrap());
    let ix = serde_json::from_str(&std::env::var("CERULION_TEST_IX_CONFIG").unwrap()).unwrap();
    let manager = TransportManager::init_for_test(
        TransportConfig {
            network: Some(cerulion_core::NetworkConfig {
                robot_identity: Some("controller-fixture".into()),
                ..Default::default()
            }),
            ..Default::default()
        },
        ix,
    )
    .unwrap();
    let gate = Arc::new(Mutex::new(()));
    let guard = gate.lock().unwrap();
    let mirror: Arc<dyn MirrorPlane> = Arc::new(HeldController {
        inner: GatewayMirrorPlane::new(Arc::clone(&manager)),
        gate: Arc::clone(&gate),
        root: root.clone(),
    });
    let egress = Arc::new(
        cerulion_netd::supervised_egress::SupervisedEgressPlane::new(
            Arc::new(cerulion_netd::egress::GatewayEgressPlane::new(manager)),
            Arc::clone(&mirror),
            false,
        ),
    );
    let running = Arc::new(AtomicBool::new(true));
    let signalled = Arc::clone(&running);
    ctrlc::set_handler(move || signalled.store(false, Ordering::Release)).unwrap();
    let mut daemon = cerulion_netd::daemon::start_with_planes(
        root.join("netd.sock"),
        mirror,
        egress.clone(),
        Arc::new(cerulion_netd::query::NoopQueryPlane),
        cerulion_netd::daemon::NetdConfig {
            idle_self_exit: false,
            ..Default::default()
        },
    )
    .unwrap();
    while running.load(Ordering::Acquire) && !root.join("release-controller").exists() {
        std::thread::sleep(Duration::from_millis(10));
    }
    drop(guard);
    while running.load(Ordering::Acquire) {
        std::thread::sleep(Duration::from_millis(10));
    }
    daemon.shutdown();
    egress.shutdown();
}

#[test]
fn a_failed_server_spawn_keeps_registration_and_releases_endpoint_fencing() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let fixture = ServerFixture::new("#!/nonexistent/cerulion-test-interpreter\n");
    let ix = IsolatedRoot::mint("spawn");
    let mut daemon = Daemon::start_with(
        &ix,
        false,
        false,
        None,
        false,
        Some(&fixture.netd),
        |_, _| {},
    );
    let mut first = daemon.connect();
    first
        .1
        .get_mut()
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    assert!(matches!(
        request(&mut first, register(1, TOPIC)),
        Response::Egress(_)
    ));
    until(|| {
        std::fs::read_to_string(daemon.root.path().join("stderr"))
            .unwrap()
            .contains("starting robot server")
    });
    assert!(children(daemon.child.id()).is_empty());
    let fence = cerulion_netd::serving_hold::IdleFence::try_acquire(
        &cerulion_netd::serving_hold::fence_path(&daemon.root.path().join("robot-state")),
    )
    .unwrap();
    assert!(
        fence.is_some(),
        "failed spawn must release its native transfer fence"
    );
    drop(fence);
    let mut second = daemon.connect();
    assert!(matches!(
        request(&mut second, register(2, TOPIC)),
        Response::Egress(_)
    ));
    let snapshot = request(&mut first, Request::ServingSchemaSnapshot { id: 3 });
    assert!(matches!(snapshot, Response::ServingSchemaSnapshot(_)));
    drop(first);
    drop(second);
    daemon.stop();
    drop(fixture.root);
}

/// Hold serving on `client` for `ttl_s` seconds; the reply names `state_root`
/// and grants the length asked for.
fn hold_serving(
    client: &mut (UnixStream, BufReader<UnixStream>),
    id: u64,
    ttl_s: u64,
    state_root: &Path,
) {
    match request(client, Request::HoldServing { id, ttl_s }) {
        Response::ServingHeld(held) => {
            assert_eq!(held.id, id);
            assert_eq!(held.ttl_s, ttl_s);
            assert_eq!(Path::new(&held.state_root), state_root);
        }
        other => panic!("serving hold refused: {other:?}"),
    }
}

/// Start the owned server for a fresh serving hold and return its process
/// and loopback address, read from the beacon facts this server writes.
fn served_by_hold(
    daemon: &Daemon,
    holder: &mut (UnixStream, BufReader<UnixStream>),
    id: u64,
) -> (Vec<u32>, EndpointAddr) {
    let state_root = daemon.root.path().join("robot-state");
    let facts = state_root.join("remoted/beacon_facts.json");
    let _ = std::fs::remove_file(&facts);
    hold_serving(holder, id, 900, &state_root);
    until(|| !children(daemon.child.id()).is_empty() && facts.exists());
    let server = children(daemon.child.id());
    assert_eq!(server.len(), 1);
    let facts: serde_json::Value = serde_json::from_slice(&std::fs::read(facts).unwrap()).unwrap();
    let address = EndpointAddr::new(
        cerulion_link::EndpointId::from_bytes(
            &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
        )
        .unwrap(),
    )
    .with_ip_addr(
        format!("127.0.0.1:{}", facts["iroh_port"].as_u64().unwrap())
            .parse()
            .unwrap(),
    );
    (server, address)
}

/// A serving hold lapses at its deadline although peers without paired
/// authority keep connections to the owned server open: two overlapping
/// unpaired ops connections that never open a stream and a refused rcp
/// connection do not fence the idle reap, so the server is reaped while the
/// hold's own connection stays open, before remoted's handshake deadline
/// would drop the refused one. A paired ops session does fence it: the
/// server keeps serving past the deadline until that session closes, then it
/// is reaped.
#[test]
fn a_lapsed_serving_hold_reaps_past_unpaired_peers_and_waits_for_a_paired_session() {
    let _device = PAIRED_DEVICE.lock().unwrap_or_else(|e| e.into_inner());
    let ix = IsolatedRoot::mint("hold");
    let mut daemon = Daemon::start(&ix, false, false, None, false);
    let state_root = daemon.root.path().join("robot-state");
    let fence = cerulion_netd::serving_hold::fence_path(&state_root);
    let mut holder = daemon.connect();
    let runtime = tokio::runtime::Runtime::new().unwrap();

    // The paired control: an ops session open across the deadline.
    let (server, address) = served_by_hold(&daemon, &mut holder, 1);
    let (desk, session, mut send, mut recv) = runtime.block_on(async {
        let desk = build_endpoint(EndpointConfig::new(DESK_SEED).with_relay(RelayConfig::Disabled))
            .await
            .unwrap();
        let session = tokio::time::timeout(LIMIT, dial(&desk, address, alpn::OPS))
            .await
            .unwrap()
            .unwrap();
        let (mut send, mut recv) = open_frame_stream(&session).await.unwrap();
        let ack = tokio::time::timeout(
            LIMIT,
            ops_exchange(
                &mut send,
                &mut recv,
                br#"{"min_version":1,"max_version":1}"#,
            ),
        )
        .await
        .unwrap();
        assert_eq!(ack["type"], "ack", "{ack}");
        (desk, session, send, recv)
    });
    hold_serving(&mut holder, 2, 1, &state_root);
    // netd's deadline is at most one second after this reply; its worker
    // re-decides every 100 ms, so two seconds past it is twenty passes.
    let past = Instant::now() + Duration::from_secs(3);
    while Instant::now() < past {
        assert_eq!(
            children(daemon.child.id()),
            server,
            "a paired ops session keeps the server past the hold's deadline"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(
        cerulion_netd::serving_hold::IdleFence::try_acquire(&fence)
            .unwrap()
            .is_none(),
        "the paired session holds the serving fence"
    );
    let inventory = runtime.block_on(async {
        tokio::time::timeout(
            LIMIT,
            ops_exchange(
                &mut send,
                &mut recv,
                br#"{"id":1,"verb":"inventory","args":{}}"#,
            ),
        )
        .await
        .unwrap()
    });
    assert_eq!(inventory["status"], "ok", "{inventory}");
    runtime.block_on(async {
        drop((send, recv));
        session.close(0u32.into(), b"done");
        desk.close().await;
    });
    until(|| children(daemon.child.id()).is_empty());

    // Peers without paired authority: two overlapping unpaired ops connections
    // and a refused rcp connection, none of which opens a stream. The rcp one
    // is dialed last; remoted closes it once its control-frame handshake
    // deadline passes, so the reap must come before that.
    let (_server, address) = served_by_hold(&daemon, &mut holder, 3);
    let (strangers, connections, rcp_dialed) = runtime.block_on(async {
        let mut strangers = Vec::new();
        let mut connections = Vec::new();
        for (seed, plane) in [(0x51, alpn::OPS), (0x52, alpn::OPS), (0x53, alpn::RCP)] {
            let stranger =
                build_endpoint(EndpointConfig::new([seed; 32]).with_relay(RelayConfig::Disabled))
                    .await
                    .unwrap();
            let connection = tokio::time::timeout(LIMIT, dial(&stranger, address.clone(), plane))
                .await
                .unwrap()
                .unwrap();
            strangers.push(stranger);
            connections.push(connection);
        }
        (strangers, connections, Instant::now())
    });
    assert!(
        connections.iter().all(|c| c.close_reason().is_none()),
        "every unpaired connection is open before the hold lapses"
    );
    hold_serving(&mut holder, 4, 1, &state_root);
    until(|| children(daemon.child.id()).is_empty());
    assert!(
        rcp_dialed.elapsed() < REMOTED_HANDSHAKE_DEADLINE,
        "the server is reaped while the refused rcp connection is inside its control-frame \
         handshake deadline, after {:?}",
        rcp_dialed.elapsed()
    );
    assert!(
        matches!(
            request(&mut holder, Request::Status { id: 5 }),
            Response::Status(_)
        ),
        "the hold's connection stays open after the reap"
    );
    runtime.block_on(async {
        for connection in connections {
            connection.close(0u32.into(), b"done");
        }
        for stranger in strangers {
            stranger.close().await;
        }
    });
    drop(holder);
    daemon.stop();
}
