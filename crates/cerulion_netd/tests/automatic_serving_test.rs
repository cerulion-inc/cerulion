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
        let binary = std::env::var_os("CERULION_TEST_NETD_BINARY")
            .unwrap_or_else(|| env!("CARGO_BIN_EXE_cerulion-netd").into());
        let mut command = Command::new(binary);
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
