// SPDX-License-Identifier: AGPL-3.0-only
//! The real `cerulion-remoted` binary (`CARGO_BIN_EXE_cerulion-remoted`) with
//! its stderr pipe full:
//!
//! - an `estop` sent over `cerulion/rcp/1` reaches rcpd, and the desk reads
//!   its result, within 5 s;
//! - an ops `engage-estop` answers `engaged: true`, and the `estop` it sends
//!   reaches rcpd, within 5 s;
//! - the `cerulion/wire/2` `status` reply counts the log lines the daemon
//!   dropped; once the pipe drains, a warn line reports the drops and an info
//!   line reports that the writer caught up, naming the same total that
//!   `status` then reads.
//!
//! The pipe is the one #804's full-stderr stop test fills: the test reads
//! stderr up to the listening line, then reads nothing until it releases the
//! pipe. A stand-in rcpd (hand-rolled HTTP/1 on loopback) reports each
//! request it reads. The test seam's queue bound of one line makes a few log
//! lines overflow the daemon's log queue. Each daemon serves its own temp state
//! root and device key, relays disabled, with Flashback off; desks are paired
//! as the owner through the device index. Every wait is bounded.
#![cfg(unix)]

use std::io::{BufRead, BufReader, PipeReader, PipeWriter, Write};
use std::net::{Ipv4Addr, SocketAddr};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{self, RecvTimeoutError};
use std::time::{Duration, Instant};

use cerud::client::OpsClient;
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, read_frame, write_frame, Endpoint,
    EndpointAddr, EndpointConfig, EndpointId, QuicOpsStream, RecvStream, RelayConfig, SendStream,
    DEFAULT_MAX_FRAME_LEN,
};
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::DeviceAccountIndex;
use serde_json::{json, Value};

const MAC_KEY: &[u8] = b"full-stderr-mac-key";
const CHASSIS: &[u8] = b"full-stderr-chassis-secret-not-serial-derived";
const T_NOW: u64 = 1_000_000_000_000;
const OWNER: AccountId = AccountId([10; 32]);
const TOKEN: &str = "full-stderr-fixture-token";
/// The test seam that bounds the daemon's log queue in lines.
const QUEUE_LINES_ENV: &str = "CERULION_REMOTED_LOG_QUEUE_LINES_FOR_TEST";
/// How long a daemon may take to log that it listens.
const START_BOUND: Duration = Duration::from_secs(60);
/// How long a step that is not the oracle may take.
const STEP: Duration = Duration::from_secs(20);
/// How long an estop may take to reach rcpd while stderr is full.
const REACH: Duration = Duration::from_secs(5);
const LISTENING: &str = "cerulion_remoted: listening";
const DROPPED: &str = "cerulion_remoted: log lines were dropped";
const CAUGHT_UP: &str = "cerulion_remoted: the log writer caught up";

async fn bounded<F: std::future::Future>(what: &str, bound: Duration, fut: F) -> F::Output {
    tokio::time::timeout(bound, fut)
        .await
        .unwrap_or_else(|_| panic!("'{what}' did not finish within {bound:?}"))
}

/// The public key of the ed25519 secret `[seed; 32]`.
fn public(seed: u8) -> [u8; 32] {
    ed25519_dalek::SigningKey::from_bytes(&[seed; 32])
        .verifying_key()
        .to_bytes()
}

/// The `x-cerulion-caller` rcpd reads for the desk `seed` paired as the owner.
fn caller(seed: u8) -> String {
    format!(
        "iroh:{}/account:{}",
        hex::encode(public(seed)),
        hex::encode(OWNER.0)
    )
}

/// A claimed state root bound to the device key `[robot; 32]`, with each of
/// `desks` paired as the owner, and the rcpd token file.
fn write_state(root: &Path, robot: u8, desks: &[u8]) -> std::path::PathBuf {
    let remoted = root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    std::fs::write(remoted.join("device_key"), [robot; 32]).unwrap();
    std::fs::write(remoted.join("trust_store.mac_key"), MAC_KEY).unwrap();
    let roots = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    let mut store = TrustStore::provision(
        RobotId([5; 32]),
        PublicKey(public(robot)),
        roots,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(remoted.join("trust_store"));
    store
        .claim(OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC_KEY).unwrap();
    let mut index = DeviceAccountIndex::new().with_path(remoted.join("device_index.json"));
    for desk in desks {
        index.bind(PublicKey(public(*desk)), OWNER);
    }
    index.save(MAC_KEY).unwrap();
    let token_file = root.join("rcp.token");
    std::fs::write(&token_file, format!("CERULION_RCP_TOKEN={TOKEN}\n")).unwrap();
    use std::os::unix::fs::PermissionsExt;
    std::fs::set_permissions(&token_file, std::fs::Permissions::from_mode(0o600)).unwrap();
    token_file
}

/// One request the stand-in rcpd read.
#[derive(Debug, Clone)]
struct Seen {
    method: String,
    tool: String,
    caller: String,
}

/// A loopback stand-in for rcpd: `initialize` gets session `s1`, a
/// notification 202, an `estop` call the result rcpd's estop tool gives, a
/// `ping` an empty result, a `GET` 405 and a `DELETE` 200. Each request goes
/// on the receiver as soon as it is read.
async fn stand_in_rcpd() -> (SocketAddr, tokio::sync::mpsc::UnboundedReceiver<Seen>) {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    let (seen_tx, seen) = tokio::sync::mpsc::unbounded_channel();
    tokio::spawn(async move {
        loop {
            let Ok((mut tcp, _)) = listener.accept().await else {
                return;
            };
            let seen = seen_tx.clone();
            tokio::spawn(async move {
                let mut buf = Vec::new();
                let mut chunk = [0u8; 4096];
                let head_end = loop {
                    if let Some(at) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
                        break at + 4;
                    }
                    match tcp.read(&mut chunk).await {
                        Ok(0) | Err(_) => return,
                        Ok(n) => buf.extend_from_slice(&chunk[..n]),
                    }
                };
                let head = String::from_utf8_lossy(&buf[..head_end]).to_ascii_lowercase();
                let length: usize = head
                    .lines()
                    .find_map(|l| l.strip_prefix("content-length:"))
                    .map_or(0, |v| v.trim().parse().unwrap());
                while buf.len() < head_end + length {
                    match tcp.read(&mut chunk).await {
                        Ok(0) | Err(_) => return,
                        Ok(n) => buf.extend_from_slice(&chunk[..n]),
                    }
                }
                let body: Value = serde_json::from_slice(&buf[head_end..head_end + length])
                    .unwrap_or(Value::Null);
                let verb = head.split(' ').next().unwrap_or_default().to_string();
                let method = body["method"].as_str().unwrap_or_default().to_string();
                let tool = body["params"]["name"]
                    .as_str()
                    .unwrap_or_default()
                    .to_string();
                let caller = head
                    .lines()
                    .find_map(|l| l.strip_prefix("x-cerulion-caller:"))
                    .map(|v| v.trim().to_string())
                    .unwrap_or_default();
                let _ = seen.send(Seen {
                    method: method.clone(),
                    tool: tool.clone(),
                    caller,
                });
                let json = |result: Value| {
                    let body =
                        json!({ "jsonrpc": "2.0", "id": body["id"], "result": result }).to_string();
                    format!(
                        "HTTP/1.1 200 OK\r\ncontent-type: application/json\r\n\
                         mcp-session-id: s1\r\ncontent-length: {}\r\n\r\n{body}",
                        body.len()
                    )
                };
                let reply = match (verb.as_str(), method.as_str()) {
                    ("post", "initialize") => json(json!({
                        "protocolVersion": "2025-06-18",
                        "capabilities": {},
                        "serverInfo": { "name": "stand-in", "version": "0" }
                    })),
                    ("post", m) if m.starts_with("notifications/") => {
                        "HTTP/1.1 202 Accepted\r\ncontent-length: 0\r\n\r\n".to_string()
                    }
                    ("post", "tools/call") if tool == "estop" => json(json!({
                        "content": [{
                            "type": "text",
                            "text": "{\"estopped\":true,\"graph_acknowledged\":true}"
                        }]
                    })),
                    ("post", "ping") => json(json!({})),
                    ("get", _) => {
                        "HTTP/1.1 405 Method Not Allowed\r\ncontent-length: 0\r\n\r\n".to_string()
                    }
                    ("delete", _) => "HTTP/1.1 200 OK\r\ncontent-length: 0\r\n\r\n".to_string(),
                    _ => "HTTP/1.1 404 Not Found\r\ncontent-length: 0\r\n\r\n".to_string(),
                };
                let _ = tcp.write_all(reply.as_bytes()).await;
                let _ = tcp.shutdown().await;
            });
        }
    });
    (addr, seen)
}

/// The first request on `seen` that is the `estop` tool call `caller` sent,
/// within `bound`.
async fn estop_call_from(
    seen: &mut tokio::sync::mpsc::UnboundedReceiver<Seen>,
    caller: &str,
    bound: Duration,
) -> Seen {
    bounded("rcpd reads the estop call", bound, async {
        loop {
            let request = seen.recv().await.expect("the stand-in runs");
            if request.method == "tools/call" && request.tool == "estop" && request.caller == caller
            {
                return request;
            }
        }
    })
    .await
}

/// A child process, killed and reaped on drop.
struct KillOnDrop(Child);

impl Drop for KillOnDrop {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// A running daemon whose stderr the test holds full.
struct Daemon {
    _child: KillOnDrop,
    lines: mpsc::Receiver<String>,
    log: Vec<String>,
    /// While held, the stderr reader reads nothing past the listening line.
    hold: Option<mpsc::Sender<()>>,
    robot: EndpointAddr,
    _root: tempfile::TempDir,
}

/// Send each line of `stderr` to `lines`; after the listening line, read
/// nothing more, the pipe still open, until the hold's sender is dropped.
fn pump(stderr: PipeReader, lines: mpsc::Sender<String>, hold: mpsc::Receiver<()>) {
    for line in BufReader::new(stderr).lines().map_while(Result::ok) {
        let listening = line.contains(LISTENING);
        if lines.send(line).is_err() {
            return;
        }
        if listening {
            let _ = hold.recv();
        }
    }
}

/// Fill the pipe behind `writer` until it takes no more bytes. The write end
/// is non-blocking only while it fills: its file status flags are shared with
/// the daemon's stderr, which then blocks on its next write.
fn fill(writer: &PipeWriter) {
    use std::os::fd::AsRawFd;
    let fd = writer.as_raw_fd();
    // SAFETY: fcntl(2) reads and sets the flags of a descriptor this test owns.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    assert!(flags >= 0, "F_GETFL: {}", std::io::Error::last_os_error());
    // SAFETY: as above.
    assert_eq!(
        unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) },
        0
    );
    // Large chunks first, so a pipe that grows on a large write grows to its
    // largest size before it is full.
    let mut writer = writer;
    for chunk in [65_536, 1_024, 1] {
        let bytes = vec![b'x'; chunk];
        loop {
            match writer.write(&bytes) {
                Ok(_) => {}
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => panic!("filling the pipe: {e}"),
            }
        }
    }
    // SAFETY: as above.
    assert_eq!(unsafe { libc::fcntl(fd, libc::F_SETFL, flags) }, 0);
}

/// The first line of `log`, or else of `lines` (added to `log`), that
/// contains every one of `needles`, within `bound`.
fn read_until(
    lines: &mpsc::Receiver<String>,
    log: &mut Vec<String>,
    needles: &[&str],
    bound: Duration,
) -> String {
    let until = Instant::now() + bound;
    loop {
        if let Some(line) = log.iter().find(|l| needles.iter().all(|n| l.contains(n))) {
            return line.clone();
        }
        let left = until.saturating_duration_since(Instant::now());
        match lines.recv_timeout(left) {
            Ok(line) => log.push(line),
            Err(RecvTimeoutError::Timeout | RecvTimeoutError::Disconnected) => panic!(
                "no line with {needles:?} within {bound:?}; log:\n{}",
                log.join("\n")
            ),
        }
    }
}

/// The endpoint of the daemon serving `root`, on loopback at the port its
/// beacon facts name.
fn beacon_address(root: &Path) -> EndpointAddr {
    let facts = std::fs::read(root.join("remoted/beacon_facts.json"))
        .expect("the daemon wrote its beacon facts before it listened");
    let facts: Value = serde_json::from_slice(&facts).unwrap();
    let port = u16::try_from(facts["iroh_port"].as_u64().unwrap()).unwrap();
    let id: [u8; 32] = hex::decode(facts["eid"].as_str().unwrap())
        .unwrap()
        .try_into()
        .unwrap();
    direct_addr(
        EndpointId::from_bytes(&id).unwrap(),
        [SocketAddr::from((Ipv4Addr::LOCALHOST, port))],
    )
}

impl Daemon {
    /// Start a daemon on the device key `[robot; 32]`, with `desks` paired and
    /// rcp forwarded to `rcpd`; wait until it listens, then fill its stderr.
    fn start_with_full_stderr(robot: u8, desks: &[u8], rcpd: SocketAddr) -> Daemon {
        let root = tempfile::tempdir().unwrap();
        let token_file = write_state(root.path(), robot, desks);
        let (reader, writer) = std::io::pipe().unwrap();
        let filler = writer.try_clone().unwrap();
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-remoted"));
        command
            .arg("--state-root")
            .arg(root.path())
            .arg("--relay-disabled")
            .arg("--rcp-addr")
            .arg(rcpd.to_string())
            .arg("--rcp-token-file")
            .arg(&token_file)
            .env("NO_COLOR", "1")
            .env("RUST_LOG", "info")
            .env("CERULION_FLASHBACK", "off")
            .env(QUEUE_LINES_ENV, "1")
            .env_remove("CERULION_NETWORK")
            .env_remove("CERULION_STATE_ROOT")
            .env_remove("CERULION_RELAY_URL")
            .env_remove("CERULION_REMOTED_HARD_EXIT_MS")
            .env_remove("CERULION_REMOTED_SHUTDOWN_PROBE_FOR_TEST")
            .env_remove("CERULION_REMOTED_CLOSE_ENDPOINT_FOR_TEST")
            .env_remove("CERULION_REMOTED_EXIT_LOG_GRACE_MS_FOR_TEST")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(writer);
        let child = KillOnDrop(command.spawn().expect("spawn cerulion-remoted"));
        // The command's copy of the pipe's write end closes here.
        drop(command);
        let (sender, lines) = mpsc::channel();
        let (hold, held) = mpsc::channel();
        std::thread::spawn(move || pump(reader, sender, held));
        let mut log = Vec::new();
        read_until(&lines, &mut log, &[LISTENING], START_BOUND);
        let robot = beacon_address(root.path());
        fill(&filler);
        Daemon {
            _child: child,
            lines,
            log,
            hold: Some(hold),
            robot,
            _root: root,
        }
    }

    /// Read stderr again.
    fn release_stderr(&mut self) {
        self.hold = None;
    }

    /// The first stderr line containing every one of `needles`, read within
    /// `bound`.
    fn wait_for_line(&mut self, needles: &[&str], bound: Duration) -> String {
        read_until(&self.lines, &mut self.log, needles, bound)
    }

    /// Whether the pipe is still full: nothing past the listening line read.
    fn stderr_still_held(&self) -> bool {
        self.hold.is_some()
    }
}

async fn desk(seed: u8) -> Endpoint {
    bounded(
        "build_endpoint",
        STEP,
        build_endpoint(EndpointConfig::new([seed; 32]).with_relay(RelayConfig::Disabled)),
    )
    .await
    .expect("the desk endpoint binds")
}

async fn send_json(send: &mut SendStream, message: &Value) {
    bounded(
        "write_frame",
        STEP,
        write_frame(send, message.to_string().as_bytes()),
    )
    .await
    .expect("write a frame");
}

/// Frames read until one carries `id`; one `read_frame` per frame, each run
/// to completion or failing the test.
async fn reply(recv: &mut RecvStream, id: i64, bound: Duration) -> Value {
    bounded("the desk reads its reply", bound, async {
        loop {
            let frame = read_frame(recv, 16 << 20).await.expect("a reply frame");
            let message: Value = serde_json::from_slice(&frame).expect("JSON reply");
            if message["id"] == json!(id) {
                return message;
            }
        }
    })
    .await
}

/// The daemon's `status` reply over `cerulion/wire/2`, as JSON.
async fn status(desk: &Endpoint, robot: EndpointAddr) -> Value {
    let connection = bounded("dial wire", STEP, dial(desk, robot, alpn::WIRE))
        .await
        .expect("dial wire");
    let (mut send, mut recv) = bounded("open wire stream", STEP, open_frame_stream(&connection))
        .await
        .expect("open wire stream");
    send_json(&mut send, &json!({ "verb": "status" })).await;
    let frame = bounded(
        "the status reply",
        STEP,
        read_frame(&mut recv, DEFAULT_MAX_FRAME_LEN),
    )
    .await
    .expect("the status reply");
    let reply: Value = serde_json::from_slice(&frame).unwrap();
    assert_eq!(reply["reply"], json!("status"), "{reply}");
    reply
}

/// The value of the `name=` field of `line`.
fn field(line: &str, name: &str) -> u64 {
    let prefix = format!("{name}=");
    line.split_whitespace()
        .find_map(|word| word.strip_prefix(prefix.as_str()))
        .unwrap_or_else(|| panic!("no {name} in {line}"))
        .parse()
        .unwrap_or_else(|e| panic!("{name} in {line}: {e}"))
}

/// While stderr is still full, `status` counts dropped lines; once the pipe
/// drains, the drop line and the caught-up line are written, and `status`
/// reads the total the caught-up line names.
async fn drops_are_counted_then_reported(daemon: &mut Daemon, observer: &Endpoint) {
    let held = status(observer, daemon.robot.clone()).await;
    let while_held = held["log_lines_dropped"]
        .as_u64()
        .unwrap_or_else(|| panic!("no log_lines_dropped count in {held}"));
    assert!(
        while_held > 0,
        "with stderr full and a queue of one line, lines were dropped: {held}"
    );
    assert!(daemon.stderr_still_held());
    daemon.release_stderr();
    let dropped = daemon.wait_for_line(&[DROPPED, "WARN"], STEP);
    assert!(field(&dropped, "dropped") > 0, "{dropped}");
    let caught_up = daemon.wait_for_line(&[CAUGHT_UP, "INFO"], STEP);
    let total = field(&caught_up, "total_dropped");
    assert!(total >= while_held, "{caught_up} after {held}");
    let after = status(observer, daemon.robot.clone()).await;
    assert_eq!(
        after["log_lines_dropped"],
        json!(total),
        "status reads the total the caught-up line names; log:\n{}",
        daemon.log.join("\n")
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_rcp_estop_reaches_rcpd_while_stderr_is_full() {
    const ROBOT: u8 = 0x61;
    const DESK: u8 = 0x62;
    let (rcpd, mut seen) = stand_in_rcpd().await;
    let mut daemon = Daemon::start_with_full_stderr(ROBOT, &[DESK], rcpd);
    let desk = desk(DESK).await;
    let connection = bounded(
        "dial rcp",
        STEP,
        dial(&desk, daemon.robot.clone(), alpn::RCP),
    )
    .await
    .expect("dial rcp");
    let (mut send, mut recv) = bounded("open rcp stream", STEP, open_frame_stream(&connection))
        .await
        .expect("open rcp stream");
    // Sent one after another: the plane handles a stream's frames in order.
    let initialize = json!({ "jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18",
        "capabilities": {},
        "clientInfo": { "name": "full-stderr", "version": "0" }
    } });
    send_json(&mut send, &initialize).await;
    send_json(
        &mut send,
        &json!({ "jsonrpc": "2.0", "method": "notifications/initialized" }),
    )
    .await;
    let estop = json!({ "jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {
        "name": "estop", "arguments": { "reason": "full stderr" }
    } });
    send_json(&mut send, &estop).await;
    estop_call_from(&mut seen, &caller(DESK), REACH).await;
    let result = reply(&mut recv, 2, REACH).await;
    let text = result["result"]["content"][0]["text"]
        .as_str()
        .unwrap_or_else(|| panic!("the estop result: {result}"));
    assert_eq!(
        serde_json::from_str::<Value>(text).unwrap()["estopped"],
        json!(true)
    );
    assert!(
        daemon.stderr_still_held(),
        "the estop reached rcpd with stderr full"
    );
    drops_are_counted_then_reported(&mut daemon, &desk).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn an_ops_engage_estop_reaches_rcpd_while_stderr_is_full() {
    const ROBOT: u8 = 0x71;
    const DESK: u8 = 0x72;
    let (rcpd, mut seen) = stand_in_rcpd().await;
    let mut daemon = Daemon::start_with_full_stderr(ROBOT, &[DESK], rcpd);
    let desk = desk(DESK).await;
    let connection = bounded(
        "dial ops",
        STEP,
        dial(&desk, daemon.robot.clone(), alpn::OPS),
    )
    .await
    .expect("dial ops");
    let (send, recv) = bounded("open ops stream", STEP, open_frame_stream(&connection))
        .await
        .expect("open ops stream");
    let engaged = tokio::task::spawn_blocking(move || {
        let mut client =
            OpsClient::connect_current(QuicOpsStream::new(send, recv)).expect("handshake");
        client.call("engage-estop", json!({}))
    });
    estop_call_from(&mut seen, &caller(DESK), REACH).await;
    let answer = bounded("engage-estop answers", REACH, engaged)
        .await
        .expect("the ops task")
        .expect("engage-estop answers");
    assert_eq!(answer["engaged"], json!(true), "{answer}");
    assert!(
        daemon.stderr_still_held(),
        "the estop reached rcpd with stderr full"
    );
    drops_are_counted_then_reported(&mut daemon, &desk).await;
}
