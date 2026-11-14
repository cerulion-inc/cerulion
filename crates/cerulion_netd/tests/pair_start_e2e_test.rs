// SPDX-License-Identifier: AGPL-3.0-only
//! Code pairing end to end: the robot runs `cerulion pair --start` against the
//! real netd and remoted binaries, over loopback with relays disabled. A code
//! pairs the one desk whose real `cerulion pair` types it; a remoted started
//! again refuses the real `cerulion connect` of a desk it never paired, and
//! the paired desk's `cerulion connect` reads a topic with its saved key and
//! no code. Three wrong codes typed into three desks' `cerulion pair` use a
//! code up, and a fourth desk's `cerulion pair` with the right code is
//! refused as used up. The stop of netd that ends each test checks that every
//! remoted the test saw netd run is gone with it.
//!
//! Prerequisite, in the same target and profile:
//! `cargo build -p cerulion_cli -p cerulion_remoted -p cerulion_connectd`.
#![cfg(all(feature = "wan", unix))]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::{DirBuilderExt, PermissionsExt};
use std::os::unix::net::UnixStream;
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::testing::IsolatedRoot;
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{
    GatewayEgressPolicy, GatewayPlan, SchemaServing, TransportConfig, TransportManager,
};
use cerulion_netd::protocol::{Hello, Request, Response};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{
    AccountId, PrincipalKind, PublicKey, RobotId, Role, RootSet, Scope,
};
use cerulion_pairing::verify::{AccessRow, PairingSource, TrustStore};
use cerulion_remoted::DeviceAccountIndex;

const TOPIC: &str = "/pair_start/oracle";
/// RFC 8032 section 7.1, test 2: the robot's device key seed, and its public
/// key, which is the robot's endpoint id.
const ROBOT_SEED: &str = "4ccd089b28ff96da9db6c346ec114e0f5b8a319f35aba624da8cf6ed4fb8a6fb";
const ROBOT_EID: &str = "3d4017c3e843895a92b70aa74d1b7ebc9c982ccf2ec4968cc0cd55f12af4660c";
/// RFC 8032 section 7.1, test 1: the paired desk's key seed, and its public
/// key, which is the account a code pairing binds for that desk.
const DESK_SEED: &str = "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60";
const DESK_ACCOUNT: &str = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
const OWNER: AccountId = AccountId([0x43; 32]);
const OWNER_DESK_SEED: [u8; 32] = [0x42; 32];
const MAC: [u8; 32] = [0x44; 32];
const LABEL: &str = "desk-e2e";
/// A sign-in that expired: the desk's login gate proceeds on it with no
/// request to the account service.
const EXPIRED_LOGIN: &[u8] = br#"{"account_id":"offline","session_token":"","refresh_token":"","expires_at_ns":0,"logged_in_ever":true}"#;
const WAITING_NOTE: &str = "Waiting for cerulion-remoted to start serving. A first start on this \
                            robot registers it with your account and can take up to 2 minutes.";
/// Part of the refusal of a try that comes less than 2 s after the robot's
/// last admitted try.
const THROTTLED: &str = "this robot takes one code pairing try every 2 s";
/// The bound on each wait for an effect whose cause has happened: netd
/// listening once started and exiting once signalled, a line on its control
/// socket, remoted serving once a topic is registered and reaped once nothing
/// holds serving, the robot's `paired:` line once its desk paired, a command
/// exiting once it printed its last line, its stdin closed or it was
/// signalled, a frame delivered once the desk's tap is open, and both pipes
/// closing once a command exited. Dropping a netd that still runs waits twice
/// this for its exit.
const LIMIT: Duration = Duration::from_secs(15);
/// The bound on the robot printing its code, which starts the serving daemon
/// first.
const ARM_LIMIT: Duration = Duration::from_secs(60);
/// The bound on a desk's `cerulion pair` or `cerulion connect` exiting, and
/// on the topic a reading desk re-injects appearing: each of the desk
/// helper's own waits on the robot is 30 s, and `cerulion pair` makes three
/// (the dial, the stream and the ceremony).
const DESK_LIMIT: Duration = Duration::from_secs(100);
/// The bound on the robot reporting how a desk's try ended once that desk's
/// `cerulion pair` exited. The desk helper closes its endpoint before it
/// exits, so the robot's session for it ends at once; a connection that is
/// only dropped ends there at the 30 s QUIC idle timeout, past this bound.
const TRY_LIMIT: Duration = Duration::from_secs(15);
/// The most runs of `cerulion pair` one wrong-code try makes: a run refused
/// as throttled is made again 250 ms later.
const THROTTLED_RUNS: usize = 40;

fn seed(hex_seed: &str) -> [u8; 32] {
    hex::decode(hex_seed).unwrap().try_into().unwrap()
}

/// The netd binary cargo built for this test. The other binaries the test
/// runs are its siblings, each named through `with_file_name`.
fn netd_binary() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_cerulion-netd"))
}

fn mode(path: &Path) -> u32 {
    std::fs::symlink_metadata(path)
        .unwrap()
        .permissions()
        .mode()
        & 0o777
}

/// The direct children of `pid`, or `None` when `pgrep` cannot list them;
/// `pgrep` exits 1 when it matched none.
fn listed_children(pid: u32) -> Option<Vec<u32>> {
    let output = Command::new("pgrep")
        .args(["-P", &pid.to_string()])
        .output()
        .ok()?;
    if !output.status.success() && output.status.code() != Some(1) {
        return None;
    }
    String::from_utf8(output.stdout)
        .ok()?
        .split_whitespace()
        .map(|pid| pid.parse().ok())
        .collect()
}

fn children(pid: u32) -> Vec<u32> {
    listed_children(pid).expect("pgrep lists the children")
}

/// `pid` as a signal target; child ids are positive.
fn target(pid: u32) -> Option<rustix::process::Pid> {
    i32::try_from(pid)
        .ok()
        .and_then(rustix::process::Pid::from_raw)
}

/// Whether a process with `pid` exists that this user may signal, a zombie
/// included.
fn exists(pid: u32) -> bool {
    target(pid).is_some_and(|pid| rustix::process::test_kill_process(pid).is_ok())
}

/// Wait up to `limit` for `ready`; a timeout names `what` and the caller's
/// line.
#[track_caller]
fn until(limit: Duration, what: &str, mut ready: impl FnMut() -> bool) {
    let deadline = Instant::now() + limit;
    while !ready() {
        assert!(Instant::now() < deadline, "{what}: not within {limit:?}");
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn write_iceoryx_config(dir: &Path, ix: &IsolatedRoot) {
    std::fs::create_dir_all(dir.join("config")).unwrap();
    std::fs::write(
        dir.join("config/iceoryx2.toml"),
        format!(
            "[global]\nroot-path = {:?}\nprefix = {:?}\n",
            ix.root, ix.prefix
        ),
    )
    .unwrap();
}

/// A claimed robot under `root`: its device key, a trust store claimed by
/// [`OWNER`] and a device index binding the owner's desk, both saved with
/// [`MAC`]. The remoted directory is owner-only, as the code handoff needs.
fn claimed_robot_state(root: &Path) {
    let dir = root.join("robot-state/remoted");
    std::fs::DirBuilder::new()
        .recursive(true)
        .mode(0o700)
        .create(&dir)
        .unwrap();
    let robot_seed = seed(ROBOT_SEED);
    std::fs::write(dir.join("device_key"), robot_seed).unwrap();
    std::fs::write(dir.join("trust_store.mac_key"), MAC).unwrap();
    let mut store = TrustStore::provision(
        RobotId([0x45; 32]),
        DeviceIdentity::from_seed(&robot_seed).public_key(),
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
    index.bind(
        DeviceIdentity::from_seed(&OWNER_DESK_SEED).public_key(),
        OWNER,
    );
    index.save(&MAC).unwrap();
}

/// The real `cerulion-netd` serving a claimed robot from a fresh root, which
/// is also the robot's `CERULION_HOME`.
struct Netd {
    child: Child,
    socket: PathBuf,
    root: tempfile::TempDir,
    /// Every server the test saw netd run, in the order first seen, recorded
    /// by each listing of netd's children: once netd is gone a server it left
    /// is no longer its child.
    seen: Vec<u32>,
}

impl Netd {
    fn start(ix: &IsolatedRoot) -> Self {
        let root = tempfile::tempdir().unwrap();
        claimed_robot_state(root.path());
        write_iceoryx_config(root.path(), ix);
        let socket = root.path().join("netd.sock");
        let mut command = Command::new(env!("CARGO_BIN_EXE_cerulion-netd"));
        command
            .current_dir(root.path())
            .env("CERULION_HOME", root.path())
            .env("CERULION_NETD_SOCK", &socket)
            .env("CERULION_NETD_IDLE_GRACE_MS", "60000")
            .env("CERULION_NETD_RELAY_DISABLED", "1")
            .env("RUST_LOG", "warn,cerulion_remoted=info")
            .env_remove("CERULION_STATE_ROOT")
            .env_remove("CERULION_NETWORK")
            .env_remove("CERULION_NETD_NETWORK")
            .env_remove("CERULION_NETD_LISTEN")
            .env_remove("CERULION_NETD_CONNECT")
            .env_remove("CERULION_NETD_WAN_ROBOTS")
            .env_remove("CERULION_NETD_DESK_KEY")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(std::fs::File::create(root.path().join("stderr")).unwrap());
        let child = command.spawn().unwrap();
        let mut netd = Self {
            child,
            socket,
            root,
            seen: Vec::new(),
        };
        until(LIMIT, "netd listening", || {
            assert!(netd.child.try_wait().unwrap().is_none(), "{}", netd.log());
            netd.socket.exists()
        });
        netd
    }

    fn log(&self) -> String {
        std::fs::read_to_string(self.root.path().join("stderr")).unwrap_or_default()
    }

    fn remoted_dir(&self) -> PathBuf {
        self.root.path().join("robot-state/remoted")
    }

    /// The servers netd runs now, each recorded in [`Netd::seen`].
    fn servers(&mut self) -> Vec<u32> {
        let servers = children(self.child.id());
        self.record(&servers);
        servers
    }

    fn record(&mut self, servers: &[u32]) {
        for &pid in servers {
            if !self.seen.contains(&pid) {
                self.seen.push(pid);
            }
        }
    }

    fn connect(&self) -> Control {
        let stream = UnixStream::connect(&self.socket).unwrap();
        stream.set_read_timeout(Some(LIMIT)).unwrap();
        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut line = String::new();
        reader.read_line(&mut line).unwrap();
        let _: Hello = serde_json::from_str(&line).unwrap();
        (stream, reader)
    }

    /// The port in the beacon facts the running remoted wrote, which name
    /// this robot's endpoint.
    fn facts_port(&self) -> u16 {
        let facts: serde_json::Value = serde_json::from_slice(
            &std::fs::read(self.remoted_dir().join("beacon_facts.json")).unwrap(),
        )
        .unwrap();
        assert_eq!(facts["eid"], ROBOT_EID, "{facts}");
        u16::try_from(facts["iroh_port"].as_u64().unwrap()).unwrap()
    }

    /// The robot's access rows, read from disk with [`MAC`].
    fn access_rows(&self) -> Vec<AccessRow> {
        TrustStore::load(self.remoted_dir().join("trust_store"), &MAC)
            .unwrap()
            .access_rows()
            .to_vec()
    }

    fn device_index(&self) -> DeviceAccountIndex {
        DeviceAccountIndex::load(self.remoted_dir().join("device_index.json"), &MAC).unwrap()
    }

    /// The result line the robot left, its nonce checked as 32 lowercase hex
    /// digits and written `<nonce>`; the file is owner-only.
    fn result_line(&self) -> String {
        let path = self.remoted_dir().join("pair-code.result");
        assert_eq!(mode(&path), 0o600);
        let text = std::fs::read_to_string(&path).unwrap();
        let fields: Vec<&str> = text.splitn(3, ' ').collect();
        assert_eq!(fields.len(), 3, "{text:?}");
        assert!(
            fields[1].len() == 32
                && fields[1]
                    .bytes()
                    .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)),
            "{text:?}"
        );
        format!("{} <nonce> {}", fields[0], fields[2])
    }

    /// Stop netd with SIGTERM and return the servers it ran, listed before
    /// the signal: netd exits within [`LIMIT`] and removes its socket, and no
    /// server in [`Netd::seen`] exists any more, since netd stops and reaps
    /// the server it owns before it exits.
    #[must_use]
    fn stop(&mut self) -> Vec<u32> {
        let servers = self.servers();
        assert!(Command::new("kill")
            .args(["-TERM", &self.child.id().to_string()])
            .status()
            .unwrap()
            .success());
        until(LIMIT, "netd stopped", || {
            self.child.try_wait().unwrap().is_some()
        });
        // Netd's log is printed by the drop of a panicking test.
        for &pid in &self.seen {
            assert!(!exists(pid), "server {pid} outlived netd");
        }
        assert!(!self.socket.exists());
        servers
    }
}

impl Drop for Netd {
    fn drop(&mut self) {
        if std::thread::panicking() {
            eprintln!("{}", self.log());
        }
        if self.child.try_wait().ok().flatten().is_none() {
            let servers = listed_children(self.child.id()).unwrap_or_default();
            self.record(&servers);
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
        // A server netd left behind, also one of a netd that exited on its
        // own, leads its own process group: end the group and the server.
        for pid in self
            .seen
            .iter()
            .filter(|&&pid| exists(pid))
            .filter_map(|&pid| target(pid))
        {
            let _ = rustix::process::kill_process_group(pid, rustix::process::Signal::KILL);
            let _ = rustix::process::kill_process(pid, rustix::process::Signal::KILL);
        }
    }
}

type Control = (UnixStream, BufReader<UnixStream>);

fn request(control: &mut Control, request: Request) -> Response {
    writeln!(control.0, "{}", request.to_json_line()).unwrap();
    let mut line = String::new();
    assert!(control.1.read_line(&mut line).unwrap() > 0);
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

/// A frame of schema hash 7 carrying `payload`, written field by field.
fn frame(payload: &[u8]) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    let size = u32::try_from(frame.len()).unwrap();
    frame[..8].copy_from_slice(&7u64.to_le_bytes());
    frame[8..12].copy_from_slice(&size.to_le_bytes());
    frame[12..16].copy_from_slice(&size.to_le_bytes());
    frame[WireHeader::SIZE..].copy_from_slice(payload);
    frame
}

/// A `cerulion` command for the machine whose `HOME` and `CERULION_HOME`
/// are `home`: no telemetry, no account service, no netd started by the
/// command itself, and the real login gate.
fn cli(home: &Path, socket: &Path, args: &[&str]) -> Command {
    let binary = netd_binary().with_file_name("cerulion");
    assert!(
        binary.is_file(),
        "build the CLI first: cargo build -p cerulion_cli"
    );
    assert!(
        netd_binary().with_file_name("cerulion-connectd").is_file(),
        "build the desk helper first: cargo build -p cerulion_connectd"
    );
    let mut command = Command::new(binary);
    command
        .args(args)
        .current_dir(home)
        .env("HOME", home)
        .env("CERULION_HOME", home)
        .env("CERULION_NETD_SOCK", socket)
        // The netd binary named here does not exist, so a command that tried
        // to start its own netd would fail instead.
        .env("CERULION_NETD_BIN", home.join("absent-cerulion-netd"))
        .env("CERULION_TELEMETRY", "off")
        .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:9")
        .env("NO_COLOR", "1")
        .env_remove("CERULION_LOGIN_GATE")
        .env_remove("CERULION_STATE_ROOT")
        .env_remove("CERULION_NETWORK")
        .env_remove("CERULION_NETD_NETWORK")
        .env_remove("CERULION_CONNECTD_BIN")
        .env_remove("RUST_LOG");
    command
}

/// One `cerulion` process in its own process group, its stdout read line by
/// line and its stderr collected. Unless the test reaped it and read both
/// pipes to their end, its whole group is stopped on drop, and it is reaped.
struct CliRun {
    child: Child,
    /// Reaped, and both pipes read to their end.
    drained: bool,
    stdin: Option<ChildStdin>,
    lines: Receiver<String>,
    stdout: Vec<String>,
    stderr: Arc<Mutex<String>>,
    stderr_closed: Receiver<()>,
}

impl CliRun {
    fn spawn(mut command: Command) -> Self {
        command
            .process_group(0)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = command.spawn().unwrap();
        let stdin = child.stdin.take();
        let out = child.stdout.take().unwrap();
        let err = child.stderr.take().unwrap();
        let (sender, lines) = mpsc::channel();
        std::thread::spawn(move || {
            for line in BufReader::new(out).lines() {
                let Ok(line) = line else { break };
                if sender.send(line).is_err() {
                    break;
                }
            }
        });
        let stderr = Arc::new(Mutex::new(String::new()));
        let collected = stderr.clone();
        let (closed, stderr_closed) = mpsc::channel();
        std::thread::spawn(move || {
            for line in BufReader::new(err).lines() {
                let Ok(line) = line else { break };
                let mut text = collected.lock().unwrap();
                text.push_str(&line);
                text.push('\n');
            }
            let _ = closed.send(());
        });
        Self {
            child,
            drained: false,
            stdin,
            lines,
            stdout: Vec::new(),
            stderr,
            stderr_closed,
        }
    }

    fn stderr(&self) -> String {
        self.stderr.lock().unwrap().clone()
    }

    fn stdout(&self) -> String {
        self.stdout.iter().map(|line| format!("{line}\n")).collect()
    }

    /// The next stdout line, within `limit`.
    #[track_caller]
    fn next_line(&mut self, limit: Duration) -> String {
        match self.lines.recv_timeout(limit) {
            Ok(line) => {
                self.stdout.push(line.clone());
                line
            }
            Err(RecvTimeoutError::Timeout) => panic!(
                "no stdout line within {limit:?}; stdout:\n{}stderr:\n{}",
                self.stdout(),
                self.stderr()
            ),
            Err(RecvTimeoutError::Disconnected) => panic!(
                "stdout closed; stdout:\n{}stderr:\n{}",
                self.stdout(),
                self.stderr()
            ),
        }
    }

    fn write_line(&mut self, line: &str) {
        let stdin = self.stdin.as_mut().unwrap();
        stdin.write_all(line.as_bytes()).unwrap();
        stdin.write_all(b"\n").unwrap();
        stdin.flush().unwrap();
    }

    fn close_stdin(&mut self) {
        self.stdin = None;
    }

    #[track_caller]
    fn assert_running(&mut self) {
        assert!(
            self.child.try_wait().unwrap().is_none(),
            "exited early; stdout:\n{}stderr:\n{}",
            self.stdout(),
            self.stderr()
        );
    }

    /// The exit code, within `limit`, once both output streams closed.
    #[track_caller]
    fn wait(&mut self, limit: Duration) -> i32 {
        let deadline = Instant::now() + limit;
        let status = loop {
            if let Some(status) = self.child.try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < deadline,
                "still running after {limit:?}; stdout:\n{}stderr:\n{}",
                self.stdout(),
                self.stderr()
            );
            std::thread::sleep(Duration::from_millis(20));
        };
        let code = status
            .code()
            .unwrap_or_else(|| panic!("ended by a signal: {status:?}; stderr:\n{}", self.stderr()));
        loop {
            match self.lines.recv_timeout(LIMIT) {
                Ok(line) => self.stdout.push(line),
                Err(RecvTimeoutError::Disconnected) => break,
                Err(RecvTimeoutError::Timeout) => panic!("stdout stayed open after the exit"),
            }
        }
        assert!(
            self.stderr_closed.recv_timeout(LIMIT).is_ok(),
            "stderr stayed open after the exit"
        );
        self.drained = true;
        code
    }

    fn signal_group(&self, signal: rustix::process::Signal) {
        if let Some(pid) = target(self.child.id()) {
            let _ = rustix::process::kill_process_group(pid, signal);
        }
    }
}

impl Drop for CliRun {
    fn drop(&mut self) {
        if self.drained {
            return;
        }
        if std::thread::panicking() {
            eprintln!("stdout:\n{}stderr:\n{}", self.stdout(), self.stderr());
        }
        self.stdin = None;
        // A process the command started can hold a pipe open past the
        // command's reaped exit and keeps the group alive, so the group is
        // signalled whether or not the command was reaped; the wait below
        // ends at once for a reaped command.
        self.signal_group(rustix::process::Signal::TERM);
        let deadline = Instant::now() + LIMIT;
        while self.child.try_wait().ok().flatten().is_none() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(20));
        }
        self.signal_group(rustix::process::Signal::KILL);
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// `cerulion pair --start` on the robot `netd` serves.
fn pair_start(netd: &Netd) -> CliRun {
    let mut run = CliRun::spawn(cli(netd.root.path(), &netd.socket, &["pair", "--start"]));
    run.close_stdin();
    run
}

/// The code and port of a `pair-code:` line, checked field by field.
#[track_caller]
fn armed(line: &str) -> (String, u16) {
    let fields: Vec<&str> = line.split(' ').collect();
    assert_eq!(fields.len(), 6, "{line}");
    let code = fields[1].strip_prefix("code=").unwrap_or_default();
    assert!(
        code.len() == 8 && code.bytes().all(|b| b.is_ascii_digit()),
        "{line}"
    );
    let port: u16 = fields[3]
        .strip_prefix("port=")
        .and_then(|port| port.parse().ok())
        .unwrap_or_else(|| panic!("{line}"));
    assert_eq!(
        line,
        format!("pair-code: code={code} eid={ROBOT_EID} port={port} expires_in_s=600 tries=3")
    );
    (code.to_string(), port)
}

/// The robot's stderr after the lines that show the armed code, which are
/// checked: the asking line, the waiting note when it was shown, the code,
/// its expiry at an `HH:MM` wall-clock time, the desk hint with `port` and
/// the waiting line.
#[track_caller]
fn after_armed_lines(stderr: &str, code: &str, port: u16) -> Vec<String> {
    let mut lines: Vec<String> = stderr.lines().map(str::to_string).collect();
    assert_eq!(
        lines.first().map(String::as_str),
        Some("Asking cerulion-netd to serve this robot for code pairing."),
        "{stderr}"
    );
    lines.remove(0);
    if lines.first().map(String::as_str) == Some(WAITING_NOTE) {
        lines.remove(0);
    }
    assert!(lines.len() >= 4, "{stderr}");
    assert_eq!(lines[0], format!("Pairing code: {code}"), "{stderr}");
    let expiry = lines[1]
        .strip_prefix("It pairs one desk and expires at ")
        .and_then(|rest| rest.strip_suffix(" UTC (in 10 minutes). All desks share 3 tries."))
        .unwrap_or_else(|| panic!("{stderr}"));
    let digits: Vec<u32> = expiry
        .split(':')
        .map(|part| {
            assert_eq!(part.len(), 2, "{stderr}");
            part.parse().unwrap()
        })
        .collect();
    assert!(
        digits.len() == 2 && digits[0] < 24 && digits[1] < 60,
        "{stderr}"
    );
    assert_eq!(
        lines[2],
        format!(
            "On the desk, run `cerulion pair --eid {ROBOT_EID}` and type the code. Add \
             `--addr <robot-ip>:{port}` when the desk has no internet."
        ),
        "{stderr}"
    );
    assert_eq!(
        lines[3],
        "Waiting for a desk. Ctrl-C stops waiting; the code keeps working until it is used or \
         expires.",
        "{stderr}"
    );
    lines.split_off(4)
}

/// A desk machine: its own home, signed in once with a login that expired.
struct Desk {
    home: tempfile::TempDir,
}

impl Desk {
    fn new() -> Self {
        let home = tempfile::tempdir().unwrap();
        std::fs::write(home.path().join("auth.json"), EXPIRED_LOGIN).unwrap();
        Self { home }
    }

    fn command(&self, args: &[&str]) -> Command {
        cli(self.home.path(), &self.home.path().join("netd.sock"), args)
    }

    /// `cerulion pair` against the robot at `port` with `code` typed on
    /// stdin: its exit code, stdout and stderr.
    fn pair(&self, label: &str, port: u16, code: &str) -> (i32, String, String) {
        let addr = format!("127.0.0.1:{port}");
        let mut run = CliRun::spawn(self.command(&[
            "pair",
            "--eid",
            ROBOT_EID,
            "--addr",
            &addr,
            "--relay-disabled",
            "--label",
            label,
        ]));
        run.write_line(code);
        run.close_stdin();
        let exit = run.wait(DESK_LIMIT);
        (exit, run.stdout(), run.stderr())
    }

    /// One wrong-code try the robot admits: a run refused as throttled is
    /// run again. The desk sees the wrong code itself and exits 4, with only
    /// the `pairing:` state line on stdout.
    fn wrong_try(&self, label: &str, port: u16, code: &str) {
        for _ in 0..THROTTLED_RUNS {
            let (exit, stdout, stderr) = self.pair(label, port, code);
            assert_eq!(stdout, pairing_stdout(), "{stderr}");
            if exit == 2 && stderr.contains(THROTTLED) {
                std::thread::sleep(Duration::from_millis(250));
                continue;
            }
            assert_eq!(exit, 4, "{stderr}");
            assert!(
                stderr.contains("wrong pairing code: the pairing code did not match"),
                "{stderr}"
            );
            return;
        }
        panic!("every one of {THROTTLED_RUNS} runs was throttled");
    }

    /// The desk key `cerulion pair` left: `identity/` owner-only, and
    /// `identity/device.key` 32 bytes, owner read and write only, and the
    /// same bytes as the legacy `desk.key`.
    fn key(&self) -> [u8; 32] {
        let identity = self.home.path().join("identity");
        assert_eq!(mode(&identity), 0o700);
        let key = identity.join("device.key");
        assert_eq!(mode(&key), 0o600);
        let bytes = std::fs::read(&key).unwrap();
        assert_eq!(
            std::fs::read(self.home.path().join("desk.key")).unwrap(),
            bytes
        );
        bytes.try_into().unwrap()
    }
}

fn pairing_stdout() -> String {
    format!("pairing: robot={ROBOT_EID} eid={ROBOT_EID}\n")
}

#[track_caller]
fn assert_owner_row(row: &AccessRow) {
    assert_eq!(row.account, OWNER);
    assert_eq!(row.device, None);
    assert_eq!(row.source, PairingSource::Claim);
    assert!(!row.revoked);
}

/// A code made with `cerulion pair --start` pairs the desk that types it:
/// both sides print their `paired:` line, the robot saves an observe-only
/// row for the desk's key and binds that key to the desk's own account, and
/// the result reads `paired`. Once nothing holds serving, remoted is reaped; a
/// remoted started again reads the saved row from disk: a desk it never
/// paired is refused, and the paired desk's `cerulion connect`, with no
/// code, receives the robot's frame.
#[test]
fn a_pair_start_code_pairs_one_desk_and_the_saved_pairing_reads() {
    let desk_public = DeviceIdentity::from_seed(&seed(DESK_SEED)).public_key();
    assert_eq!(hex::encode(desk_public.0), DESK_ACCOUNT);
    assert_eq!(
        hex::encode(DeviceIdentity::from_seed(&seed(ROBOT_SEED)).public_key().0),
        ROBOT_EID
    );
    let ix = IsolatedRoot::mint("pse");
    let manager = TransportManager::init_for_test(TransportConfig::default(), ix.config()).unwrap();
    let mut publisher = manager
        .create_publisher_simple(TOPIC, MaxSliceLen::const_new(64))
        .unwrap();
    let mut netd = Netd::start(&ix);
    let mut control = netd.connect();
    assert!(netd.servers().is_empty(), "nothing serves before the start");

    // A desk that kept its key under the legacy name, readable by others.
    let desk = Desk::new();
    let legacy = desk.home.path().join("desk.key");
    std::fs::write(&legacy, seed(DESK_SEED)).unwrap();
    std::fs::set_permissions(&legacy, std::fs::Permissions::from_mode(0o644)).unwrap();

    let mut robot = pair_start(&netd);
    let line = robot.next_line(ARM_LIMIT);
    let (code, port) = armed(&line);
    assert_eq!(port, netd.facts_port());
    assert_eq!(netd.servers().len(), 1, "the start serves one remoted");

    let (exit, stdout, stderr) = desk.pair(LABEL, port, &code);
    assert_eq!(exit, 0, "{stderr}");
    assert_eq!(
        stdout,
        format!(
            "{}paired: robot={ROBOT_EID} eid={ROBOT_EID} account={DESK_ACCOUNT}\n",
            pairing_stdout()
        )
    );
    assert_eq!(
        robot.next_line(LIMIT),
        format!("paired: desk={LABEL} account={DESK_ACCOUNT}")
    );
    assert_eq!(robot.wait(LIMIT), 0);
    assert_eq!(
        robot.stdout(),
        format!("{line}\npaired: desk={LABEL} account={DESK_ACCOUNT}\n")
    );
    assert_eq!(
        after_armed_lines(&robot.stderr(), &code, port),
        [format!(
            "Paired with desk {LABEL} (account {DESK_ACCOUNT}). The code is used up."
        )]
    );
    assert_eq!(
        netd.result_line(),
        format!(
            "cerulion-pair-code-result/1 <nonce> paired - {ROBOT_EID} 600 3 0 {DESK_ACCOUNT} \
             {LABEL}\n"
        )
    );

    // The desk adopted its key into the identity layout, owner-only.
    assert_eq!(desk.key(), seed(DESK_SEED));

    let rows = netd.access_rows();
    assert_eq!(rows.len(), 2, "{rows:?}");
    assert_owner_row(rows.iter().find(|row| row.account == OWNER).unwrap());
    let row = rows
        .iter()
        .find(|row| row.account == AccountId(desk_public.0))
        .unwrap_or_else(|| panic!("no row for the desk: {rows:?}"));
    assert_eq!(row.device, Some(PublicKey(desk_public.0)));
    assert_eq!(
        row.scope,
        Scope {
            role: Role::VIEWER,
            caps: Scope::CAP_OBSERVE
        }
    );
    assert_eq!(row.principal_kind, PrincipalKind::Human);
    assert_eq!(row.source, PairingSource::CodePaired);
    assert_eq!(row.name.as_deref(), Some(LABEL));
    assert_eq!(row.expires_at_ns, None);
    assert!(!row.revoked);
    let index = netd.device_index();
    assert_eq!(index.len(), 2);
    assert_eq!(
        index.account_for(&desk_public),
        Some(AccountId(desk_public.0))
    );
    assert_eq!(
        index.account_for(&DeviceIdentity::from_seed(&OWNER_DESK_SEED).public_key()),
        Some(OWNER)
    );

    until(LIMIT, "remoted reaped once nothing holds serving", || {
        netd.servers().is_empty()
    });

    // The topic a graph run registers starts remoted again; its new beacon
    // facts name the port it bound.
    let facts = netd.remoted_dir().join("beacon_facts.json");
    match std::fs::remove_file(&facts) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        removed => removed.unwrap(),
    }
    assert!(matches!(
        request(&mut control, register(1, TOPIC)),
        Response::Egress(_)
    ));
    until(LIMIT, "remoted serving the topic", || {
        netd.servers().len() == 1 && facts.exists()
    });
    let addr = format!("127.0.0.1:{}", netd.facts_port());
    let connect = [
        "connect",
        "--eid",
        ROBOT_EID,
        "--addr",
        addr.as_str(),
        "--relay-disabled",
        "--topic",
        TOPIC,
    ];

    // A desk the robot never paired is refused the topic.
    let stranger = Desk::new();
    let stranger_ix = IsolatedRoot::mint("pss");
    write_iceoryx_config(stranger.home.path(), &stranger_ix);
    let mut refused = CliRun::spawn(stranger.command(&connect));
    assert_eq!(refused.wait(DESK_LIMIT), 1, "{}", refused.stderr());
    assert!(
        refused
            .stderr()
            .contains("the robot refused the wire plane"),
        "{}",
        refused.stderr()
    );

    let desk_ix = IsolatedRoot::mint("psd");
    write_iceoryx_config(desk.home.path(), &desk_ix);
    let desk_manager =
        TransportManager::init_for_test(TransportConfig::default(), desk_ix.config()).unwrap();
    // The desk helper stops when its stdin closes, so it stays open.
    let mut reader = CliRun::spawn(desk.command(&connect));
    let oracle = frame(b"paired read");
    let mut tap = None;
    until(DESK_LIMIT, "the desk's re-injected topic", || {
        reader.assert_running();
        publisher.publish_raw(&oracle).unwrap();
        tap = desk_manager.create_data_only_subscriber(TOPIC).ok();
        tap.is_some()
    });
    let mut tap = tap.unwrap();
    let mut received = Vec::new();
    until(LIMIT, "a frame delivered to the desk", || {
        publisher.publish_raw(&oracle).unwrap();
        tap.drain_owned(1, &mut received).unwrap() > 0
    });
    assert_eq!(received[0].payload(), oracle.as_slice());
    reader.close_stdin();
    assert_eq!(reader.wait(LIMIT), 0, "{}", reader.stderr());

    // The topic's registration on `control` holds the remoted serving it, so
    // netd is stopped while it is held: a close of `control` first is a last
    // release, which reaps that remoted.
    assert_eq!(netd.stop().len(), 1, "one remoted served until the stop");
    drop(control);
}

/// Three desks, each with a key `cerulion pair` creates, type a wrong code:
/// each exits 4, and once each has exited the robot prints the line for its
/// try, `try-failed:` for the first two and `exhausted:` for the third, then
/// exits 4 and tells the user to start again. The result reads `exhausted`
/// with 3 of 3 tries failed. The used-up code keeps the same remoted serving
/// after the robot's command exits, so a fourth desk with the right code
/// reaches it and is refused as used up, and no access row is written for any
/// desk.
#[test]
fn three_failed_tries_from_three_desks_use_the_code_up_loudly() {
    let ix = IsolatedRoot::mint("psx");
    let mut netd = Netd::start(&ix);
    let control = netd.connect();

    let mut robot = pair_start(&netd);
    let line = robot.next_line(ARM_LIMIT);
    let (code, port) = armed(&line);
    assert_eq!(port, netd.facts_port());
    let server = netd.servers();
    assert_eq!(server.len(), 1, "the start serves one remoted");
    let last = code.as_bytes()[7] - b'0';
    let wrong = format!("{}{}", &code[..7], (last + 1) % 10);

    let desks = [Desk::new(), Desk::new(), Desk::new()];
    for (failed, desk) in (1..).zip(&desks) {
        desk.wrong_try(&format!("desk-{failed}"), port, &wrong);
        let expected = if failed < 3 {
            format!("try-failed: failed={failed} tries=3")
        } else {
            "exhausted: failed=3 tries=3".to_string()
        };
        assert_eq!(robot.next_line(TRY_LIMIT), expected);
    }
    assert_eq!(robot.wait(LIMIT), 4);
    assert_eq!(
        robot.stdout(),
        format!(
            "{line}\ntry-failed: failed=1 tries=3\ntry-failed: failed=2 tries=3\nexhausted: \
             failed=3 tries=3\n"
        )
    );
    assert_eq!(
        after_armed_lines(&robot.stderr(), &code, port),
        [
            "A pairing try failed: 1 of 3 tries failed.",
            "A pairing try failed: 2 of 3 tries failed.",
            "The code was used up by 3 failed tries and no desk paired. Run `cerulion pair \
             --start` again for a new code.",
        ]
    );
    assert_eq!(
        netd.result_line(),
        format!("cerulion-pair-code-result/1 <nonce> exhausted - {ROBOT_EID} 600 3 3 - -\n")
    );

    // Nothing but the used-up code holds serving now.
    let fence = cerulion_netd::serving_hold::fence_path(&netd.root.path().join("robot-state"));
    assert!(
        cerulion_netd::serving_hold::IdleFence::try_acquire(&fence)
            .unwrap()
            .is_none(),
        "the used-up code keeps the serving fence taken"
    );
    assert_eq!(netd.servers(), server);
    let fourth = Desk::new();
    let (exit, stdout, stderr) = fourth.pair("desk-4", port, &code);
    assert_eq!(exit, 4, "{stderr}");
    assert_eq!(stdout, pairing_stdout());
    assert!(
        stderr.contains("code pairing rejected: pairing code burned: all 3 attempts exhausted"),
        "{stderr}"
    );
    assert_eq!(netd.servers(), server, "the same remoted answered");

    let rows = netd.access_rows();
    assert_eq!(rows.len(), 1, "{rows:?}");
    assert_owner_row(&rows[0]);
    let index = netd.device_index();
    assert_eq!(index.len(), 1);
    let keys: Vec<PublicKey> = desks
        .iter()
        .chain([&fourth])
        .map(|desk| DeviceIdentity::from_seed(&desk.key()).public_key())
        .collect();
    for (i, key) in keys.iter().enumerate() {
        assert_eq!(index.account_for(key), None);
        assert!(!keys[..i].contains(key), "each desk made its own key");
    }

    drop(control);
    assert_eq!(
        netd.stop(),
        server,
        "the same remoted served until the stop"
    );
}
