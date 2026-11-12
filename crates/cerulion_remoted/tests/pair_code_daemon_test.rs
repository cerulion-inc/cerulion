// SPDX-License-Identifier: AGPL-3.0-only
//! The pair-code handoff of the real `cerulion-remoted` binary
//! (`CARGO_BIN_EXE_cerulion-remoted`), its stderr read in order:
//!
//! - the handoff files left before the start are removed before the daemon logs
//!   that it is listening, and a request left right after that line is armed:
//!   its result reads `armed`, and the daemon holds the serving fence that
//!   keeps it from being stopped for idle, until a pairing or an expiry is
//!   written or the code's window plus a margin passes (a code whose tries
//!   are all used up holds it to then);
//! - once a stop request started the stop, a request left while the shutdown
//!   stage runs is not armed, though the pollers still run;
//! - a code armed before a stop request is withdrawn before the shutdown stage
//!   runs, and its result says `refused` with the reason `stopping`.
//!
//! Each daemon serves its own temp state root and device key, relays disabled;
//! no peer connects. Parallel-safe.
#![cfg(unix)]

mod common;

use std::os::unix::fs::PermissionsExt as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use cerulion_netd::serving_hold::IdleFence;
use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, RootSet};
use cerulion_pairing::pair_code::{
    is_handoff_file, PairCodeNonce, PairCodeRefusal, PairCodeRequest, PairCodeResult,
    PairCodeStatus, REQUEST_FILE, REQUEST_STAGING_PREFIX, RESULT_FILE,
};
use cerulion_pairing::verify::TrustStore;
use cerulion_remoted::log_writer::QUEUE_LINES_FOR_TEST_ENV as QUEUE_ENV;
use cerulion_remoted::pair_code_request::WITHDRAWN_LINE;
use cerulion_remoted::shutdown::{
    CLOSE_ENDPOINT_FOR_TEST_ENV as CLOSE_ENV, EXIT_LOG_GRACE_FOR_TEST_ENV as GRACE_ENV,
    HARD_EXIT_ENV, PROBE_FOR_TEST_ENV as PROBE_ENV,
};
use common::stderr_pipe::{pump, read_matching, KillOnDrop};

const MAC_KEY: &[u8] = b"pair-code-daemon-mac-key";
const CHASSIS: &[u8] = b"pair-code-daemon-chassis-secret-not-serial-derived";
const T_NOW: u64 = 1_000_000_000_000;
const CODE: &str = "39271846";
const NONCE: PairCodeNonce = PairCodeNonce::from_bytes([0x3C; 16]);
const EARLIER_NONCE: PairCodeNonce = PairCodeNonce::from_bytes([0x11; 16]);
/// How long a daemon may take to log that it listens.
const START_BOUND: Duration = Duration::from_secs(60);
/// How long a step of the stop or an arm may take before the test fails.
const STEP_BOUND: Duration = Duration::from_secs(20);
const LISTENING: &str = "cerulion_remoted: listening";
const REMOVED_AT_START: &str =
    "cerulion_remoted: removed the pair-code files left before this daemon reported listening";
const PROBE_RAN: &str = "the shutdown probe runs";

/// A provisioned, claimed state root whose trust store is bound to `secret`'s
/// public key, with the remoted directory owner-only; returns that public key.
fn write_state(root: &Path, secret: [u8; 32]) -> [u8; 32] {
    let remoted = root.join("remoted");
    std::fs::create_dir_all(&remoted).unwrap();
    std::fs::set_permissions(&remoted, std::fs::Permissions::from_mode(0o700)).unwrap();
    std::fs::write(remoted.join("device_key"), secret).unwrap();
    std::fs::write(remoted.join("trust_store.mac_key"), MAC_KEY).unwrap();
    let transport_key = ed25519_dalek::SigningKey::from_bytes(&secret)
        .verifying_key()
        .to_bytes();
    let roots = RootSet::new(vec![PublicKey([1; 32])], 1).unwrap();
    let mut store = TrustStore::provision(
        RobotId([6; 32]),
        PublicKey(transport_key),
        roots,
        CHASSIS,
        T_NOW,
    )
    .unwrap()
    .with_path(remoted.join("trust_store"));
    store
        .claim(AccountId([10; 32]), CHASSIS, PrincipalKind::Human, T_NOW)
        .unwrap();
    store.save(MAC_KEY).unwrap();
    transport_key
}

/// Leave a request as the step on the robot does: a 0600 staging file renamed
/// into place.
fn leave_request(remoted: &Path, nonce: PairCodeNonce) -> PathBuf {
    let staging = remoted.join(format!("{REQUEST_STAGING_PREFIX}1.tmp"));
    std::fs::write(
        &staging,
        PairCodeRequest::new(nonce, CODE).unwrap().render(),
    )
    .unwrap();
    std::fs::set_permissions(&staging, std::fs::Permissions::from_mode(0o600)).unwrap();
    let request = remoted.join(REQUEST_FILE);
    std::fs::rename(&staging, &request).unwrap();
    request
}

/// The handoff files in `remoted`, sorted.
fn handoff_files(remoted: &Path) -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(remoted)
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .filter(|name| is_handoff_file(name))
        .collect();
    names.sort();
    names
}

/// A running daemon and the stderr lines read from it; killed and reaped on drop.
struct Daemon {
    child: KillOnDrop,
    lines: mpsc::Receiver<String>,
    log: Vec<String>,
    remoted: PathBuf,
    endpoint_id: [u8; 32],
    _root: tempfile::TempDir,
}

impl Daemon {
    /// Start a daemon on the device key `[seed; 32]` with `env`, after `before`
    /// has run over its remoted directory.
    fn start(seed: u8, env: &[(&str, &str)], before: impl FnOnce(&Path)) -> Daemon {
        let root = tempfile::tempdir().unwrap();
        let endpoint_id = write_state(root.path(), [seed; 32]);
        let remoted = root.path().join("remoted");
        before(&remoted);
        let mut child = KillOnDrop(
            Command::new(env!("CARGO_BIN_EXE_cerulion-remoted"))
                .arg("--state-root")
                .arg(root.path())
                .arg("--relay-disabled")
                .env("NO_COLOR", "1")
                .env("RUST_LOG", "info")
                .env_remove("CERULION_NETWORK")
                .env_remove("CERULION_STATE_ROOT")
                .env_remove("CERULION_RELAY_URL")
                .env_remove(HARD_EXIT_ENV)
                .env_remove(PROBE_ENV)
                .env_remove(CLOSE_ENV)
                .env_remove(GRACE_ENV)
                .env_remove(QUEUE_ENV)
                .envs(env.iter().copied())
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::piped())
                .spawn()
                .expect("spawn cerulion-remoted"),
        );
        let stderr = child.0.stderr.take().unwrap();
        let (sender, lines) = mpsc::channel();
        std::thread::spawn(move || pump(stderr, sender, None, LISTENING));
        Daemon {
            child,
            lines,
            log: Vec::new(),
            remoted,
            endpoint_id,
            _root: root,
        }
    }

    /// The index of the first stderr line containing `needle`, read within
    /// `bound`.
    fn wait_for_line(&mut self, needle: &str, bound: Duration) -> usize {
        read_matching(&self.lines, &mut self.log, 0, bound, |l| l.contains(needle));
        self.line_index(&[needle]).unwrap()
    }

    /// The index of the first line read containing every one of `needles`.
    fn line_index(&self, needles: &[&str]) -> Option<usize> {
        self.log
            .iter()
            .position(|line| needles.iter().all(|n| line.contains(n)))
    }

    /// Send SIGTERM through `kill(1)`.
    fn terminate(&self) {
        let status = Command::new("kill")
            .arg("-TERM")
            .arg(self.child.0.id().to_string())
            .status()
            .expect("run kill");
        assert!(status.success(), "kill -TERM: {status:?}");
    }

    fn dump(&self) -> String {
        self.log.join("\n")
    }
}

fn until(what: &str, bound: Duration, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + bound;
    while !done() {
        assert!(Instant::now() < deadline, "{what}: not within {bound:?}");
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn files_left_before_the_start_are_removed_before_the_daemon_listens_and_a_request_after_is_armed()
{
    let mut daemon = Daemon::start(0x5A, &[], |remoted| {
        leave_request(remoted, EARLIER_NONCE);
        std::fs::write(remoted.join(".pair-code.request.9.tmp"), b"left").unwrap();
    });
    let listening = daemon.wait_for_line(LISTENING, START_BOUND);
    // Both files are gone once the listening line is read, and the line that
    // says so came before it.
    assert_eq!(handoff_files(&daemon.remoted), Vec::<String>::new());
    let removed = daemon
        .line_index(&[REMOVED_AT_START, "removed=2"])
        .unwrap_or_else(|| panic!("no removal line; log:\n{}", daemon.dump()));
    assert!(
        removed < listening,
        "the removal is logged before the daemon listens; log:\n{}",
        daemon.dump()
    );

    // Idle: nothing holds the serving fence.
    let fence = cerulion_remoted::serving_hold::fence_path(daemon.remoted.parent().unwrap());
    assert!(
        IdleFence::try_acquire(&fence).unwrap().is_some(),
        "an idle daemon holds no work; log:\n{}",
        daemon.dump()
    );

    // A request left right after the listening line is this run's to arm.
    let request = leave_request(&daemon.remoted, NONCE);
    let result = daemon.remoted.join(RESULT_FILE);
    until("the daemon answers the request", STEP_BOUND, || {
        result.exists()
    });
    let armed = PairCodeResult::parse(&std::fs::read(&result).unwrap()).expect("a result line");
    assert_eq!(armed.nonce(), Some(NONCE));
    assert_eq!(armed.endpoint_id(), &daemon.endpoint_id);
    assert_eq!(
        (armed.status(), armed.ttl_s(), armed.tries(), armed.failed()),
        (PairCodeStatus::Armed, 600, 3, 0)
    );
    assert!(!request.exists(), "the request is removed");
    // The armed code keeps the daemon serving: the fence is held.
    assert!(
        IdleFence::try_acquire(&fence).unwrap().is_none(),
        "an armed code holds the serving fence; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn a_code_armed_before_a_stop_request_is_withdrawn_before_the_shutdown_stage_runs() {
    let mut daemon = Daemon::start(0x5C, &[(PROBE_ENV, "stall")], |_| {});
    daemon.wait_for_line(LISTENING, START_BOUND);
    leave_request(&daemon.remoted, NONCE);
    let result = daemon.remoted.join(RESULT_FILE);
    let read = |result: &Path| {
        std::fs::read(result)
            .ok()
            .and_then(|bytes| PairCodeResult::parse(&bytes).ok())
    };
    until("the daemon arms the code", STEP_BOUND, || {
        read(&result).is_some_and(|r| r.status() == PairCodeStatus::Armed)
    });
    daemon.terminate();
    // The stage runs and never finishes; the code was withdrawn before it.
    daemon.wait_for_line(PROBE_RAN, STEP_BOUND);
    let withdrawn = read(&result).expect("a result line");
    assert_eq!(withdrawn.nonce(), Some(NONCE));
    assert_eq!(withdrawn.refusal(), Some(PairCodeRefusal::Stopping));
    assert_eq!(
        (withdrawn.ttl_s(), withdrawn.tries(), withdrawn.failed()),
        (600, 3, 0)
    );
    let line = daemon
        .line_index(&[WITHDRAWN_LINE])
        .unwrap_or_else(|| panic!("no withdrawal line; log:\n{}", daemon.dump()));
    assert!(
        line < daemon.line_index(&[PROBE_RAN]).unwrap(),
        "the code is withdrawn before the stage runs; log:\n{}",
        daemon.dump()
    );
}

#[test]
fn a_request_left_once_a_stop_request_started_the_stop_is_not_armed() {
    let mut daemon = Daemon::start(0x5B, &[(PROBE_ENV, "stall")], |_| {});
    daemon.wait_for_line(LISTENING, START_BOUND);
    daemon.terminate();
    // The stage runs and never finishes: the pollers are not stopped yet.
    daemon.wait_for_line(PROBE_RAN, STEP_BOUND);
    let request = leave_request(&daemon.remoted, NONCE);
    std::thread::sleep(Duration::from_millis(2_500));
    assert!(
        daemon.child.0.try_wait().unwrap().is_none(),
        "the daemon still runs its stage; log:\n{}",
        daemon.dump()
    );
    assert!(request.exists(), "the request is left where it is");
    assert!(
        !daemon.remoted.join(RESULT_FILE).exists(),
        "nothing was armed; log:\n{}",
        daemon.dump()
    );
}
