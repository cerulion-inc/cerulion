// SPDX-License-Identifier: AGPL-3.0-only
//! A2 — the CLI's install-time robot registration end-to-end against the
//! REAL `cerulion_accountd` router on an ephemeral port (Principle #13: no mock
//! service — the actual A2 `POST /v1/robots` issuer serves every request).
//!
//! Process-test prerequisites: `cargo build -p cerulion_cli --bin cerulion
//! -p cerulion_remoted --bin cerulion-remoted`. Relay-disabled process tests
//! open no discovery or public relay connection.
//!
//! Each test spins the real account-service router, drives the blocking
//! [`login_cmd::run_login`] to a real session (authorizing the device code
//! out-of-band via the magic-link seam), then calls
//! [`robot_cmd::register_robot`] — the install-funnel entry — and asserts the
//! returned [`RobotOwnership`] binds the logged-in account, is idempotent for the
//! same machine, and refuses a stale session loudly.
//!
//! `#[serial]` — the tests mutate the process env (`CERULION_ACCOUNT_SERVICE` /
//! `CERULION_HOME`), which is global.

use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, EmailSender, ServiceConfig, UnconfiguredResolver,
};
use cerulion_cli_engine::auth::AuthState;
use cerulion_cli_engine::error::CliError;
use cerulion_cli_engine::{login_cmd, robot_cmd};
use serial_test::serial;

// ===========================================================================
// harness (self-contained, mirrors login_flow_e2e_test.rs)
// ===========================================================================

#[derive(Clone)]
struct SharedBuf(Arc<Mutex<Vec<u8>>>);
impl SharedBuf {
    fn new() -> Self {
        SharedBuf(Arc::new(Mutex::new(Vec::new())))
    }
    fn snapshot(&self) -> String {
        String::from_utf8_lossy(&self.0.lock().unwrap()).to_string()
    }
}
impl Write for SharedBuf {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

struct EnvGuard(&'static str, Option<String>);
impl EnvGuard {
    fn set(key: &'static str, val: &str) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, val);
        EnvGuard(key, prev)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match &self.1 {
            Some(v) => std::env::set_var(self.0, v),
            None => std::env::remove_var(self.0),
        }
    }
}

/// Start the REAL account-service router on an ephemeral 127.0.0.1 port. Returns
/// the bound port. The server thread is detached (it dies with the test process).
fn start_accountd(email: Arc<CapturingEmailSender>) -> u16 {
    let (tx, rx) = std::sync::mpsc::channel::<u16>();
    let email_dyn: Arc<dyn EmailSender> = email;
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async move {
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let bound = listener.local_addr().unwrap().port();
            let config = ServiceConfig {
                device_code_interval_secs: 0,
                verification_base_uri: format!("http://127.0.0.1:{bound}"),
                ..ServiceConfig::default()
            };
            let state = Arc::new(
                AppState::dev(
                    config,
                    Clock::System,
                    email_dyn,
                    Arc::new(UnconfiguredResolver),
                )
                .unwrap(),
            );
            tx.send(bound).unwrap();
            cerulion_accountd::serve(listener, state).await.unwrap();
        });
    });
    rx.recv_timeout(Duration::from_secs(10))
        .expect("accountd bound")
}

fn wait_for<T>(mut f: impl FnMut() -> Option<T>, timeout: Duration) -> Option<T> {
    let deadline = Instant::now() + timeout;
    loop {
        if let Some(v) = f() {
            return Some(v);
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

fn extract_user_code(prompt: &str) -> Option<String> {
    let re = regex::Regex::new(r"[BCDFGHJKLMNPQRSTVWXZ]{4}-[BCDFGHJKLMNPQRSTVWXZ]{4}").unwrap();
    re.find(prompt).map(|m| m.as_str().to_string())
}

fn authorize_via_magic_link(port: u16, email: &CapturingEmailSender, user_code: &str) {
    let base = format!("http://127.0.0.1:{port}");
    let client = reqwest::blocking::Client::new();
    let resp = client
        .post(format!("{base}/v1/auth/magic-link/start"))
        .json(&serde_json::json!({ "email": "owner@example.com", "user_code": user_code }))
        .send()
        .expect("magic-link/start reachable");
    assert!(
        resp.status().is_success(),
        "magic-link/start: {}",
        resp.status()
    );
    let link = wait_for(|| email.last_link(), Duration::from_secs(5)).expect("link captured");
    let resp = client
        .get(&link)
        .send()
        .expect("magic-link/complete reachable");
    assert!(
        resp.status().is_success(),
        "magic-link/complete: {}",
        resp.status()
    );
}

/// Log in for real against the running service and return the session-bearing
/// [`AuthState`] (writes `~/.cerulion/{auth.json, desk.key, device.cert}` under the
/// current `CERULION_HOME`).
fn login(port: u16, email: &Arc<CapturingEmailSender>) -> AuthState {
    let buf = SharedBuf::new();
    let worker = std::thread::spawn({
        let mut b = buf.clone();
        move || login_cmd::run_login(&mut b)
    });
    let code = wait_for(
        || extract_user_code(&buf.snapshot()),
        Duration::from_secs(15),
    )
    .expect("run_login printed a user_code");
    authorize_via_magic_link(port, email, &code);
    worker
        .join()
        .expect("worker thread")
        .expect("login succeeds")
}

// ===========================================================================
// tests
// ===========================================================================

#[test]
#[serial]
fn register_robot_binds_the_logged_in_account_and_returns_an_owner_grant() {
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let state = login(port, &email);
    let ownership = robot_cmd::register_robot(&state.session_token, "orin-lab-01")
        .expect("a fresh CLI install registers a robot owned by the installer");

    // The robot is owned by the LOGGED-IN account (== the install identity).
    assert_eq!(
        ownership.account_id, state.account_id,
        "the robot's owner is the installer's account"
    );
    // A real robot id + owner-credential blobs were returned (base64url, non-empty).
    assert!(!ownership.robot_id.is_empty());
    let robot_id = URL_SAFE_NO_PAD
        .decode(&ownership.robot_id)
        .expect("robot_id is base64url");
    assert_eq!(robot_id.len(), 32, "robot_id is a 32-byte RobotId");
    assert!(
        !URL_SAFE_NO_PAD
            .decode(&ownership.owner_grant)
            .expect("owner_grant is base64url")
            .is_empty(),
        "the owner grant is a non-empty offline-verifiable blob"
    );
    assert!(!URL_SAFE_NO_PAD
        .decode(&ownership.intermediate)
        .expect("intermediate is base64url")
        .is_empty());
}

#[test]
#[serial]
fn re_registering_the_same_machine_is_idempotent() {
    // A retried/re-run install (same machine = same device key = same transport
    // key) returns the SAME robot id — no second robot minted.
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let state = login(port, &email);
    let first = robot_cmd::register_robot(&state.session_token, "orin-lab-01").unwrap();
    let again = robot_cmd::register_robot(&state.session_token, "orin-renamed").unwrap();
    assert_eq!(
        first.robot_id, again.robot_id,
        "same machine ⇒ same robot id"
    );
    assert_eq!(first.account_id, again.account_id);
}

#[test]
#[serial]
fn register_robot_with_a_bad_session_is_refused_and_names_login() {
    // A stale/invalid session is refused server-side (401) — the error names the
    // fix (`cerulion login`), never a silent success.
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let err = robot_cmd::register_robot("not-a-real-session-token", "orin-lab-01")
        .expect_err("a bogus session must be refused");
    assert!(matches!(err, CliError::Login(_)), "got {err:?}");
    let msg = err.to_string();
    assert!(
        msg.contains("cerulion login"),
        "the refusal names the fix: {msg}"
    );
}

#[cfg(unix)]
#[test]
#[serial]
fn first_serve_registration_survives_offline_restart_and_rejects_account_switch() {
    use cerulion_cli_engine::{auth, robot_bootstrap};
    use cerulion_pairing::format::{PublicKey, RobotId, RootSet};
    use cerulion_pairing::verify::{PairingPresentation, TrustStore};

    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let home = tempfile::tempdir().unwrap();
    let home_path = home.path().canonicalize().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home_path.to_str().unwrap());
    let state = login(port, &email);
    let root = home_path.join("robot-state");
    let prepared =
        robot_bootstrap::prepare(&root).expect("real account service registers first serve");
    assert_eq!(
        prepared.owner_account,
        hex::encode(URL_SAFE_NO_PAD.decode(&state.account_id).unwrap())
    );
    let bytes = std::fs::read(&prepared.registration_bundle).unwrap();
    let bundle: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    assert!(
        bundle.get("device_seed").is_none(),
        "the handoff never duplicates a key seed"
    );
    assert_eq!(
        bundle["device_key_file"],
        home_path.join("desk.key").to_str().unwrap()
    );
    let decode = |field: &str| hex::decode(bundle[field].as_str().unwrap()).unwrap();
    let roots: RootSet = postcard::from_bytes(&decode("root_set_postcard")).unwrap();
    let robot = RobotId(hex::decode(&prepared.robot_id).unwrap().try_into().unwrap());
    let key = PublicKey(
        hex::decode(&prepared.endpoint_id)
            .unwrap()
            .try_into()
            .unwrap(),
    );
    let now = auth::now_unix_ns();
    let mut trust = TrustStore::provision(robot, key, roots, &[0x37; 32], now).unwrap();
    let presentation = PairingPresentation {
        intermediate: postcard::from_bytes(&decode("intermediate_postcard")).unwrap(),
        device_cert: postcard::from_bytes(&decode("device_cert_postcard")).unwrap(),
        grant: postcard::from_bytes(&decode("owner_grant_postcard")).unwrap(),
        delegation: None,
    };
    trust
        .claim_by_owner_grant(&presentation, &key, now)
        .expect("bundle verifies through production owner claim");
    assert!(trust.is_claimed());

    let mut expired = state.clone();
    expired.expires_at_ns = 0;
    auth::write_to(&home_path.join("auth.json"), &expired).unwrap();
    let _offline = EnvGuard::set("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    assert_eq!(
        robot_bootstrap::prepare(&root).unwrap(),
        prepared,
        "restart must need no cloud refresh"
    );
    assert_eq!(
        std::fs::read(&prepared.registration_bundle).unwrap(),
        bytes,
        "offline restart makes no registration write"
    );

    expired.account_id = URL_SAFE_NO_PAD.encode([0x79; 32]);
    auth::write_to(&home_path.join("auth.json"), &expired).unwrap();
    assert!(robot_bootstrap::prepare(&root)
        .unwrap_err()
        .to_string()
        .contains("does not match"));
    assert_eq!(
        std::fs::read(&prepared.registration_bundle).unwrap(),
        bytes,
        "account switch cannot rewrite the robot owner"
    );
}

#[cfg(unix)]
fn built_binary(name: &str) -> std::path::PathBuf {
    let current = std::env::current_exe().unwrap();
    let path = current.parent().unwrap().parent().unwrap().join(name);
    assert!(
        path.is_file(),
        "build the real process fixture first: {}",
        path.display()
    );
    path
}

#[cfg(unix)]
struct ProcessGuard(std::process::Child);

#[cfg(unix)]
impl Drop for ProcessGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[cfg(unix)]
fn bounded_output(command: &mut std::process::Command) -> std::process::Output {
    use std::process::Stdio;
    command
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = ProcessGuard(command.spawn().expect("start real worker"));
    let status = wait_for(|| child.0.try_wait().unwrap(), Duration::from_secs(15))
        .expect("worker exited within its test deadline");
    let mut stdout = Vec::new();
    let mut stderr = Vec::new();
    use std::io::Read;
    child
        .0
        .stdout
        .take()
        .unwrap()
        .read_to_end(&mut stdout)
        .unwrap();
    child
        .0
        .stderr
        .take()
        .unwrap()
        .read_to_end(&mut stderr)
        .unwrap();
    std::process::Output {
        status,
        stdout,
        stderr,
    }
}

#[cfg(unix)]
#[test]
#[serial]
fn real_background_worker_refuses_missing_login_without_interactive_flow() {
    use std::process::Command;
    let temp = tempfile::tempdir().unwrap();
    let home = temp.path().canonicalize().unwrap();
    let output = bounded_output(
        Command::new(built_binary("cerulion"))
            .arg("bootstrap-robot")
            .arg("--state-root")
            .arg(home.join("robot-state"))
            .env("CERULION_HOME", &home)
            .env("CERULION_LOGIN_GATE", "1")
            .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1"),
    );
    assert_eq!(output.status.code(), Some(1));
    assert!(
        output.stdout.is_empty(),
        "failure cannot impersonate a machine result"
    );
    let error = String::from_utf8(output.stderr).unwrap();
    assert!(
        error.contains("run `cerulion login` before serving a robot"),
        "{error}"
    );
    assert!(!home.join("auth.json").exists());
    assert!(!home.join("desk.key").exists());
    assert!(!home.join("robot-state").exists());
    let help = bounded_output(
        Command::new(built_binary("cerulion"))
            .arg("--help")
            .env("CERULION_HOME", &home),
    );
    assert!(help.status.success());
    assert!(!String::from_utf8(help.stdout)
        .unwrap()
        .contains("bootstrap-robot"));
}

#[cfg(unix)]
#[test]
#[serial]
fn real_worker_and_writer_publish_claimed_state_and_serve_the_registered_key_offline() {
    use cerulion_cli_engine::{auth, robot_bootstrap::BootstrapPrepared};
    use cerulion_pairing::format::AccountId;
    use cerulion_pairing::verify::TrustStore;
    use std::process::{Command, Stdio};

    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let temp = tempfile::tempdir().unwrap();
    let home = temp.path().canonicalize().unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.to_str().unwrap());
    let _service = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let state = login(port, &email);
    let root = home.join("robot-state");
    let output = bounded_output(
        Command::new(built_binary("cerulion"))
            .arg("bootstrap-robot")
            .arg("--state-root")
            .arg(&root)
            .env("CERULION_LOGIN_GATE", "1"),
    );
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let prepared: BootstrapPrepared = serde_json::from_slice(&output.stdout).unwrap();
    let writer = bounded_output(
        Command::new(built_binary("cerulion-remoted"))
            .arg("--state-root")
            .arg(&root)
            .arg("--provision-bundle")
            .arg(&prepared.registration_bundle),
    );
    assert!(
        writer.status.success(),
        "{}",
        String::from_utf8_lossy(&writer.stderr)
    );
    let result: serde_json::Value = serde_json::from_slice(&writer.stdout).unwrap();
    assert_eq!(result["version"], 1);
    assert_eq!(result["robot_id"], prepared.robot_id);
    assert_eq!(result["endpoint_id"], prepared.endpoint_id);
    assert_eq!(result["owner_account"], prepared.owner_account);
    let remoted = root.join("remoted");
    let mac = std::fs::read(remoted.join("trust_store.mac_key")).unwrap();
    let trust = TrustStore::load(remoted.join("trust_store"), &mac).unwrap();
    assert!(trust.is_claimed());
    assert_eq!(
        trust.owner(),
        Some(AccountId(
            URL_SAFE_NO_PAD
                .decode(&state.account_id)
                .unwrap()
                .try_into()
                .unwrap()
        ))
    );
    assert_eq!(
        hex::encode(trust.robot_transport_key().0),
        prepared.endpoint_id
    );

    // Prove the actual server can load the files and bind the registered key.
    // Empty relay configuration has no public discovery or relay side effects.
    let mut server = ProcessGuard(
        Command::new(built_binary("cerulion-remoted"))
            .arg("--state-root")
            .arg(&root)
            .arg("--relay-disabled")
            .env_remove("CERULION_NETWORK")
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(std::fs::File::create(home.join("server.log")).unwrap())
            .spawn()
            .unwrap(),
    );
    let facts = wait_for(
        || {
            assert!(
                server.0.try_wait().unwrap().is_none(),
                "server exited before binding"
            );
            std::fs::read(remoted.join("beacon_facts.json"))
                .ok()
                .and_then(|bytes| serde_json::from_slice::<serde_json::Value>(&bytes).ok())
        },
        Duration::from_secs(10),
    )
    .expect("real server bound and published facts");
    assert_eq!(facts["eid"], prepared.endpoint_id);
    assert_eq!(facts["claimable"], "0");
    assert!(facts["iroh_port"].as_u64().is_some_and(|port| port > 0));
    server.0.kill().unwrap();
    server.0.wait().unwrap();

    // With the issuer unreachable and the login token expired, both actual
    // workers still accept the already committed same-account ownership.
    let before = std::fs::read(remoted.join("trust_store")).unwrap();
    let mut expired = state;
    expired.expires_at_ns = 0;
    auth::write_to(&home.join("auth.json"), &expired).unwrap();
    let output = bounded_output(
        Command::new(built_binary("cerulion"))
            .arg("bootstrap-robot")
            .arg("--state-root")
            .arg(&root)
            .env("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1"),
    );
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let cached: BootstrapPrepared = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(cached, prepared);
    let writer = bounded_output(
        Command::new(built_binary("cerulion-remoted"))
            .arg("--state-root")
            .arg(&root)
            .arg("--provision-bundle")
            .arg(&prepared.registration_bundle),
    );
    assert!(
        writer.status.success(),
        "{}",
        String::from_utf8_lossy(&writer.stderr)
    );
    assert_eq!(std::fs::read(remoted.join("trust_store")).unwrap(), before);
}
