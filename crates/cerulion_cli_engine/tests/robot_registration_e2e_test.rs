// SPDX-License-Identifier: AGPL-3.0-only
//! A2 — the CLI's install-time robot registration end-to-end against the
//! REAL `cerulion_accountd` router on an ephemeral port (Principle #13: no mock
//! service — the actual A2 `POST /v1/robots` issuer serves every request).
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
