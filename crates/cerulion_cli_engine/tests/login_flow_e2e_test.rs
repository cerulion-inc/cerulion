// SPDX-License-Identifier: AGPL-3.0-only
//! The device-code login FLOW end-to-end against the REAL
//! `cerulion_accountd` router on an ephemeral port (Principle #13: no mock
//! service — the actual account-service issuer serves every request).
//!
//! Each test spins the real account-service router on a background tokio runtime,
//! drives the blocking [`login_cmd::run_login`] / [`login_cmd::ensure_login_gate`]
//! from a worker thread, and AUTHORIZES the device code out-of-band via the
//! magic-link seam (the hermetic login path — `magic-link/start` +
//! `magic-link/complete`). Then it asserts the closed loop: `auth.json` written
//! with `logged_in_ever: true`, the device cert cached, and the gate now
//! proceeding.
//!
//! `#[serial]` — the tests mutate the process env (`CERULION_ACCOUNT_SERVICE` /
//! `CERULION_HOME`), which is global.

use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, EmailSender, ServiceConfig, UnconfiguredResolver,
};
use cerulion_cli_engine::account_cmd;
use cerulion_cli_engine::auth::{self, LoadedAuth, LocalGate};
use cerulion_cli_engine::login_cmd;
use serial_test::serial;

// ===========================================================================
// harness
// ===========================================================================

/// A `Write` the worker thread (running `run_login`) writes its TTY prompt to,
/// and the main thread reads to extract the `user_code`.
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

/// RAII env override that restores the previous value on drop.
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

/// Start the REAL account-service router on an ephemeral 127.0.0.1 port using the
/// supplied capturing email sender. Returns the bound port. The server thread is
/// detached (it dies with the test process).
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
            // The magic link the client authorizes with must point back at THIS
            // server, so the completable URL base is the ephemeral address.
            let config = ServiceConfig {
                device_code_interval_secs: 0, // no slow-down (fast test polls)
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

/// Poll `f` until it returns `Some`, up to `timeout`.
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

/// Extract the RFC 8628 `user_code` (`XXXX-XXXX` over the unambiguous consonant
/// alphabet) from the printed prompt.
fn extract_user_code(prompt: &str) -> Option<String> {
    let re = regex::Regex::new(r"[BCDFGHJKLMNPQRSTVWXZ]{4}-[BCDFGHJKLMNPQRSTVWXZ]{4}").unwrap();
    re.find(prompt).map(|m| m.as_str().to_string())
}

/// Authorize the device code that `run_login` created, via the magic-link seam:
/// `magic-link/start` (mints + "emails" a completable link) then GET the captured
/// link (`magic-link/complete`), which authorizes the pending device code.
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
    let body: serde_json::Value = resp.json().unwrap();
    assert_eq!(
        body["status"], "authorized",
        "the device code must be authorized (got {body})"
    );
}

// ===========================================================================
// tests
// ===========================================================================

#[test]
#[serial]
fn device_code_login_writes_auth_json_and_caches_cert() {
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());

    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // Drive `run_login` on a worker thread; it blocks polling for authorization.
    let buf = SharedBuf::new();
    let worker = std::thread::spawn({
        let mut b = buf.clone();
        move || login_cmd::run_login(&mut b)
    });

    // The prompt (with the user_code + "Waiting for authorization...") must print
    // BEFORE any token is issued — i.e. the poll starts pending.
    let code = wait_for(
        || extract_user_code(&buf.snapshot()),
        Duration::from_secs(15),
    )
    .expect("run_login printed a user_code");
    assert!(
        buf.snapshot().contains("Waiting for authorization"),
        "the headless prompt must show the waiting state"
    );

    authorize_via_magic_link(port, &email, &code);

    let state = worker
        .join()
        .expect("worker thread")
        .expect("run_login succeeds after authorization");

    // Closed loop: the returned state + the persisted auth.json agree.
    assert!(state.logged_in_ever, "logged_in_ever set");
    assert!(!state.account_id.is_empty(), "account id resolved");
    assert!(!state.session_token.is_empty());
    assert!(!state.refresh_token.is_empty());

    match auth::load_from(&home.path().join("auth.json")) {
        LoadedAuth::Present(persisted) => {
            assert_eq!(persisted, state, "auth.json == the returned state");
            assert!(persisted.logged_in_ever);
        }
        other => panic!("expected auth.json Present, got {other:?}"),
    }

    // The device cert was cached and the device key created (0600 on Unix).
    assert!(
        home.path().join("device.cert").exists(),
        "the SignedDeviceCert must be cached"
    );
    assert!(
        home.path().join("desk.key").exists(),
        "the device key must be created at login"
    );

    // The gate now proceeds (fires-once semantics: the NEXT run reads this).
    let now = auth::now_unix_ns();
    assert_eq!(
        auth::local_gate(&auth::load_from(&home.path().join("auth.json")), now),
        LocalGate::ProceedValidSession,
    );
}

#[test]
#[serial]
fn ensure_login_gate_at_a_terminal_auto_triggers_device_flow() {
    // The auto-trigger path: never signed in, at a terminal ⇒ `ensure_login_gate`
    // runs the device flow itself and, once authorized, leaves the machine logged
    // in — the enforced-gate, never-logged-in behavior.
    //
    // The terminal answer is passed in rather than sensed. Under libtest stderr
    // and stdin are whatever the harness inherited, so sensing here would make
    // the test pass at a developer's terminal and take the other arm in CI.
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());

    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // Precondition: never logged in ⇒ the gate would refuse.
    assert_eq!(
        auth::local_gate(&auth::load(), auth::now_unix_ns()),
        LocalGate::RefuseNeverLoggedIn,
    );

    let buf = SharedBuf::new();
    let worker = std::thread::spawn({
        let mut b = buf.clone();
        move || login_cmd::ensure_login_gate_with(&mut b, true)
    });

    let code = wait_for(
        || extract_user_code(&buf.snapshot()),
        Duration::from_secs(15),
    )
    .expect("the gate auto-triggered the device flow + printed a user_code");
    // The refusal message names the fix (`cerulion login`).
    assert!(
        buf.snapshot().contains("cerulion login"),
        "the refusal must name the fix"
    );

    authorize_via_magic_link(port, &email, &code);

    worker
        .join()
        .expect("worker thread")
        .expect("ensure_login_gate succeeds after auto-login");

    // The machine is now logged in — a subsequent gate proceeds.
    assert_eq!(
        auth::local_gate(&auth::load(), auth::now_unix_ns()),
        LocalGate::ProceedValidSession,
    );
    assert!(home.path().join("auth.json").exists());
}

#[test]
#[serial]
fn stale_session_refreshes_against_the_real_service() {
    // Opportunistic refresh: log in for real, artificially
    // stale the cached session, then `refresh_session_if_stale` exchanges the
    // refresh token for a FRESH session against the real `/v1/auth/refresh`.
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());

    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // Full device-code login.
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
    authorize_via_magic_link(port, &email, &code);
    let orig = worker.join().unwrap().expect("run_login");

    // Stale ONLY the cached session (the server's session row is still valid, so
    // its refresh token works); keep the refresh token.
    let auth_path = home.path().join("auth.json");
    let stale = auth::AuthState {
        expires_at_ns: 1, // in the distant past ⇒ stale
        ..orig.clone()
    };
    auth::write_to(&auth_path, &stale).unwrap();
    assert_eq!(
        auth::local_gate(&auth::load(), auth::now_unix_ns()),
        LocalGate::ProceedExpiredLocalForever,
        "the cached session is now stale (but local ops still proceed)",
    );

    // Refresh exchanges the refresh token for a fresh session.
    let refreshed = login_cmd::refresh_session_if_stale()
        .expect("refresh ok")
        .expect("a stale session must be refreshed");
    assert_ne!(
        refreshed.session_token, orig.session_token,
        "a NEW session token was issued"
    );
    assert!(
        refreshed.expires_at_ns > auth::now_unix_ns(),
        "the refreshed session is valid"
    );
    assert_eq!(refreshed.account_id, orig.account_id, "same account");
    assert!(refreshed.logged_in_ever);

    // The fresh session is persisted, and the gate now reads a valid session.
    match auth::load_from(&auth_path) {
        LoadedAuth::Present(p) => assert_eq!(p, refreshed, "auth.json == refreshed state"),
        other => panic!("expected Present, got {other:?}"),
    }
    assert_eq!(
        auth::local_gate(&auth::load(), auth::now_unix_ns()),
        LocalGate::ProceedValidSession,
    );
}

#[test]
#[serial]
fn run_login_carries_a_preexisting_robot_role_forward() {
    // Carry-forward pin (run_login site): a re-login must NOT wipe the
    // install-determined `role` marker. Seed a prior logged-in auth.json stamped as a
    // ROBOT (what the install funnel writes), then run a FRESH device-code login — the
    // rewritten auth.json (a NEW account + session) must STILL carry role: robot.
    //
    // Reverting the run_login carry-forward (`role: prior_role` →
    // `role: None` in login_cmd::run_login) makes `state.role` come back None ⇒ the
    // `Some(Robot)` asserts below FAIL. Without this test, that revert passes the suite.
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());

    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // Seed a prior logged-in auth.json stamped ROBOT. A stale session is fine —
    // run_login overwrites the account + session but reads this file for the role.
    let auth_path = home.path().join("auth.json");
    let seed = auth::AuthState {
        account_id: "prior-account".to_string(),
        session_token: "prior-session".to_string(),
        refresh_token: "prior-refresh".to_string(),
        expires_at_ns: 1,
        logged_in_ever: true,
        role: Some(auth::MachineRole::Robot),
    };
    auth::write_to(&auth_path, &seed).unwrap();

    // A fresh device-code login.
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
    authorize_via_magic_link(port, &email, &code);
    let state = worker
        .join()
        .expect("worker thread")
        .expect("run_login succeeds after authorization");

    // The role marker survived the re-login: a NEW session, the SAME role.
    assert_ne!(
        state.session_token, "prior-session",
        "a fresh session token was issued (the login really ran)"
    );
    assert_eq!(
        state.role,
        Some(auth::MachineRole::Robot),
        "run_login carries the pre-existing role forward"
    );
    match auth::load_from(&auth_path) {
        LoadedAuth::Present(persisted) => {
            assert_eq!(
                persisted.role,
                Some(auth::MachineRole::Robot),
                "the rewritten auth.json still carries role: robot"
            );
            assert_eq!(persisted, state, "auth.json == the returned state");
        }
        other => panic!("expected auth.json Present, got {other:?}"),
    }
}

#[test]
#[serial]
fn refresh_session_carries_a_stamped_robot_role_forward() {
    // Carry-forward pin (refresh_session_if_stale site): a token refresh
    // preserves the install-determined `role` (it is orthogonal to the session).
    // Log in for real (to obtain a VALID refresh token the server accepts), stamp the
    // on-disk auth.json ROBOT + stale the session, then refresh — the refreshed state
    // AND the rewritten auth.json must STILL carry role: robot.
    //
    // Reverting the refresh carry-forward (`role: state.role` →
    // `role: None` in refresh_session_if_stale) makes `refreshed.role` come back None ⇒
    // the `Some(Robot)` asserts below FAIL.
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());

    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // Full device-code login → a real, refreshable session (role: None on a fresh machine).
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
    authorize_via_magic_link(port, &email, &code);
    let orig = worker.join().unwrap().expect("run_login");
    assert_eq!(orig.role, None, "a fresh login is role-unmarked (control)");

    // Stamp the machine ROBOT (the install-funnel mark) + stale ONLY the session,
    // KEEPING the valid refresh token so the server accepts the refresh.
    let auth_path = home.path().join("auth.json");
    let stamped = auth::AuthState {
        role: Some(auth::MachineRole::Robot),
        expires_at_ns: 1, // stale
        ..orig.clone()
    };
    auth::write_to(&auth_path, &stamped).unwrap();

    let refreshed = login_cmd::refresh_session_if_stale()
        .expect("refresh ok")
        .expect("a stale session must be refreshed");
    assert_ne!(
        refreshed.session_token, orig.session_token,
        "a NEW session token was issued (the refresh really ran)"
    );
    assert_eq!(
        refreshed.role,
        Some(auth::MachineRole::Robot),
        "refresh_session_if_stale carries the stamped role forward"
    );
    match auth::load_from(&auth_path) {
        LoadedAuth::Present(p) => assert_eq!(
            p.role,
            Some(auth::MachineRole::Robot),
            "the refreshed auth.json still carries role: robot"
        ),
        other => panic!("expected Present, got {other:?}"),
    }
}

#[test]
#[serial]
fn run_login_survives_and_drops_a_foreign_role_value() {
    // Never-bricks (unknown role VALUE): a prior
    // auth.json carrying a role string THIS binary does not recognize (a future
    // "operator" written by a newer binary) must NOT brick the login command. A strict
    // parse would fail on that value → Corrupt → gate refusal. Instead run_login
    // parses the tolerant field (→ None), completes a FRESH login, and writes a clean
    // file with the foreign value DROPPED (not preserved, not a placeholder).
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());

    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // Seed a prior logged-in auth.json with a role value this binary does not know.
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        br#"{"account_id":"prior","session_token":"s","refresh_token":"r",
             "expires_at_ns":1,"logged_in_ever":true,"role":"operator"}"#,
    )
    .unwrap();

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
    authorize_via_magic_link(port, &email, &code);
    let state = worker
        .join()
        .expect("worker thread")
        .expect("run_login survives a foreign role value (never bricks)");

    // The login did NOT brick, and the foreign role was dropped (→ None, key omitted).
    assert_eq!(
        state.role, None,
        "the unrecognized role degraded to None (dropped, not carried)"
    );
    let raw = std::fs::read_to_string(&auth_path).unwrap();
    assert!(
        !raw.contains("operator"),
        "the foreign role value must be gone: {raw}"
    );
    assert!(
        !raw.contains("\"role\""),
        "an unmarked file omits the role key (no placeholder written): {raw}"
    );
}

// ===========================================================================
// an issuer that mints sessions but certifies no devices
// ===========================================================================

/// Serve exactly the identity half of the protocol on an ephemeral loopback
/// port: the device flow authorizes immediately, `/v1/me` names the account,
/// and every `/v1/devices*` path answers 404. Handles one request per
/// connection and stops after `requests` of them.
///
/// This is not a stand-in for `cerulion_accountd` (Principle #13 — the tests
/// above drive the real router). It is the OTHER service shape a desk now
/// meets: the hosted Supabase-backed issuer at
/// [`login_cmd::DEFAULT_ACCOUNT_SERVICE`] has no device-certificate surface at
/// all, which accountd cannot express, and a 404 there must be an ordinary
/// login rather than a failed one.
///
/// Returns the port and the log of request targets it served, so a test can
/// assert the client ASKED for the device surface: everything unrecognized
/// answers 404, so a login that skipped registration entirely (or mistyped its
/// path) would otherwise pass on the same `/v1/me` fallback.
/// `account_id: None` makes `/v1/me` itself fail (500) — the shape that proves a
/// cert survives a login that never resolved an account.
fn start_identity_only_issuer(
    account_id: Option<&'static str>,
    requests: usize,
) -> (u16, Arc<Mutex<Vec<String>>>) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let served: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::clone(&served);
    std::thread::spawn(move || {
        use std::io::{BufRead, BufReader};
        for stream in listener.incoming().take(requests) {
            let mut stream = stream.unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut request_line = String::new();
            reader.read_line(&mut request_line).unwrap();
            // Drain the headers so the client's write completes before we reply.
            let mut length = 0usize;
            loop {
                let mut header = String::new();
                if reader.read_line(&mut header).unwrap() == 0 || header == "\r\n" {
                    break;
                }
                if let Some(v) = header.to_ascii_lowercase().strip_prefix("content-length:") {
                    length = v.trim().parse().unwrap_or(0);
                }
            }
            if length > 0 {
                let mut body = vec![0u8; length];
                std::io::Read::read_exact(&mut reader, &mut body).ok();
            }
            log.lock().unwrap().push(request_line.trim().to_string());
            let (status, body) = if request_line.contains("/v1/auth/device/start") {
                (
                    200,
                    r#"{"device_code":"dc","user_code":"WXYZ-WXYZ",
                        "verification_uri":"http://127.0.0.1/device",
                        "verification_uri_complete":"http://127.0.0.1/device?user_code=WXYZ-WXYZ",
                        "expires_in":600,"interval":0}"#
                        .to_string(),
                )
            } else if request_line.contains("/v1/auth/device/poll") {
                (
                    200,
                    r#"{"session_token":"st","refresh_token":"rt","expires_in":3600}"#.to_string(),
                )
            } else if request_line.contains("/v1/me") {
                match account_id {
                    Some(id) => (200, format!("{{\"account_id\":\"{id}\"}}")),
                    None => (500, r#"{"error":"server_error"}"#.to_string()),
                }
            } else {
                // Every /v1/devices* path: this service certifies nothing.
                (404, r#"{"error":"not_found"}"#.to_string())
            };
            let response = format!(
                "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            std::io::Write::write_all(&mut stream, response.as_bytes()).ok();
        }
    });
    (port, served)
}

/// Serve the identity half AND a working PoP challenge, but answer the
/// registration itself (`POST /v1/devices`) with 500: a service that DOES
/// certify devices and is momentarily broken at the last step. Only a 404 is
/// evidence of an identity-only issuer, so this shape must not be read as one —
/// and the returned log proves the registration endpoint was the one that broke,
/// rather than the login having stopped short of it.
fn start_issuer_with_a_broken_device_surface(requests: usize) -> (u16, Arc<Mutex<Vec<String>>>) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let served: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::clone(&served);
    std::thread::spawn(move || {
        use std::io::{BufRead, BufReader};
        for stream in listener.incoming().take(requests) {
            let mut stream = stream.unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut request_line = String::new();
            reader.read_line(&mut request_line).unwrap();
            let mut length = 0usize;
            loop {
                let mut header = String::new();
                if reader.read_line(&mut header).unwrap() == 0 || header == "\r\n" {
                    break;
                }
                if let Some(v) = header.to_ascii_lowercase().strip_prefix("content-length:") {
                    length = v.trim().parse().unwrap_or(0);
                }
            }
            if length > 0 {
                let mut body = vec![0u8; length];
                std::io::Read::read_exact(&mut reader, &mut body).ok();
            }
            let (status, body) = if request_line.contains("/v1/auth/device/start") {
                (
                    200,
                    r#"{"device_code":"dc","user_code":"WXYZ-WXYZ",
                        "verification_uri":"http://127.0.0.1/device",
                        "verification_uri_complete":"http://127.0.0.1/device?user_code=WXYZ-WXYZ",
                        "expires_in":600,"interval":0}"#
                        .to_string(),
                )
            } else if request_line.contains("/v1/auth/device/poll") {
                (
                    200,
                    r#"{"session_token":"st","refresh_token":"rt","expires_in":3600}"#.to_string(),
                )
            } else if request_line.contains("/v1/me") {
                (200, r#"{"account_id":"account-b"}"#.to_string())
            } else if request_line.contains("/v1/devices/challenge") {
                // A real challenge, so registration is REACHED: 43 base64url
                // 'A's are the 32 zero bytes the client decodes into AccountId.
                (
                    200,
                    format!(
                        r#"{{"challenge":"ch-1","account_id":"{}","expires_in":300}}"#,
                        "A".repeat(43)
                    ),
                )
            } else {
                (500, r#"{"error":"server_error"}"#.to_string())
            };
            log.lock().unwrap().push(request_line.trim().to_string());
            let response = format!(
                "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            std::io::Write::write_all(&mut stream, response.as_bytes()).ok();
        }
    });
    (port, served)
}

/// A 5xx on the device surface is UNKNOWN, not "no device surface": reading it
/// as identity-only would delete a cert this service does issue and commit the
/// switch with no certified key, on nothing worse than a blip.
#[test]
#[serial]
fn a_broken_device_surface_refuses_the_login_rather_than_discarding_the_cert() {
    let (port, served) = start_issuer_with_a_broken_device_surface(8);
    let home = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(&auth_path, b"{not json at all").unwrap();

    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("a device surface that answers 500 is not a service that certifies nothing")
        .to_string();

    // The 500 that refused the login is the REGISTRATION's, not an earlier step's:
    // the challenge succeeded, so `POST /v1/devices` was reached and answered.
    let seen = served.lock().unwrap().clone();
    // Whole request lines, not substrings: every unrecognized path answers the
    // same 500, so `/v1/devices/typo` would satisfy a `contains` and let a
    // mistyped registration URL pass as a reached registration.
    assert!(
        seen.iter()
            .any(|r| r.starts_with("POST /v1/devices/challenge HTTP/1.1")),
        "the challenge must have been fetched: {seen:?}"
    );
    assert!(
        seen.iter()
            .any(|r| r.starts_with("POST /v1/devices HTTP/1.1")),
        "and the registration itself must have been attempted: {seen:?}"
    );
    assert!(
        failed.contains("device registration refused") && failed.contains("500"),
        "the refusal names the registration and its status: {failed}"
    );

    assert_eq!(
        std::fs::read(&own_cert).unwrap(),
        b"cert-issued-to-account-a",
        "the cert of the account still signed in is untouched by a failed login"
    );
    assert_eq!(
        std::fs::read(&auth_path).unwrap(),
        b"{not json at all",
        "and nothing was published: the store is exactly as the login found it"
    );
}

#[test]
#[serial]
fn login_against_an_issuer_with_no_device_surface_signs_in_via_me() {
    let (port, served) = start_identity_only_issuer(Some("supabase-user-uuid"), 8);
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    let state = login_cmd::run_login(&mut buf).expect("a 404 on /v1/devices is not a login error");

    // The account id came from /v1/me, and the session is usable.
    assert_eq!(state.account_id, "supabase-user-uuid");
    assert!(state.logged_in_ever);
    assert_eq!(
        auth::local_gate(
            &auth::load_from(&home.path().join("auth.json")),
            auth::now_unix_ns()
        ),
        LocalGate::ProceedValidSession,
    );

    // No certificate was invented for a service that issues none — and the
    // device key still exists, because the NEXT service might certify it.
    assert!(
        !home.path().join("device.cert").exists(),
        "no device cert may be cached when the service has no /v1/devices"
    );
    assert!(
        home.path().join("desk.key").exists(),
        "the device key is still created"
    );

    // The 404 fallback is only meaningful if registration was actually tried:
    // the challenge fetch is where the client learns there is no device surface.
    let served = served.lock().unwrap().clone();
    assert!(
        served
            .iter()
            .any(|r| r.starts_with("POST /v1/devices/challenge HTTP/1.1")),
        "login must attempt device registration before falling back to /v1/me: {served:?}"
    );
    assert!(
        served.iter().any(|r| r.starts_with("GET /v1/me HTTP/1.1")),
        "the account id must come from /v1/me: {served:?}"
    );
}

#[test]
#[serial]
fn identity_only_login_drops_a_previous_account_device_cert() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // A cert cached by an EARLIER login, against a service that did certify
    // devices. Its content is irrelevant here — what matters is the file being
    // gone afterwards, because `resolve_device_binding` reads whatever is there
    // and reports THAT cert's account as this machine's binding.
    let cert_path = home.path().join("device.cert");
    std::fs::write(&cert_path, "cert-issued-to-account-a").unwrap();

    let mut buf = SharedBuf::new();
    let state = login_cmd::run_login(&mut buf).expect("the identity-only login succeeds");

    assert_eq!(state.account_id, "account-b");
    assert!(
        !cert_path.exists(),
        "a login that was issued no cert must not leave the previous account's cert \
         authoritative for device binding"
    );
}

/// A cert is stale only once a NEW account has been resolved. `/v1/me` failing
/// means no login happened at all, so the previous session — and the binding its
/// cert answers with — must survive intact.
#[test]
#[serial]
fn a_login_that_never_resolves_an_account_keeps_the_cached_cert() {
    let (port, _served) = start_identity_only_issuer(None, 8);
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let cert_path = home.path().join("device.cert");
    std::fs::write(&cert_path, "cert-issued-to-account-a").unwrap();

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect_err("a 500 on /v1/me is a failed login");

    assert!(
        cert_path.exists(),
        "a login that resolved no account must leave the working binding alone"
    );
}

/// `cerulion-netd` resolves the cert from ITS environment, so an override that
/// moves the file out of `~/.cerulion` moves the account confusion with it: the
/// WAN registry would keep presenting account A after signing in as B.
#[test]
#[serial]
fn identity_only_login_drops_the_cert_netd_was_pointed_at() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    let state = login_cmd::run_login(&mut buf).expect("the identity-only login succeeds");

    assert_eq!(state.account_id, "account-b");
    assert!(
        !netd_cert.exists(),
        "the cert netd actually reads must be cleared too, not just the CLI's own path"
    );
}

#[test]
#[serial]
fn account_device_verbs_refuse_an_issuer_that_registers_nothing() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 4);
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );

    // Both verbs meet the same 404 an unknown device id would give, so each must
    // say the SERVICE has no device surface rather than blame the request.
    let listed = account_cmd::list_my_devices("st").expect_err("no collection to list");
    let revoked = account_cmd::revoke_my_device("st", "dev-1").expect_err("nothing to revoke");
    for e in [&listed, &revoked] {
        let msg = e.to_string();
        assert!(
            msg.contains("registers no devices") && msg.contains(login_cmd::ACCOUNT_SERVICE_ENV),
            "the refusal must name the capability and the variable that changes it: {msg}"
        );
    }
}

/// The capability probe answers "does this service register devices at all?".
/// When the probe itself cannot answer, neither reading is available — reporting
/// the ordinary missing-device outcome would be a guess dressed as a finding.
#[test]
#[serial]
fn a_probe_that_cannot_answer_is_reported_rather_than_swallowed() {
    // One request only: the revoke POST is answered 404, then the listener is
    // gone, so the subsequent GET fails at the transport.
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 1);
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );

    let err = account_cmd::revoke_my_device("st", "dev-1").expect_err("the probe cannot answer");
    let msg = err.to_string();
    assert!(
        !msg.contains("registers no devices"),
        "an unanswered probe must not be read as a capability verdict: {msg}"
    );
    assert!(
        msg.contains("unknown"),
        "the error must say which of the two outcomes is undetermined: {msg}"
    );
}

#[test]
#[serial]
fn revoking_an_unknown_device_on_a_certifying_service_is_not_a_capability_error() {
    // A service that DOES register devices: the collection answers, only the
    // revoke route 404s (that id is not this account's). The capability refusal
    // must not swallow it.
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        use std::io::{BufRead, BufReader};
        for stream in listener.incoming().take(2) {
            let mut stream = stream.unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut request_line = String::new();
            reader.read_line(&mut request_line).unwrap();
            loop {
                let mut header = String::new();
                if reader.read_line(&mut header).unwrap() == 0 || header == "\r\n" {
                    break;
                }
            }
            let (status, body) = if request_line.starts_with("GET /v1/devices ") {
                (200, r#"{"devices":[]}"#.to_string())
            } else {
                (404, r#"{"error":"device_not_found"}"#.to_string())
            };
            let response = format!(
                "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            std::io::Write::write_all(&mut stream, response.as_bytes()).ok();
        }
    });
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );

    let msg = account_cmd::revoke_my_device("st", "not-mine")
        .expect_err("a device the caller does not own is refused")
        .to_string();
    assert!(
        !msg.contains("registers no devices"),
        "a certifying service's 404 is about the id, not the surface: {msg}"
    );
}

/// A probe that *answers* with a failure status answers nothing: an expired
/// session (401) or a broken service (500) leaves both readings open, exactly as
/// a timeout does. Reporting the revoke's 404 then would call a device missing on
/// the word of a request that never ran.
#[test]
#[serial]
fn a_probe_that_answers_a_failure_status_is_not_a_capability_verdict() {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    std::thread::spawn(move || {
        use std::io::{BufRead, BufReader};
        for stream in listener.incoming().take(2) {
            let mut stream = stream.unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut request_line = String::new();
            reader.read_line(&mut request_line).unwrap();
            loop {
                let mut header = String::new();
                if reader.read_line(&mut header).unwrap() == 0 || header == "\r\n" {
                    break;
                }
            }
            let (status, body) = if request_line.starts_with("GET /v1/devices ") {
                (401, r#"{"error":"invalid_token"}"#.to_string())
            } else {
                (404, r#"{"error":"device_not_found"}"#.to_string())
            };
            let response = format!(
                "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            std::io::Write::write_all(&mut stream, response.as_bytes()).ok();
        }
    });
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );

    let msg = account_cmd::revoke_my_device("st", "dev-1")
        .expect_err("the probe answered nothing usable")
        .to_string();
    assert!(
        msg.contains("unknown") && !msg.contains("registers no devices"),
        "an unanswerable probe must report the outcome as undetermined: {msg}"
    );
}

/// One unclearable cert path must not strand the others. Aborting at the first
/// failure would leave the desk with some consumers' caches cleared and others
/// still naming the previous account, and the message would name a file that is
/// not the one to delete.
///
/// The clear is a RENAME to a hidden sibling, so the way to make one refuse is to
/// occupy that sibling with something a rename cannot replace: a DIRECTORY. (This
/// is not a contrivance — it is exactly what a crashed login's aside would look
/// like if the path it names were a directory.)
#[test]
#[serial]
fn a_cert_path_that_refuses_removal_names_itself_and_stops_the_login() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    std::fs::create_dir(elsewhere.path().join(".desk.cert.superseded.account-a")).unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let before = std::fs::read(&auth_path).unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    let msg = login_cmd::run_login(&mut buf)
        .expect_err("a cert that outlives the switch fails the login")
        .to_string();

    assert!(
        msg.contains(netd_cert.to_str().unwrap()),
        "the error must name the path that is still there, not a fixed one: {msg}"
    );
    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-issued-to-account-a",
        "a clear that cannot finish is put back whole: a desk left with the netd copy \
         and not its own reads two different answers about which account it is"
    );
    let after: serde_json::Value =
        serde_json::from_slice(&std::fs::read(&auth_path).unwrap()).unwrap();
    let before: serde_json::Value = serde_json::from_slice(&before).unwrap();
    assert_eq!(
        after, before,
        "a login that cannot finish must leave the previous session it rolled back to, \
         not account-b's tokens under account-a's surviving cert"
    );
}

/// Every path is attempted even when the FIRST one refuses. Resolution puts
/// `~/.cerulion/device.cert` before netd's override, so a clear that returned at
/// its first error would satisfy the assertions of the test above for free: this
/// one makes BOTH paths refuse and proves the second was still attempted, by the
/// only witness that survives an all-or-nothing rollback — the refusal has to name
/// them both.
#[cfg(unix)]
#[test]
#[serial]
fn a_first_path_that_refuses_does_not_strand_the_paths_after_it() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    // Both resolved paths hold a cert, and both have their hidden sibling occupied
    // by a directory, which a rename cannot replace.
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    std::fs::create_dir(home.path().join(".device.cert.superseded.account-a")).unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    std::fs::create_dir(elsewhere.path().join(".desk.cert.superseded.account-a")).unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();

    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("a cert that outlives the switch fails the login")
        .to_string();

    assert!(
        failed.contains(own_cert.to_str().unwrap()) && failed.contains(netd_cert.to_str().unwrap()),
        "the refusal names BOTH paths that are still there: a clear that returned at \
         the first one would name only `device.cert`, and the operator would delete \
         one file and hit the same refusal again: {failed}"
    );
    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-issued-to-account-a",
        "and the path it reached second still reads the previous account's cert"
    );
}

/// A cert path that is a SYMLINK is restored as a symlink. Writing the bytes it
/// resolved to would freeze that account's cert at the link path and take it out
/// of whatever rotates the target (netd re-writing the file it points into),
/// leaving a stale cert authoritative for device binding.
#[cfg(unix)]
#[test]
#[serial]
fn a_rollback_puts_a_cert_symlink_back_as_a_symlink() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let target = elsewhere.path().join("rotated-by-netd.cert");
    std::fs::write(&target, "cert-issued-to-account-a").unwrap();
    let link = home.path().join("device.cert");
    std::os::unix::fs::symlink(&target, &link).unwrap();
    // A later path that refuses, so the clear has to be undone: its hidden sibling
    // is occupied by a directory, which a rename cannot replace.
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    std::fs::create_dir(elsewhere.path().join(".desk.cert.superseded.unknown")).unwrap();

    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect_err("an unclearable cert fails the login");

    let md = std::fs::symlink_metadata(&link).expect("the link is back");
    assert!(
        md.file_type().is_symlink(),
        "restored as a link, not as a regular file holding the bytes it pointed at"
    );
    assert_eq!(
        std::fs::read_link(&link).unwrap(),
        target,
        "and at the same target, so the path still follows whatever rotates it"
    );
    // The rotation the link exists for still reaches the link's readers.
    std::fs::write(&target, "cert-rotated-since").unwrap();
    assert_eq!(
        std::fs::read_to_string(&link).unwrap(),
        "cert-rotated-since",
        "a regular file frozen at the old bytes would answer with the stale cert"
    );
}

/// A corrupt `auth.json` is a recovery artifact, never deleted ("corrupt ⇒
/// re-login"). The rollback restores its BYTES, so a refused login cannot turn
/// unparseable-but-present into absent.
#[test]
#[serial]
fn a_rollback_puts_a_corrupt_store_back_rather_than_deleting_it() {
    let (port, served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    // Its hidden sibling is occupied by a directory, so the clear refuses. A
    // corrupt store names no account, so the aside tag is `unknown`.
    std::fs::create_dir(elsewhere.path().join(".desk.cert.superseded.unknown")).unwrap();
    std::fs::write(home.path().join("device.cert"), "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(&auth_path, b"{not json at all").unwrap();

    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("an unclearable cert fails the login")
        .to_string();

    // Prove the ROLLBACK is what left those bytes behind. A regression that
    // refused the corrupt store up front — before the device flow, before
    // `/v1/me`, before the cert clear — would leave the same bytes for free and
    // pass an equality assertion on its own.
    let asked = served.lock().unwrap().join("\n");
    assert!(
        asked.lines().any(|r| r.starts_with("GET /v1/me HTTP/1.1")),
        "the login must get as far as resolving the account, so the write it rolls \
         back is a write it actually attempted: {asked}"
    );
    assert!(
        failed.contains(netd_cert.to_str().unwrap()),
        "and it must fail on the cert that refused removal, naming it: {failed}"
    );
    assert_eq!(
        std::fs::read(&auth_path).unwrap(),
        b"{not json at all",
        "the corrupt file the operator may still need is restored byte-for-byte, \
         not deleted because it parsed to no state"
    );
}

/// A cert this process cannot READ is still cleared: the clear renames it, and a
/// rename needs no access to the file's contents. This is the concrete reason the
/// clear does not snapshot bytes — a cert whose mode, ACL or type denies a read
/// would be unclearable, so a mode-000 file left behind by an earlier install
/// would make every account switch on the machine fail with no way out but `rm`.
///
/// Unix-only: the unreadable file is made with a mode.
#[cfg(unix)]
#[test]
#[serial]
fn a_cert_that_cannot_be_read_is_cleared_anyway_because_the_clear_is_a_rename() {
    use std::os::unix::fs::PermissionsExt;

    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    // Mode 000: present, renameable (the DIRECTORY is writable), unreadable.
    std::fs::set_permissions(&own_cert, std::fs::Permissions::from_mode(0o000)).unwrap();

    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    let state =
        login_cmd::run_login(&mut buf).expect("an unreadable cert does not block the login");
    assert_eq!(state.account_id, "account-b");

    assert!(
        std::fs::symlink_metadata(&own_cert).is_err(),
        "the previous account's cert is gone from the path device binding reads"
    );
    let left: Vec<String> = std::fs::read_dir(home.path())
        .unwrap()
        .flatten()
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.contains("superseded"))
        .collect();
    assert!(
        left.is_empty(),
        "and the aside it was moved to is dropped once the store is published: it \
         holds a secret for an account this machine is no longer signed in to: {left:?}"
    );
}

/// The same unreadable cert, through a login that REFUSES: the entry the clear
/// could not read is back at its own path afterwards, mode and all. That is the
/// property the rename-aside exists for, observed from outside the login — a
/// clear that unlinked what it cannot read would have nothing to put back here.
///
/// Unix-only: file modes.
#[cfg(unix)]
#[test]
#[serial]
fn a_login_that_refuses_puts_back_the_cert_it_could_not_read() {
    use std::os::unix::fs::MetadataExt;
    use std::os::unix::fs::PermissionsExt;

    let (port, served) = start_certifying_issuer("account-b", "cert-issued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    let netd_cert = elsewhere.path().join("desk.cert");

    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    // The identity of the entry itself, taken while it is still readable: a
    // rollback that deleted the cert and recreated an empty mode-000 file in
    // its place would satisfy every other assertion here.
    let before = std::fs::symlink_metadata(&own_cert).unwrap();
    std::fs::set_permissions(&own_cert, std::fs::Permissions::from_mode(0o000)).unwrap();
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    // A directory occupies the staging name for netd's target, so the login
    // fails AFTER both stale certs were moved aside and before anything is
    // published: exactly the window a killed login leaves behind.
    std::fs::create_dir(
        elsewhere
            .path()
            .join(format!(".desk.cert.{}.tmp", std::process::id())),
    )
    .unwrap();

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("a target that cannot take the cert stops the login")
        .to_string();

    // Without these two, a login that failed BEFORE the clear — an issuer that
    // never certified, a validation that refused first — leaves the mode-000
    // cert untouched and no aside, and every assertion below passes without a
    // rename having happened at all.
    let requests = served.lock().unwrap().clone();
    assert!(
        requests
            .iter()
            .any(|r| r.starts_with("POST /v1/devices HTTP/1.1")),
        "the login got as far as being ISSUED a cert, so there was a clear to undo: {requests:?}"
    );
    assert!(
        failed.contains(&netd_cert.display().to_string()),
        "and it is the occupied staging target that stopped it: {failed}"
    );

    let restored = std::fs::symlink_metadata(&own_cert)
        .expect("the entry the clear could not read is back at its own path");
    assert_eq!(
        restored.permissions().mode() & 0o777,
        0o000,
        "unchanged, because putting it back is a rename and not a copy"
    );
    assert_eq!(
        (restored.ino(), restored.len()),
        (before.ino(), before.len()),
        "and it is the SAME entry, not an empty file wearing its mode: a rollback \
         that recreated the path would lose the binding it claims to have kept"
    );
    std::fs::set_permissions(&own_cert, std::fs::Permissions::from_mode(0o600)).unwrap();
    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-issued-to-account-a",
        "contents and all"
    );
    let left: Vec<String> = std::fs::read_dir(home.path())
        .unwrap()
        .flatten()
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.contains("superseded"))
        .collect();
    assert!(
        left.is_empty(),
        "and the aside it was held at is consumed, not left as a second copy: {left:?}"
    );
}

/// A symlink pointing at nothing is a cache ENTRY, not an absence: `read` says
/// NotFound, but the moment its target reappears the link answers with the
/// previous account's cert again. The clear unlinks it.
///
/// Unix-only: symlinks.
#[cfg(unix)]
#[test]
#[serial]
fn a_dangling_cert_symlink_is_unlinked_rather_than_read_as_absent() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let target = elsewhere.path().join("comes-back-later.cert");
    let link = home.path().join("device.cert");
    std::os::unix::fs::symlink(&target, &link).unwrap();

    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("the switch itself succeeds");

    assert!(
        std::fs::symlink_metadata(&link).is_err(),
        "the link is gone, not merely pointing at nothing: writing its target would \
         otherwise resurrect the previous account's binding after the switch"
    );
    // The proof that mattered: a target appearing now reaches no cache entry.
    std::fs::write(&target, "cert-issued-to-account-a").unwrap();
    assert!(
        std::fs::read(&link).is_err(),
        "and no consumer can follow the old path to it"
    );
}

/// The store write and the cert discard are one commit: a login that cannot
/// publish its state must leave the previous account's cert where it found it,
/// or the desk loses a binding to a login that never happened.
#[test]
#[serial]
fn a_login_that_cannot_publish_its_state_keeps_the_previous_cert() {
    let (port, _served) = start_identity_only_issuer(Some("account-b"), 8);
    let home = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    // A DIRECTORY where auth.json belongs: the atomic rename onto it refuses,
    // which is the shape of any store this process cannot publish to.
    std::fs::create_dir(home.path().join("auth.json")).unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect_err("an unpublishable store fails the login");

    assert!(
        own_cert.exists(),
        "the cert may only be discarded once the state that makes it stale is committed"
    );
}

// ===========================================================================
// an issuer that DOES certify device keys, without accountd's storage
// ===========================================================================

/// Serve the identity half plus a working `/v1/devices/challenge` AND a
/// successful `POST /v1/devices` that hands back `cert` for `account_id`.
///
/// The real router (`cerulion_accountd`, driven by the tests at the top of this
/// file) is the proof that certification WORKS; this shape exists to drive the
/// certifying login's COMMIT ORDER, which needs the account id and the cert
/// bytes to be known constants and the store to be arranged around them.
fn start_certifying_issuer(
    account_id: &'static str,
    cert: &'static str,
    requests: usize,
) -> (u16, Arc<Mutex<Vec<String>>>) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let served: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let log = Arc::clone(&served);
    std::thread::spawn(move || {
        use std::io::{BufRead, BufReader};
        for stream in listener.incoming().take(requests) {
            let mut stream = stream.unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            let mut request_line = String::new();
            reader.read_line(&mut request_line).unwrap();
            let mut length = 0usize;
            loop {
                let mut header = String::new();
                if reader.read_line(&mut header).unwrap() == 0 || header == "\r\n" {
                    break;
                }
                if let Some(v) = header.to_ascii_lowercase().strip_prefix("content-length:") {
                    length = v.trim().parse().unwrap_or(0);
                }
            }
            if length > 0 {
                let mut body = vec![0u8; length];
                std::io::Read::read_exact(&mut reader, &mut body).ok();
            }
            log.lock().unwrap().push(request_line.trim().to_string());
            let (status, body) = if request_line.contains("/v1/auth/device/start") {
                (
                    200,
                    r#"{"device_code":"dc","user_code":"WXYZ-WXYZ",
                        "verification_uri":"http://127.0.0.1/device",
                        "verification_uri_complete":"http://127.0.0.1/device?user_code=WXYZ-WXYZ",
                        "expires_in":600,"interval":0}"#
                        .to_string(),
                )
            } else if request_line.contains("/v1/auth/device/poll") {
                (
                    200,
                    r#"{"session_token":"st","refresh_token":"rt","expires_in":3600}"#.to_string(),
                )
            } else if request_line.starts_with("POST /v1/devices/challenge ") {
                // 43 base64url 'A's = the 32 zero bytes the client decodes.
                (
                    200,
                    format!(
                        r#"{{"challenge":"ch-1","account_id":"{}","expires_in":300}}"#,
                        "A".repeat(43)
                    ),
                )
            } else if request_line.starts_with("POST /v1/devices ") {
                (
                    200,
                    format!(
                        r#"{{"device_id":"dev-1","device_cert":"{cert}","intermediate":"int-1",
                            "account_id":"{account_id}"}}"#
                    ),
                )
            } else {
                // Including `/v1/me`: a certifying login must never need it, and
                // an answer here would let a regression that skipped
                // registration resolve the same account and pass.
                (500, r#"{"error":"server_error"}"#.to_string())
            };
            let response = format!(
                "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\n\
                 Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                body.len()
            );
            std::io::Write::write_all(&mut stream, response.as_bytes()).ok();
        }
    });
    (port, served)
}

/// The cert and the `auth.json` are ONE binding, so a certifying login publishes
/// them in ONE critical section and in an order no interruption can split into a
/// cross-account pair: the previous account's cert goes first, the store second,
/// the new cert last. Caching the cert as it arrives — before the store names
/// the account it was issued to — leaves every window between the two naming
/// DIFFERENT accounts, and `resolve_device_binding` reads the cert as truth.
///
/// The store is unpublishable here, which is the failure that window is entered
/// through: the login must end with the previous account's cert intact and the
/// new one nowhere on disk. The ORDER that guarantees it is pinned separately, by
/// `the_certifying_commit_publishes_the_store_before_it_caches_the_cert` — the
/// bytes here cannot distinguish it, since a cache-first commit that rolls back
/// leaves exactly this state too.
#[test]
#[serial]
fn a_certifying_login_that_cannot_publish_its_state_caches_no_cert() {
    let (port, served) = start_certifying_issuer("account-b", "cert-issued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    // A DIRECTORY where auth.json belongs: the atomic rename onto it refuses.
    std::fs::create_dir(home.path().join("auth.json")).unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("an unpublishable store fails the login")
        .to_string();

    // The cert was ISSUED — the failure is the store, not the registration, so
    // the ordering under test was actually reached.
    let asked = served.lock().unwrap().join("\n");
    assert!(
        asked
            .lines()
            .any(|r| r.starts_with("POST /v1/devices HTTP/1.1")),
        "the login must get its certificate before the store write it fails on: {asked}"
    );
    assert!(
        failed.contains("auth.json"),
        "and fail on publishing the store: {failed}"
    );
    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-issued-to-account-a",
        "the previous account's cert is put back whole: a desk left holding \
         account-b's certificate under account-a's session is bound to an account \
         it is not signed in to"
    );
}

/// The bytes above cannot tell the two orders apart — a login that cached the new
/// cert FIRST and put the old one back on failure leaves exactly the same file,
/// and the rollback rewrites it either way, so even the inode changes in the
/// correct order. What separates them is the order of the two publications
/// themselves, and there is no seam to observe it through from outside the
/// process: the whole point is that both happen under one hold of the store lock,
/// with nothing in between for a concurrent reader to catch.
///
/// So it is pinned where it is decided. A commit that publishes the cert before
/// the store is a source-order change, and this fails on it — as is one that
/// stages the cert targets AFTER the store, which is what makes the multi-path
/// cache all-or-nothing (a target that cannot take the cert must refuse the login
/// while nothing is published, not warn after the session is durable).
#[test]
fn the_certifying_commit_stages_the_certs_then_publishes_the_store_then_commits_them() {
    let source =
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/login_cmd.rs")).unwrap();
    let commit = &source[source.find("pub fn run_login").expect("run_login")..];
    let stage = commit
        .find("auth::stage_device_cert_at(path, cert)")
        .expect("run_login stages the issued cert at every target");
    let store = commit
        .find("auth::write_to(&auth_path, &state)")
        .expect("run_login publishes auth.json");
    let publish = commit
        .find("pending.commit()")
        .expect("run_login publishes the staged certs");
    assert!(
        stage < store,
        "every cert target is staged BEFORE the store is published: a target that \
         cannot take the cert has to refuse a login that has published nothing, \
         not leave a session naming an account no consumer is bound to"
    );
    assert!(
        store < publish,
        "and the staged certs are published AFTER it: publishing first leaves every \
         window between the two naming different accounts, and \
         `resolve_device_binding` reads the cert as truth"
    );
}

/// A cert target that cannot be written refuses the whole login: the store is
/// published only once every consumer's cache CAN take the cert, so no window
/// exists in which `auth.json` names account B while netd's path holds nothing.
/// The previous account's cert is put back, and the message names the path.
#[test]
#[serial]
fn a_certifying_switch_with_one_unusable_cert_target_publishes_nothing() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-issued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    // A DIRECTORY sits where this process stages the CLI's OWN cache, so that
    // target cannot take the cert — while netd's path, cleared by the switch,
    // stays perfectly writable. One unusable target is enough: publishing a
    // session whose cert only SOME consumers hold is the state this refuses.
    std::fs::create_dir(
        home.path()
            .join(format!(".device.cert.{}.tmp", std::process::id())),
    )
    .unwrap();

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("a cert target that refuses the cert refuses the login")
        .to_string();

    assert!(
        failed.contains(&home.path().join("device.cert").display().to_string()),
        "the refusal names the path that could not take the cert: {failed}"
    );
    let published = std::fs::read_to_string(&auth_path).unwrap();
    assert!(
        published.contains("account-a") && !published.contains("account-b"),
        "the store still names the previous account: publishing account-b here \
         would leave netd bound to nothing at all — {published}"
    );
    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-issued-to-account-a",
        "and the previous account's cert is put back where netd reads it"
    );
    assert!(
        !home.path().join("device.cert").exists(),
        "with no cert cached for the account that was never published"
    );
}

/// A relative `CERULION_NETD_*` path is refused by name rather than resolved
/// against the CLI's working directory: `cerulion-netd` resolves it against ITS
/// own, so writing the CLI's file would report a device binding netd cannot see.
#[test]
#[serial]
fn a_relative_netd_cert_path_refuses_the_login_by_name() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-issued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", "certs/desk.cert");

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("an unresolvable netd cert path refuses the login")
        .to_string();

    assert!(
        failed.contains("CERULION_NETD_DEVICE_CERT") && failed.contains("absolute"),
        "the refusal names the variable and what it needs: {failed}"
    );
    let published = std::fs::read_to_string(home.path().join("auth.json")).unwrap();
    assert!(
        published.contains("account-a"),
        "and nothing is published: {published}"
    );
}

/// netd resolves the cert from ITS environment, so the clear covers a relocated
/// cache — and a login that was issued a cert of its own therefore has to CACHE
/// there too. Writing only `~/.cerulion/device.cert` would leave netd reading
/// nothing: a desk with no WAN device binding after a login that said it worked,
/// which is worse than the stale-cert state the clear exists to prevent.
#[test]
#[serial]
fn a_certifying_switch_caches_the_new_cert_where_netd_reads_it() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-issued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("the certifying switch succeeds");

    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-issued-to-account-b",
        "netd's configured path holds the cert this login was issued, not the \
         emptiness the clear left there"
    );
    assert_eq!(
        std::fs::read_to_string(home.path().join("device.cert")).unwrap(),
        "cert-issued-to-account-b",
        "as does the CLI's own path: every consumer reads ONE account"
    );
}

/// The successful order, from the other side: once the store names account-b,
/// the cert issued for account-b replaces account-a's — a login that left the
/// old cert in place would keep resolving the old account.
#[test]
#[serial]
fn a_certifying_login_replaces_the_previous_accounts_cert() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-issued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-a","session_token":"a-session",
            "refresh_token":"a-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    let state = login_cmd::run_login(&mut buf).expect("the certifying login succeeds");

    assert_eq!(state.account_id, "account-b");
    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-issued-to-account-b",
        "the cached cert names the account the store now names"
    );
    match auth::load_from(&auth_path) {
        LoadedAuth::Present(persisted) => assert_eq!(persisted, state),
        other => panic!("expected auth.json Present, got {other:?}"),
    }
}

/// Re-signing in to the SAME account clears nothing: there is no cross-account
/// pair to prevent, and the clear reaches every path netd's env overrides move
/// the cache to — paths this login does NOT rewrite, since it caches to
/// `~/.cerulion/device.cert` and netd resolves its own. Clearing them on a
/// same-account re-login would delete a live cache for nothing.
#[test]
#[serial]
fn a_same_account_relogin_keeps_the_cache_netd_was_pointed_at() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(&netd_cert, "cert-issued-to-account-b").unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("a same-account re-login succeeds");

    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-issued-to-account-b",
        "netd's relocated cache is left alone: it names the account the store \
         still names, and nothing rewrites it after a clear"
    );
    assert_eq!(
        std::fs::read_to_string(home.path().join("device.cert")).unwrap(),
        "cert-reissued-to-account-b",
        "while the CLI's own cache takes the freshly issued cert"
    );
}

/// `~/.cerulion/device.cert` is the CLI's OWN file, and a symlink there is
/// REPLACED rather than written through: following it would write a secret
/// wherever the link points, which is the symlink-attack shape every other
/// secret write in this module is built to refuse. The relocated caches netd
/// rotates are the ones left intact, and they are left intact by not being
/// TARGETS (the same-account test above), not by writing through links.
#[cfg(unix)]
#[test]
#[serial]
fn the_cli_owned_cert_path_replaces_a_symlink_instead_of_writing_through_it() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let pointed_at = elsewhere.path().join("linked.cert");
    std::fs::write(&pointed_at, "someone-elses-file").unwrap();
    let own_cert = home.path().join("device.cert");
    std::os::unix::fs::symlink(&pointed_at, &own_cert).unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("the re-login succeeds");

    assert!(
        !std::fs::symlink_metadata(&own_cert).unwrap().is_symlink(),
        "the CLI's own path holds the cert as a regular file it owns"
    );
    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-reissued-to-account-b"
    );
    assert_eq!(
        std::fs::read_to_string(&pointed_at).unwrap(),
        "someone-elses-file",
        "and the file the link pointed at is untouched: a write-through would \
         have put a device cert wherever the link said"
    );
}

/// The renames that publish the staged certs are one per consumer, so a failure
/// part-way through must not report success: the login FAILS naming the path.
/// Here the FIRST (and only) target refuses, so no consumer holds the cert and
/// the message says so. The session itself is published and stays — it names the
/// account, and no cert on disk names another.
#[test]
#[serial]
fn a_cert_target_that_refuses_the_rename_fails_the_login_with_no_cache_left() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    // A DIRECTORY at the CLI's cache path, on a SAME-account re-login so no clear
    // inspects it: staging its sibling temp succeeds, so the login gets past the
    // all-or-nothing staging gate, and only the RENAME that publishes it fails —
    // the window this test exists for.
    std::fs::create_dir(home.path().join("device.cert")).unwrap();

    let mut buf = SharedBuf::new();
    let failed = login_cmd::run_login(&mut buf)
        .expect_err("a cert that cannot be published is not a successful login")
        .to_string();

    assert!(
        failed.contains(&home.path().join("device.cert").display().to_string())
            && failed.contains("no device binding"),
        "the failure names the path and what it costs: {failed}"
    );
    assert!(
        failed.contains("no consumer on this desk has one"),
        "and says none of them took the cert: {failed}"
    );
    let published = std::fs::read_to_string(&auth_path).unwrap();
    assert!(
        published.contains("account-b") && !published.contains("b-session"),
        "the session IS published — it is durable and correct, and re-running the \
         login is what re-certifies the desk: {published}"
    );
    let leftovers: Vec<String> = std::fs::read_dir(home.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|n| n.contains(".tmp"))
        .collect();
    assert!(
        leftovers.is_empty(),
        "with no staging files left behind: {leftovers:?}"
    );
}

/// The crash window the clear cannot close: it is a separate durable step from
/// the `auth.json` publication, so a kill between them is possible. Here the
/// PREVIOUS login died in exactly that window — its cert is sitting at the hidden
/// aside, tagged with the account the store still names — and the next login puts
/// it back, because that session is still the one on disk and its device binding
/// belongs to it.
///
/// Without the put-back this desk stays signed in with no cached cert: netd
/// resolves no device binding, and nothing tells the user why, because the login
/// that broke it reported nothing.
#[cfg(unix)]
#[test]
#[serial]
fn an_interrupted_clear_is_put_back_when_the_store_still_names_its_account() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    // What a login killed between its clear and its publication leaves: the cert
    // at the aside, nothing at the path netd reads, and a store still naming the
    // account the aside is tagged with.
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(
        elsewhere.path().join(".desk.cert.superseded.account-b"),
        "cert-issued-to-account-b",
    )
    .unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("a same-account re-login succeeds");

    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-issued-to-account-b",
        "the interrupted login's cert is back at the path netd reads, and a \
         same-account re-login does not touch it afterwards"
    );
    assert!(
        !elsewhere
            .path()
            .join(".desk.cert.superseded.account-b")
            .exists(),
        "and the aside is consumed, not left as a second copy of a live secret"
    );
}

/// The other half of the same recovery, and the reason the aside carries the
/// owner at all: an aside tagged with an account the store no longer names is
/// from a login that DID publish (or from an even older one), so putting it back
/// would bind this desk to an account it is not signed in to — which is precisely
/// the misbinding the clear exists to prevent. It is dropped instead, leaving the
/// path uncached, which is what a fresh machine is.
#[cfg(unix)]
#[test]
#[serial]
fn an_interrupted_clear_is_dropped_when_the_store_has_moved_on() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    let stale_aside = elsewhere.path().join(".desk.cert.superseded.account-a");
    std::fs::write(&stale_aside, "cert-issued-to-account-a").unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("the re-login succeeds");

    assert!(
        !stale_aside.exists(),
        "another account's cert is not left lying beside the path netd reads"
    );
    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-reissued-to-account-b",
        "and it is NOT put back: account-a's cert at the path device binding reads \
         would name an account this store does not. What the path holds is THIS \
         login's cert — an empty consumer is one this login caches to"
    );
}

/// A cert target that is a symlink ALREADY resolving to this same cert is left
/// exactly as it is. It arises on a same-account re-login, where the write would
/// change nothing but the entry's KIND — replacing the link with a regular file,
/// which takes the path out of whatever rotates the link's target. The
/// write-through refusal is unaffected: a link resolving to anything else is
/// replaced (the test above), and it is replaced, never followed.
#[cfg(unix)]
#[test]
#[serial]
fn a_cert_symlink_that_already_holds_this_cert_is_left_as_a_symlink() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let rotated = elsewhere.path().join("rotated-by-netd.cert");
    std::fs::write(&rotated, "cert-reissued-to-account-b").unwrap();
    let own_cert = home.path().join("device.cert");
    std::os::unix::fs::symlink(&rotated, &own_cert).unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("the re-login succeeds");

    assert!(
        std::fs::symlink_metadata(&own_cert)
            .unwrap()
            .file_type()
            .is_symlink(),
        "the link is still a link, so the path still follows the rotation"
    );
    std::fs::write(&rotated, "cert-rotated-since").unwrap();
    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-rotated-since",
        "a regular file frozen at the identical bytes would answer stale the moment \
         the target rotated"
    );
}

/// The certs are published one rename per consumer and nothing fuses them, so a
/// login killed part-way through leaves the consumers it reached holding the cert
/// and the rest holding nothing — a desk where netd resolves no binding while the
/// CLI resolves one. The next certifying login caches to a consumer that is EMPTY
/// whether or not it cleared anything there: an empty consumer has nothing to
/// preserve, and leaving it empty is the same silent half-binding again.
#[test]
#[serial]
fn a_consumer_left_empty_by_an_interrupted_login_is_cached_by_the_next_one() {
    let (port, _served) = start_certifying_issuer("account-b", "cert-reissued-to-account-b", 8);
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    // What a kill between the two commits leaves: the CLI's own path took the
    // cert, netd's never did, and the store names the account both belong to.
    let netd_cert = elsewhere.path().join("desk.cert");
    std::fs::write(home.path().join("device.cert"), "cert-issued-to-account-b").unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    let mut buf = SharedBuf::new();
    login_cmd::run_login(&mut buf).expect("a same-account re-login succeeds");

    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "cert-reissued-to-account-b",
        "the consumer the interrupted login never reached holds this login's cert, \
         instead of staying empty because no clear happened to touch it"
    );
    assert_eq!(
        std::fs::read_to_string(home.path().join("device.cert")).unwrap(),
        "cert-reissued-to-account-b",
        "and the consumer that DID take one is updated as always: the two agree"
    );
}

/// The put-back does not wait for another login. A command that RESOLVES the
/// binding runs it when the cert it needs is absent, so an interrupted login is
/// finished by the read that noticed rather than leaving the desk unbound until
/// someone happens to sign in again.
///
/// It restores the ASIDE and nothing else. Handing the restored bytes to the
/// other consumers is a separate step, and it belongs to the reader that has
/// VERIFIED them (`device_binding_resolve_test`) — a cert copied around on the
/// strength of its bytes could be the previous account's.
#[cfg(unix)]
#[test]
#[serial]
fn a_reader_of_the_binding_finishes_an_interrupted_login_itself() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    let netd_cert = elsewhere.path().join("desk.cert");
    // A login killed after its clear: the cert is at the aside, both consumers
    // read nothing, and the store still names the account the aside is tagged
    // with — so the binding it certifies is the one this desk is signed in as.
    std::fs::write(
        home.path().join(".device.cert.superseded.account-b"),
        "cert-issued-to-account-b",
    )
    .unwrap();
    std::fs::write(
        home.path().join("auth.json"),
        r#"{"account_id":"account-b","session_token":"b-session",
            "refresh_token":"b-refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let _cert = EnvGuard::set("CERULION_NETD_DEVICE_CERT", netd_cert.to_str().unwrap());

    auth::recover_interrupted_device_cert_state();

    assert_eq!(
        std::fs::read_to_string(&own_cert).unwrap(),
        "cert-issued-to-account-b",
        "the aside is put back where the reader looks for it"
    );
    assert!(
        std::fs::symlink_metadata(&netd_cert).is_err(),
        "and the OTHER consumer is left empty by this pass: what is at the aside has \
         not been verified as this machine's cert for this account, and a copy made \
         on the strength of the bytes alone is how a previous account's cert spreads"
    );
}

/// The unreadable cert, from the recovery side: the clear RENAMES it, so the
/// interrupted-login window holds it at the owner-tagged aside — the thing that
/// makes the transition recoverable at all. Asserting only that the live path is
/// empty would pass for a clear that deleted it outright.
///
/// Unix-only: the unreadable file is made with a mode.
#[cfg(unix)]
#[test]
#[serial]
fn a_cert_that_cannot_be_read_is_held_at_its_aside_until_the_login_publishes() {
    use std::os::unix::fs::PermissionsExt;

    let home = tempfile::tempdir().unwrap();
    let own_cert = home.path().join("device.cert");
    std::fs::write(&own_cert, "cert-issued-to-account-a").unwrap();
    std::fs::set_permissions(&own_cert, std::fs::Permissions::from_mode(0o000)).unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    let cleared = auth::clear_device_cert(Some("account-a")).expect("a rename needs no read");

    assert!(
        std::fs::symlink_metadata(&own_cert).is_err(),
        "the live path is empty for the window the login has not published yet"
    );
    let aside = home.path().join(".device.cert.superseded.account-a");
    assert!(
        std::fs::symlink_metadata(&aside).is_ok(),
        "and the cert is HELD at the owner-tagged aside, which is what a login \
         killed here can put back — a clear that deleted it could not"
    );
    cleared.put_back().expect("what is held can be put back");
    assert!(
        std::fs::symlink_metadata(&own_cert).is_ok(),
        "the rollback restores the entry it could never have read"
    );
    assert!(
        std::fs::symlink_metadata(&aside).is_err(),
        "and consumes the aside rather than leaving a second copy"
    );
}

#[test]
#[serial]
fn logout_revokes_the_session_and_the_gate_then_refuses() {
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let home = tempfile::tempdir().unwrap();
    let base = format!("http://127.0.0.1:{port}");
    let _svc = EnvGuard::set("CERULION_ACCOUNT_SERVICE", &base);
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

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
    authorize_via_magic_link(port, &email, &code);
    let signed_in = worker.join().unwrap().expect("run_login");

    let outcome = login_cmd::run_logout().expect("logout against a live service");
    assert_eq!(
        outcome,
        login_cmd::LogoutOutcome::SignedOut {
            account_id: signed_in.account_id.clone()
        }
    );

    let loaded = auth::load();
    match &loaded {
        LoadedAuth::SignedOut { account_id, .. } => {
            assert_eq!(account_id.as_deref(), Some(signed_in.account_id.as_str()));
        }
        other => panic!("expected SignedOut, got {other:?}"),
    }
    assert_eq!(
        auth::local_gate(&loaded, auth::now_unix_ns()),
        LocalGate::RefuseSignedOut
    );
    let text = std::fs::read_to_string(home.path().join("auth.json")).unwrap();
    assert!(
        !text.contains(&signed_in.session_token) && !text.contains(&signed_in.refresh_token),
        "no credential may survive a sign-out: {text}"
    );

    // The service retired the pair: the old refresh token no longer mints a session.
    let resp = reqwest::blocking::Client::new()
        .post(format!("{base}/v1/auth/refresh"))
        .json(&serde_json::json!({ "refresh_token": signed_in.refresh_token }))
        .send()
        .expect("refresh reachable");
    assert!(
        !resp.status().is_success(),
        "a revoked refresh token must be refused, got {}",
        resp.status()
    );

    let err = login_cmd::ensure_login_gate_with(&mut Vec::new(), false)
        .expect_err("a signed-out machine is refused");
    assert!(err.to_string().contains("signed out"), "{err}");

    assert_eq!(
        login_cmd::run_logout().expect("a second logout is a no-op"),
        login_cmd::LogoutOutcome::NotSignedIn
    );
}

#[test]
#[serial]
fn logout_without_the_service_still_signs_the_machine_out_and_says_so() {
    let home = tempfile::tempdir().unwrap();
    auth::seed_logged_in_at(home.path(), "acct-offline-logout").unwrap();
    let _svc = EnvGuard::set("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());

    match login_cmd::run_logout().expect("a local sign-out is not an error") {
        login_cmd::LogoutOutcome::SignedOutUnrevoked { account_id, reason } => {
            assert_eq!(account_id, "acct-offline-logout");
            assert!(reason.contains("unreachable"), "{reason}");
        }
        other => panic!("expected SignedOutUnrevoked, got {other:?}"),
    }
    assert!(matches!(
        auth::load(),
        LoadedAuth::SignedOut { account_id: Some(ref a), .. } if a == "acct-offline-logout"
    ));
}

#[test]
#[serial]
fn a_robot_that_logs_out_and_back_in_stays_a_robot() {
    let email = Arc::new(CapturingEmailSender::new());
    let port = start_accountd(email.clone());
    let home = tempfile::tempdir().unwrap();
    let _svc = EnvGuard::set(
        "CERULION_ACCOUNT_SERVICE",
        &format!("http://127.0.0.1:{port}"),
    );
    let _home = EnvGuard::set("CERULION_HOME", home.path().to_str().unwrap());
    let auth_path = home.path().join("auth.json");
    std::fs::write(
        &auth_path,
        br#"{"account_id":"prior-account","logged_in_ever":true,"role":"robot"}"#,
    )
    .unwrap();
    assert_eq!(
        auth::load_from(&auth_path).prior_identity(),
        (Some("prior-account"), Some(auth::MachineRole::Robot))
    );

    let log_in = || {
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
        authorize_via_magic_link(port, &email, &code);
        let state = worker.join().unwrap().expect("run_login");
        assert_eq!(state.role, Some(auth::MachineRole::Robot));
        state
    };

    let signed_in = log_in();
    assert_eq!(
        login_cmd::run_logout().expect("logout against a live service"),
        login_cmd::LogoutOutcome::SignedOut {
            account_id: signed_in.account_id.clone()
        }
    );
    assert_eq!(
        auth::load_from(&auth_path).prior_identity(),
        (
            Some(signed_in.account_id.as_str()),
            Some(auth::MachineRole::Robot)
        )
    );

    log_in();
    match auth::load_from(&auth_path) {
        LoadedAuth::Present(p) => assert_eq!(p.role, Some(auth::MachineRole::Robot)),
        other => panic!("expected Present, got {other:?}"),
    }
}
