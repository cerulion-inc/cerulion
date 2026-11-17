// SPDX-License-Identifier: AGPL-3.0-only
//! The credentials a machine keeps working on without a fresh login, end-to-end
//! against the REAL `cerulion_accountd` router on an ephemeral port (Principle
//! #13: no mock service), on a shared test clock so a certificate can be aged
//! without waiting. A pass-through proxy in front of the router counts requests
//! and can answer a chosen route itself, the shape of a service without it.
//!
//! The robot's revocation fetch: it proves possession of the robot's transport
//! key and needs no login session; it falls back to the session when the service
//! has no robot access route; a 429 fails the run without the session; it never
//! signs a challenge issued for another robot.
//!
//! `#[serial]`: the tests mutate the process env (`CERULION_ACCOUNT_SERVICE`,
//! `CERULION_HOME`), which is global.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Read, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, EmailSender, ServiceConfig, UnconfiguredResolver,
};
use cerulion_cli_engine::auth::AuthState;
use cerulion_cli_engine::{account_cmd, auth, login_cmd, robot_bootstrap};
use serial_test::serial;

const DAY_NS: u64 = 86_400 * 1_000_000_000;

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
    fn unset(key: &'static str) -> Self {
        let prev = std::env::var(key).ok();
        std::env::remove_var(key);
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

fn wall_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

/// The real router on a shared clock that starts at wall time. Sessions live
/// thirty days so a clock advanced by days still accepts the login's session.
fn start_accountd(email: Arc<CapturingEmailSender>, clock: Arc<AtomicU64>) -> u16 {
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
                session_ttl_ns: 30 * DAY_NS,
                ..ServiceConfig::default()
            };
            let state = Arc::new(
                AppState::dev(
                    config,
                    Clock::Shared(clock),
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

/// A pass-through proxy to `upstream` that records each request line. It asks
/// the upstream to close after one request (`Connection: close`), so every
/// request is one connection and none is missed.
/// Requests whose line contains the key are answered by the proxy itself; an
/// entry marked once answers one request and is then removed.
type Overrides = Arc<Mutex<Vec<(String, u16, String, bool)>>>;

fn start_counting_proxy(upstream: u16) -> (u16, Arc<Mutex<Vec<String>>>, Overrides) {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let overrides: Overrides = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::clone(&log);
    let answers = Arc::clone(&overrides);
    std::thread::spawn(move || {
        for client in listener.incoming() {
            let Ok(mut client) = client else { continue };
            let seen = Arc::clone(&seen);
            let answers = Arc::clone(&answers);
            std::thread::spawn(move || {
                let mut reader = BufReader::new(client.try_clone().unwrap());
                let mut request_line = String::new();
                if reader.read_line(&mut request_line).unwrap_or(0) == 0 {
                    return;
                }
                let mut head = request_line.clone();
                let mut length = 0usize;
                loop {
                    let mut header = String::new();
                    if reader.read_line(&mut header).unwrap_or(0) == 0 || header == "\r\n" {
                        break;
                    }
                    let lower = header.to_ascii_lowercase();
                    if let Some(v) = lower.strip_prefix("content-length:") {
                        length = v.trim().parse().unwrap_or(0);
                    }
                    if !lower.starts_with("connection:") {
                        head.push_str(&header);
                    }
                }
                head.push_str("Connection: close\r\n\r\n");
                let mut body = vec![0u8; length];
                reader.read_exact(&mut body).unwrap();
                seen.lock().unwrap().push(request_line.trim().to_string());
                let answer = {
                    let mut answers = answers.lock().unwrap();
                    let hit = answers
                        .iter()
                        .position(|(key, _, _, _)| request_line.contains(key.as_str()));
                    hit.map(|i| {
                        let (_, status, body, once) = answers[i].clone();
                        if once {
                            answers.remove(i);
                        }
                        (status, body)
                    })
                };
                if let Some((status, body)) = answer {
                    let response = format!(
                        "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\n\
                         Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
                        body.len()
                    );
                    client.write_all(response.as_bytes()).ok();
                    return;
                }
                let mut up = std::net::TcpStream::connect(("127.0.0.1", upstream)).unwrap();
                up.write_all(head.as_bytes()).unwrap();
                up.write_all(&body).unwrap();
                let mut response = Vec::new();
                up.read_to_end(&mut response).unwrap();
                client.write_all(&response).ok();
            });
        }
    });
    (port, log, overrides)
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

struct Desk {
    _home: tempfile::TempDir,
    home: std::path::PathBuf,
    clock: Arc<AtomicU64>,
    state: AuthState,
    log: Arc<Mutex<Vec<String>>>,
    overrides: Overrides,
    _env: Vec<EnvGuard>,
}

/// A signed-in desk whose account service is reached through the counting proxy.
fn desk() -> Desk {
    let email = Arc::new(CapturingEmailSender::new());
    let clock = Arc::new(AtomicU64::new(wall_ns()));
    let port = start_accountd(email.clone(), Arc::clone(&clock));
    let (proxy, log, overrides) = start_counting_proxy(port);
    let home = tempfile::tempdir().unwrap();
    let home_path = home.path().canonicalize().unwrap();
    let env = vec![
        EnvGuard::set(
            "CERULION_ACCOUNT_SERVICE",
            &format!("http://127.0.0.1:{proxy}"),
        ),
        EnvGuard::set("CERULION_HOME", home_path.to_str().unwrap()),
        EnvGuard::unset("CERULION_NETD_DEVICE_CERT"),
        EnvGuard::unset("CERULION_NETD_DESK_KEY"),
    ];
    let state = login(port, &email);
    // Count from here: the login made its own registration through the proxy.
    log.lock().unwrap().clear();
    Desk {
        _home: home,
        home: home_path,
        clock,
        state,
        log,
        overrides,
        _env: env,
    }
}

// ===========================================================================
// tests
// ===========================================================================

/// A registered robot on `desk`'s login with one revoked device, so it has an
/// epoch to fetch, and its remote-plane state directory provisioned.
fn registered_robot(desk: &Desk) -> (std::path::PathBuf, [u8; 32], [u8; 32]) {
    use account_cmd::{revoke_robot_access, RevokeTarget};
    use std::os::unix::fs::DirBuilderExt;
    let root = desk.home.join("robot-state");
    let prepared = robot_bootstrap::prepare(&root).unwrap();
    let robot_id: [u8; 32] = hex::decode(&prepared.robot_id).unwrap().try_into().unwrap();
    let revoked = [0x5c_u8; 32];
    revoke_robot_access(
        &desk.state.session_token,
        &URL_SAFE_NO_PAD.encode(robot_id),
        &RevokeTarget::Device(URL_SAFE_NO_PAD.encode(revoked)),
    )
    .unwrap();
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(root.join("remoted"))
        .unwrap();
    desk.log.lock().unwrap().clear();
    (root, robot_id, revoked)
}

fn pending_epoch(root: &std::path::Path) -> cerulion_pairing::verify::EpochSyncWire {
    let text = std::fs::read_to_string(root.join("remoted").join("epoch.pending")).unwrap();
    cerulion_pairing::verify::EpochSyncWire::from_postcard(&hex::decode(text.trim()).unwrap())
        .unwrap()
}

/// Replace the stored session with one the service never issued and that cannot
/// be refreshed, keeping the account: a login that lapsed while the robot was away.
fn lapse_the_session(desk: &Desk) {
    let path = desk.home.join("auth.json");
    let mut state = desk.state.clone();
    state.session_token = "lapsed-session".into();
    state.refresh_token = "lapsed-refresh".into();
    state.expires_at_ns = 0;
    auth::write_to(&path, &state).unwrap();
}

fn access_requests(desk: &Desk) -> (usize, usize, usize) {
    let log = desk.log.lock().unwrap();
    let count = |f: &dyn Fn(&str) -> bool| log.iter().filter(|l| f(l)).count();
    (
        count(&|l| l.starts_with("POST /v1/robots/") && l.contains("/access/challenge ")),
        count(&|l| l.starts_with("POST /v1/robots/") && l.contains("/access ")),
        count(&|l| l.starts_with("GET /v1/robots/") && l.contains("/access ")),
    )
}

#[test]
#[serial]
fn the_robot_fetches_its_revocations_with_its_key_and_no_login_session() {
    let desk = desk();
    let (root, robot_id, revoked) = registered_robot(&desk);
    lapse_the_session(&desk);
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    let wire = pending_epoch(&root);
    assert_eq!(wire.signed_epoch.epoch_data.robot.0, robot_id);
    assert_eq!(
        wire.signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    // One challenge, one signed read, no session read and no refresh attempt.
    assert_eq!(access_requests(&desk), (1, 1, 0));
    assert!(!desk
        .log
        .lock()
        .unwrap()
        .iter()
        .any(|l| l.contains("/v1/auth/refresh")));
}

#[test]
#[serial]
fn the_robot_falls_back_to_its_login_session_where_the_service_has_no_robot_route() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        404,
        r#"{"error":"not_found"}"#.into(),
        false,
    ));
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    assert_eq!(access_requests(&desk), (1, 0, 1));
    // With the session lapsed as well, the sync fails and the last epoch stays.
    let before = std::fs::read(root.join("remoted").join("epoch.pending")).unwrap();
    lapse_the_session(&desk);
    assert!(robot_bootstrap::sync_epoch(&root, &mut Vec::new()).is_err());
    assert_eq!(
        std::fs::read(root.join("remoted").join("epoch.pending")).unwrap(),
        before
    );
}

#[test]
#[serial]
fn a_rate_limited_robot_fails_its_run_without_using_its_login() {
    let desk = desk();
    let (root, _robot_id, _revoked) = registered_robot(&desk);
    let pending = root.join("remoted").join("epoch.pending");
    std::fs::write(&pending, "an epoch applied earlier").unwrap();
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        429,
        r#"{"error":"slow_down"}"#.into(),
        false,
    ));
    let mut out = Vec::new();
    let err = robot_bootstrap::sync_epoch(&root, &mut out).unwrap_err();
    // A failed run, so the supervisor backs off and counts it.
    assert!(err.to_string().contains("429"), "{err}");
    assert!(out.is_empty(), "no worker result line");
    // Nothing fetched, the earlier epoch left as it was, no session read or refresh.
    assert_eq!(access_requests(&desk), (1, 0, 0));
    assert_eq!(
        std::fs::read_to_string(&pending).unwrap(),
        "an epoch applied earlier"
    );
    assert!(!desk
        .log
        .lock()
        .unwrap()
        .iter()
        .any(|l| l.contains("/v1/auth/")));
}

#[test]
#[serial]
fn a_refused_robot_proof_falls_back_to_the_login_session() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        403,
        r#"{"error":"proof_of_possession_failed","error_description":"clock"}"#.into(),
        false,
    ));
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    assert_eq!(access_requests(&desk), (1, 0, 1));
}

#[test]
#[serial]
fn a_challenge_naming_another_robot_is_never_signed() {
    let desk = desk();
    let (root, _robot_id, _revoked) = registered_robot(&desk);
    let other = URL_SAFE_NO_PAD.encode([0x01_u8; 32]);
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        200,
        format!(r#"{{"challenge":"issued-elsewhere","robot_id":"{other}","expires_in":300}}"#),
        false,
    ));
    robot_bootstrap::sync_epoch(&root, &mut Vec::new()).unwrap();
    // No signed read: the robot refused the challenge and used its session.
    assert_eq!(access_requests(&desk), (1, 0, 1));
}

#[test]
#[serial]
fn a_refused_request_time_is_retried_once_at_the_time_the_service_names() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    // One refusal naming a later acceptable time (the shape of a replay after
    // this robot's clock stepped backwards), then the real service.
    let next = wall_ns() + 120 * 1_000_000_000;
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        403,
        format!(
            r#"{{"error":"proof_of_possession_failed","server_time_ns":"{}","next_requested_at_ns":"{next}"}}"#,
            wall_ns()
        ),
        true,
    ));
    lapse_the_session(&desk);
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    // Two challenge requests (the refused one and the retry), one signed read.
    assert_eq!(access_requests(&desk), (2, 1, 0));
}

#[test]
#[serial]
fn a_robot_whose_clock_is_far_behind_the_service_fetches_after_one_retry() {
    let desk = desk();
    let (root, _robot_id, _revoked) = registered_robot(&desk);
    // The service's clock ten minutes ahead of this machine's.
    desk.clock
        .store(wall_ns() + 600 * 1_000_000_000, Ordering::SeqCst);
    lapse_the_session(&desk);
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(access_requests(&desk), (2, 1, 0));
    // The next run's own clock is still far behind; its retry signs the
    // service's time, which is later than the one it accepted last.
    desk.clock.fetch_add(1_000_000_000, Ordering::SeqCst);
    desk.log.lock().unwrap().clear();
    robot_bootstrap::sync_epoch(&root, &mut Vec::new()).unwrap();
    assert_eq!(access_requests(&desk), (2, 1, 0));
}

#[test]
#[serial]
fn a_robot_whose_accepted_time_ran_ahead_and_whose_clock_then_fell_behind_fetches() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    lapse_the_session(&desk);
    let minute = 60 * 1_000_000_000;
    // The service twelve minutes ahead of this machine: a skew refusal, then
    // the retry at the service's time is accepted, so the robot's accepted
    // request time is now that time.
    desk.clock.store(wall_ns() + 12 * minute, Ordering::SeqCst);
    robot_bootstrap::sync_epoch(&root, &mut Vec::new()).unwrap();
    assert_eq!(access_requests(&desk), (2, 1, 0));
    // The service at ten minutes ahead: this machine is ten minutes behind it
    // (a skew refusal naming no next time), and the service's own time is
    // two minutes behind the accepted request (a replay refusal naming the
    // next time it accepts).
    desk.clock.store(wall_ns() + 10 * minute, Ordering::SeqCst);
    desk.log.lock().unwrap().clear();
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    // The skew refusal, the replay refusal, the request at the next time, and
    // one signed read; no session.
    assert_eq!(access_requests(&desk), (3, 1, 0));
}

#[test]
#[serial]
fn a_request_time_refused_three_times_falls_back_to_the_login_session() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        403,
        format!(
            r#"{{"error":"proof_of_possession_failed","server_time_ns":"{}"}}"#,
            wall_ns()
        ),
        false,
    ));
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    // Three challenge requests, no signed read, then the session read.
    assert_eq!(access_requests(&desk), (3, 0, 1));
}
