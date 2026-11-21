// SPDX-License-Identifier: AGPL-3.0-only
//! The credentials a machine keeps working on without a fresh login, end-to-end
//! against the REAL `cerulion_accountd` router on an ephemeral port (Principle
//! #13: no mock service), on a shared test clock so a certificate can be aged
//! without waiting. A pass-through proxy in front of the router counts requests
//! and can answer a chosen route itself, the shape of a service without it.
//!
//! Device-certificate renewal: a certificate inside its renewal window is renewed
//! with the same device key and the same device row; one outside it is left
//! untouched with no request; a refused renewal changes nothing and logs one
//! line; two concurrent renewals make exactly one registration request; no
//! renewal creates a key. Each caller renews a due certificate: an account
//! command through `require_session`, the automatic account step once per
//! process, and the robot's revocation worker after its fetch.
//!
//! The robot's revocation fetch: it proves possession of the robot's transport
//! key and needs no login session; it falls back to the session when the service
//! has no robot access route; a 429 fails the run without the session; it never
//! signs a challenge issued for another robot.
//!
//! `#[serial]`: the tests mutate the process env (`CERULION_ACCOUNT_SERVICE`,
//! `CERULION_HOME`, the renewal window), which is global.

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
use cerulion_cli_engine::device_cert_renewal::{
    renew_device_cert_if_due, renew_if_due, renew_once_within, RenewalOutcome, RENEW_WINDOW_ENV,
};
use cerulion_cli_engine::{account_cmd, auth, login_cmd, owner_certificate, robot_bootstrap};
use cerulion_pairing::format::SignedDeviceCert;
use serial_test::serial;

/// The stop flag of a revocation sync nothing stops.
#[cfg(unix)]
static NOT_STOPPING: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

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

impl Desk {
    fn cert_text(&self) -> String {
        std::fs::read_to_string(self.home.join("device.cert"))
            .unwrap()
            .trim()
            .to_string()
    }
    fn cert(&self) -> SignedDeviceCert {
        let bytes = URL_SAFE_NO_PAD.decode(self.cert_text()).unwrap();
        postcard::from_bytes(&bytes).unwrap()
    }
    fn registrations(&self) -> usize {
        self.log
            .lock()
            .unwrap()
            .iter()
            .filter(|l| l.starts_with("POST /v1/devices "))
            .count()
    }
    /// Advance the service clock so `left_ns` of the cached certificate remains,
    /// and return that instant for the client side.
    fn age_to(&self, left_ns: u64) -> u64 {
        let now = self.cert().cert.validity.not_after_ns - left_ns;
        self.clock.store(now, Ordering::SeqCst);
        now
    }
    /// Make the cached certificate due on this machine's own clock, for the
    /// callers that read it: a window longer than the certificate's lifetime, and
    /// the service clock moved up to now, in step with this one, so the reissued
    /// certificate outlives the cached one and starts at or before this clock's
    /// reading after the service answered.
    fn due_on_the_wall_clock(&self) -> EnvGuard {
        self.clock.store(wall_ns(), Ordering::SeqCst);
        EnvGuard::set(RENEW_WINDOW_ENV, "2592000")
    }
    fn failure_recorded(&self) -> bool {
        self.home.join(".device-renew.failed").exists()
    }
    /// The reissued certificate reached the CLI's copy and the owner chain.
    fn assert_renewed_from(&self, before: &str) {
        assert_ne!(
            self.cert_text(),
            before,
            "device.cert holds a new certificate"
        );
        assert_eq!(
            owner_certificate::load()
                .expect("the renewed chain is consistent")
                .device_cert,
            self.cert()
        );
        assert_eq!(self.registrations(), 1);
    }
}

/// A client clock that reads `first` once (the due decision) and `then` on every
/// later read (after the service answered).
fn stepping_clock(first: u64, then: u64) -> impl Fn() -> u64 {
    let read = std::sync::atomic::AtomicBool::new(false);
    move || {
        if read.swap(true, Ordering::SeqCst) {
            then
        } else {
            first
        }
    }
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
        EnvGuard::unset(RENEW_WINDOW_ENV),
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

#[test]
#[serial]
fn a_certificate_inside_its_window_is_renewed_with_the_same_key_and_device_row() {
    let desk = desk();
    let before = desk.cert();
    let devices = account_cmd::list_my_devices(&desk.state.session_token).unwrap();
    assert_eq!(devices.len(), 1);
    // One day left of seven: inside the default half-life window.
    let now = desk.age_to(DAY_NS);
    let outcome = renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap();
    let after = desk.cert();
    assert_eq!(
        outcome,
        RenewalOutcome::Renewed {
            old_not_after_ns: before.cert.validity.not_after_ns,
            new_not_after_ns: after.cert.validity.not_after_ns,
        }
    );
    assert_eq!(after.cert.device_key, before.cert.device_key);
    assert_eq!(after.cert.account, before.cert.account);
    assert_eq!(after.cert.principal_kind, before.cert.principal_kind);
    // The service clock is six days on: the new certificate runs seven from there.
    assert_eq!(after.cert.validity.not_after_ns, now + 7 * DAY_NS);
    assert_eq!(desk.registrations(), 1);
    // The same device row, no second one.
    let devices_after = account_cmd::list_my_devices(&desk.state.session_token).unwrap();
    assert_eq!(devices_after.len(), 1);
    assert_eq!(devices_after[0].device_id, devices[0].device_id);
    assert_eq!(devices_after[0].public_key, devices[0].public_key);
    // The owner chain reads the new leaf, and auth.json is unchanged.
    let chain = owner_certificate::load().expect("the renewed chain is consistent");
    assert_eq!(chain.device_cert, after);
    assert!(
        auth::load().state().cloned().unwrap() == desk.state,
        "auth.json is not written by a renewal"
    );
    // Renewed once: the new certificate is outside its window, so no request.
    assert_eq!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap(),
        RenewalOutcome::NotDue {
            not_after_ns: after.cert.validity.not_after_ns
        }
    );
    assert_eq!(desk.registrations(), 1);
}

#[test]
#[serial]
fn a_certificate_outside_its_window_is_untouched_and_no_request_is_made() {
    let desk = desk();
    let text = desk.cert_text();
    let not_after = desk.cert().cert.validity.not_after_ns;
    // 3.5 days and one second left: just outside the default window.
    let now = desk.age_to(7 * DAY_NS / 2 + 1_000_000_000);
    let calls = desk.log.lock().unwrap().len();
    assert_eq!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap(),
        RenewalOutcome::NotDue {
            not_after_ns: not_after
        }
    );
    assert_eq!(desk.log.lock().unwrap().len(), calls, "no request at all");
    assert_eq!(desk.cert_text(), text);
    // The window setting moves the boundary: four days renews it.
    let _window = EnvGuard::set(RENEW_WINDOW_ENV, "345600");
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap(),
        RenewalOutcome::Renewed { .. }
    ));
}

#[tracing_test::traced_test]
#[test]
#[serial]
fn a_refused_renewal_changes_nothing_and_says_so_once() {
    let desk = desk();
    let devices = account_cmd::list_my_devices(&desk.state.session_token).unwrap();
    account_cmd::revoke_my_device(&desk.state.session_token, &devices[0].device_id).unwrap();
    let text = desk.cert_text();
    let chain = std::fs::read(desk.home.join("device-chain.json")).unwrap();
    let now = desk.age_to(DAY_NS);
    let err = renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap_err();
    assert!(err.to_string().contains("409"), "{err}");
    assert_eq!(desk.cert_text(), text);
    assert_eq!(
        std::fs::read(desk.home.join("device-chain.json")).unwrap(),
        chain
    );
    // The logging entry, which reads the wall clock: a window longer than the
    // certificate's lifetime makes it due now. The failure above was recorded at
    // the service clock, six days ahead of this one, and a failure recorded after
    // now holds nothing back, so this attempt runs. No error to the caller, one
    // warning.
    let _window = EnvGuard::set(RENEW_WINDOW_ENV, "2592000");
    renew_if_due(&desk.state.session_token);
    logs_assert(|lines: &[&str]| {
        match lines
            .iter()
            .filter(|l| l.contains("could not be renewed") && l.contains("cerulion login"))
            .count()
        {
            1 => Ok(()),
            n => Err(format!("expected one warning line, got {n}")),
        }
    });
}

#[test]
#[serial]
fn two_concurrent_renewals_make_exactly_one_registration() {
    let desk = desk();
    let now = desk.age_to(DAY_NS);
    let session = desk.state.session_token.clone();
    let barrier = Arc::new(std::sync::Barrier::new(2));
    let renewers: Vec<_> = (0..2)
        .map(|_| {
            let (session, barrier) = (session.clone(), Arc::clone(&barrier));
            std::thread::spawn(move || {
                barrier.wait();
                renew_device_cert_if_due(&session, &|| now).unwrap()
            })
        })
        .collect();
    let outcomes: Vec<RenewalOutcome> = renewers.into_iter().map(|t| t.join().unwrap()).collect();
    assert_eq!(desk.registrations(), 1, "{outcomes:?}");
    assert_eq!(
        outcomes
            .iter()
            .filter(|o| matches!(o, RenewalOutcome::Renewed { .. }))
            .count(),
        1,
        "{outcomes:?}"
    );
    assert!(
        outcomes.iter().all(|o| matches!(
            o,
            RenewalOutcome::Renewed { .. }
                | RenewalOutcome::PeerRenewing
                | RenewalOutcome::NotDue { .. }
        )),
        "{outcomes:?}"
    );
}

#[test]
#[serial]
fn a_renewal_never_creates_a_device_key() {
    let desk = desk();
    let key = desk.home.join("identity").join("device.key");
    let legacy = desk.home.join("desk.key");
    for path in [&key, &legacy] {
        if path.exists() {
            std::fs::remove_file(path).unwrap();
        }
    }
    let text = desk.cert_text();
    let now = desk.age_to(DAY_NS);
    let err = renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap_err();
    assert!(err.to_string().contains("no device key"), "{err}");
    assert!(!key.exists() && !legacy.exists(), "no key was created");
    assert_eq!(desk.cert_text(), text);
    assert_eq!(desk.registrations(), 0);
}

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
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
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
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
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
    assert!(robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &NOT_STOPPING).is_err());
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
    let err = robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap_err();
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
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
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
    robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &NOT_STOPPING).unwrap();
    // No signed read: the robot refused the challenge and used its session.
    assert_eq!(access_requests(&desk), (1, 0, 1));
}

#[test]
#[serial]
fn a_publication_cut_short_is_finished_by_the_next_renewal() {
    let desk = desk();
    let chain_path = desk.home.join("device-chain.json");
    let login_chain = std::fs::read(&chain_path).unwrap();
    let now = desk.age_to(DAY_NS);
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap(),
        RenewalOutcome::Renewed { .. }
    ));
    // A stop between renaming device.cert and the chain: the chain still holds
    // the login's leaf, so the owner chain refuses and the renewal is due again.
    std::fs::write(&chain_path, &login_chain).unwrap();
    assert!(owner_certificate::load().is_err());
    // A second later, so the reissued certificate outlives the one in place.
    desk.clock.fetch_add(1_000_000_000, Ordering::SeqCst);
    let outcome =
        renew_device_cert_if_due(&desk.state.session_token, &|| now + 1_000_000_000).unwrap();
    assert!(
        matches!(outcome, RenewalOutcome::Renewed { .. }),
        "{outcome:?}"
    );
    assert_eq!(owner_certificate::load().unwrap().device_cert, desk.cert());
    assert_eq!(desk.registrations(), 2);
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
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
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
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(access_requests(&desk), (2, 1, 0));
    // The next run's own clock is still far behind; its retry signs the
    // service's time, which is later than the one it accepted last.
    desk.clock.fetch_add(1_000_000_000, Ordering::SeqCst);
    desk.log.lock().unwrap().clear();
    robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &NOT_STOPPING).unwrap();
    assert_eq!(access_requests(&desk), (2, 1, 0));
}

#[test]
#[serial]
fn a_consumer_that_cannot_be_written_is_skipped_and_retried_at_most_hourly() {
    use std::os::unix::fs::MetadataExt;
    use std::os::unix::fs::PermissionsExt;
    // Root writes through a 0500 directory, so the unwritable shape needs a user.
    let probe = tempfile::NamedTempFile::new().unwrap();
    if probe.as_file().metadata().unwrap().uid() == 0 {
        eprintln!("skipped: root writes through a read-only directory");
        return;
    }
    let desk = desk();
    let login_cert = desk.cert_text();
    let now = desk.age_to(DAY_NS);
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap(),
        RenewalOutcome::Renewed { .. }
    ));
    // netd's relocated copy, in a directory this user cannot write, still on the
    // login's certificate: a mismatch the renewal cannot repair.
    let ro = desk.home.join("netd-ro");
    std::fs::create_dir(&ro).unwrap();
    let relocated = ro.join("device.cert");
    std::fs::write(&relocated, &login_cert).unwrap();
    std::fs::set_permissions(&ro, std::fs::Permissions::from_mode(0o500)).unwrap();
    let _netd = EnvGuard::set("CERULION_NETD_DEVICE_CERT", relocated.to_str().unwrap());
    desk.clock.fetch_add(1_000_000_000, Ordering::SeqCst);
    let later = now + 1_000_000_000;
    // The CLI's own copy and the chain still move to the new certificate; the
    // unwritable copy is skipped and named.
    let before = desk.cert_text();
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| later).unwrap(),
        RenewalOutcome::Renewed { .. }
    ));
    assert_eq!(desk.registrations(), 2);
    assert_ne!(desk.cert_text(), before);
    assert_eq!(std::fs::read_to_string(&relocated).unwrap(), login_cert);
    assert_eq!(owner_certificate::load().unwrap().device_cert, desk.cert());
    // Within the hour: not due, no request.
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| later + 60 * 1_000_000_000)
            .unwrap(),
        RenewalOutcome::NotDue { .. }
    ));
    assert_eq!(desk.registrations(), 2);
    assert!(desk.failure_recorded());
    // Writable again an hour on: the mismatch is due, published everywhere, and
    // the recorded failure cleared.
    std::fs::set_permissions(&ro, std::fs::Permissions::from_mode(0o700)).unwrap();
    let hour_on = later + 3600 * 1_000_000_000;
    desk.clock.store(hour_on, Ordering::SeqCst);
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| hour_on).unwrap(),
        RenewalOutcome::Renewed { .. }
    ));
    assert_eq!(desk.registrations(), 3);
    assert_eq!(
        std::fs::read_to_string(&relocated).unwrap().trim(),
        desk.cert_text()
    );
    assert!(!desk.failure_recorded());
}

#[test]
#[serial]
fn an_account_command_renews_a_due_certificate_over_its_session() {
    let desk = desk();
    let before = desk.cert_text();
    let _due = desk.due_on_the_wall_clock();
    let session = account_cmd::require_session().unwrap();
    assert_eq!(session, desk.state.session_token);
    desk.assert_renewed_from(&before);
}

#[test]
#[serial]
fn the_robot_worker_renews_a_due_certificate_after_its_fetch() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    let before = desk.cert_text();
    let _due = desk.due_on_the_wall_clock();
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    // The fetch by the robot's key, then one registration over the login session.
    assert_eq!(access_requests(&desk), (1, 1, 0));
    desk.assert_renewed_from(&before);
}

#[test]
#[serial]
fn the_automatic_account_step_renews_a_due_certificate_once_per_process() {
    let desk = desk();
    let before = desk.cert_text();
    let _due = desk.due_on_the_wall_clock();
    renew_once_within(Duration::from_secs(20));
    desk.assert_renewed_from(&before);
    // Still due (the window outlasts the certificate), but this process has had
    // its attempt.
    let renewed = desk.cert_text();
    renew_once_within(Duration::from_secs(20));
    assert_eq!(desk.cert_text(), renewed);
    assert_eq!(desk.registrations(), 1);
}

#[test]
#[serial]
fn a_robot_whose_login_has_lapsed_tries_a_renewal_at_most_once_an_hour() {
    let desk = desk();
    let (root, _robot_id, _revoked) = registered_robot(&desk);
    let before = desk.cert_text();
    let _due = desk.due_on_the_wall_clock();
    lapse_the_session(&desk);
    robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &NOT_STOPPING).unwrap();
    robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &NOT_STOPPING).unwrap();
    // Both runs fetch by the robot's key; only the first tries the session for
    // the renewal, fails, and records it, so the second asks nothing.
    assert_eq!(access_requests(&desk), (2, 2, 0));
    let refreshes = desk
        .log
        .lock()
        .unwrap()
        .iter()
        .filter(|l| l.contains("/v1/auth/refresh"))
        .count();
    assert_eq!(refreshes, 1);
    assert!(desk.failure_recorded());
    assert_eq!(desk.cert_text(), before);
    assert_eq!(desk.registrations(), 0);
}

#[test]
#[serial]
fn a_service_without_device_certificates_is_asked_at_most_once_an_hour() {
    let desk = desk();
    let now = desk.age_to(DAY_NS);
    desk.overrides.lock().unwrap().push((
        "/v1/devices/challenge ".into(),
        404,
        r#"{"error":"not_found"}"#.into(),
        false,
    ));
    assert_eq!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap(),
        RenewalOutcome::IdentityOnly
    );
    assert!(desk.failure_recorded());
    assert!(matches!(
        renew_device_cert_if_due(&desk.state.session_token, &|| now + 60 * 1_000_000_000).unwrap(),
        RenewalOutcome::NotDue { .. }
    ));
    let asked = desk
        .log
        .lock()
        .unwrap()
        .iter()
        .filter(|l| l.contains("/v1/devices/challenge"))
        .count();
    assert_eq!(asked, 1);
}

#[test]
#[serial]
fn a_publication_that_fails_after_replacing_a_copy_is_retried_at_once() {
    let desk = desk();
    // The chain path is a directory with an entry, so its rename fails after
    // device.cert was already replaced.
    let chain = desk.home.join("device-chain.json");
    let login_chain = std::fs::read(&chain).unwrap();
    std::fs::remove_file(&chain).unwrap();
    std::fs::create_dir(&chain).unwrap();
    std::fs::write(chain.join("entry"), "x").unwrap();
    let before = desk.cert_text();
    let now = desk.age_to(DAY_NS);
    let err = renew_device_cert_if_due(&desk.state.session_token, &|| now).unwrap_err();
    assert!(err.to_string().contains("device-chain.json"), "{err}");
    assert_ne!(desk.cert_text(), before);
    // A copy was already replaced, so no failure is recorded and nothing holds
    // the next attempt back.
    assert!(!desk.failure_recorded());
    // The chain path fixed (back on the login's leaf): due again at the same
    // `now`, and renewed. The service mints one second on, so the reissued
    // certificate outlives the one already in device.cert, and this clock reads
    // that second once the service answered.
    std::fs::remove_dir_all(&chain).unwrap();
    std::fs::write(&chain, &login_chain).unwrap();
    desk.clock.store(now + 1_000_000_000, Ordering::SeqCst);
    let outcome = renew_device_cert_if_due(
        &desk.state.session_token,
        &stepping_clock(now, now + 1_000_000_000),
    )
    .unwrap();
    assert!(
        matches!(outcome, RenewalOutcome::Renewed { .. }),
        "{outcome:?}"
    );
    assert_eq!(owner_certificate::load().unwrap().device_cert, desk.cert());
    assert_eq!(desk.registrations(), 2);
}

#[test]
#[serial]
fn the_reissued_certificate_is_checked_on_the_clock_read_after_the_service_answered() {
    let desk = desk();
    let now = desk.age_to(DAY_NS);
    // The service mints one second after this clock's due reading, and this
    // clock reads two seconds on once the service answered.
    desk.clock.store(now + 1_000_000_000, Ordering::SeqCst);
    let outcome = renew_device_cert_if_due(
        &desk.state.session_token,
        &stepping_clock(now, now + 2_000_000_000),
    )
    .unwrap();
    assert!(
        matches!(outcome, RenewalOutcome::Renewed { .. }),
        "{outcome:?}"
    );
    let renewed = desk.cert().cert.validity;
    assert_eq!(renewed.not_before_ns, now + 1_000_000_000);
    assert_eq!(renewed.not_after_ns, now + 1_000_000_000 + 7 * DAY_NS);
}

#[test]
#[serial]
fn a_reissued_certificate_starting_after_this_clock_reads_is_refused() {
    let desk = desk();
    let before = desk.cert_text();
    let now = desk.age_to(DAY_NS);
    // The service mints one second after the due reading, and this clock reads
    // only 999 ms on once it answered: the certificate is not yet valid here,
    // and peers would refuse it the same way.
    desk.clock.store(now + 1_000_000_000, Ordering::SeqCst);
    let err = renew_device_cert_if_due(
        &desk.state.session_token,
        &stepping_clock(now, now + 999_000_000),
    )
    .unwrap_err();
    assert!(
        err.to_string()
            .contains("not valid on this machine's clock"),
        "{err}"
    );
    assert_eq!(desk.cert_text(), before);
    assert_eq!(desk.registrations(), 1);
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
    robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &NOT_STOPPING).unwrap();
    assert_eq!(access_requests(&desk), (2, 1, 0));
    // The service at ten minutes ahead: this machine is ten minutes behind it
    // (a skew refusal naming no next time), and the service's own time is
    // two minutes behind the accepted request (a replay refusal naming the
    // next time it accepts).
    desk.clock.store(wall_ns() + 10 * minute, Ordering::SeqCst);
    desk.log.lock().unwrap().clear();
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
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
    robot_bootstrap::sync_epoch(&root, &mut out, &NOT_STOPPING).unwrap();
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

#[test]
#[serial]
fn a_sync_told_to_stop_tries_no_login_session_after_its_robot_proof_fails() {
    let desk = desk();
    let (root, _robot_id, _revoked) = registered_robot(&desk);
    // A stale session with a live refresh token: the fallback would refresh it.
    let mut stale = desk.state.clone();
    stale.expires_at_ns = 1;
    auth::write_to(&desk.home.join("auth.json"), &stale).unwrap();
    desk.overrides.lock().unwrap().push((
        "/access/challenge ".into(),
        403,
        r#"{"error":"proof_of_possession_failed","error_description":"refused"}"#.into(),
        false,
    ));
    let stopping = std::sync::atomic::AtomicBool::new(true);
    let err = robot_bootstrap::sync_epoch(&root, &mut Vec::new(), &stopping).unwrap_err();
    assert!(
        err.to_string()
            .contains("the revocation sync was told to stop"),
        "{err}"
    );
    // The proof fetch in flight ran; no refresh, no session read.
    assert_eq!(access_requests(&desk), (1, 0, 0));
    assert!(!desk
        .log
        .lock()
        .unwrap()
        .iter()
        .any(|l| l.contains("/v1/auth/refresh")));
    assert_eq!(
        auth::load_from(&desk.home.join("auth.json"))
            .state()
            .unwrap(),
        &stale
    );
}

#[test]
#[serial]
fn a_sync_told_to_stop_starts_no_certificate_renewal_after_its_fetch() {
    let desk = desk();
    let (root, _robot_id, revoked) = registered_robot(&desk);
    let before = desk.cert_text();
    let _due = desk.due_on_the_wall_clock();
    // A stale session with a live refresh token: the renewal would refresh it.
    let mut stale = desk.state.clone();
    stale.expires_at_ns = 1;
    auth::write_to(&desk.home.join("auth.json"), &stale).unwrap();
    let stopping = std::sync::atomic::AtomicBool::new(true);
    let mut out = Vec::new();
    robot_bootstrap::sync_epoch(&root, &mut out, &stopping).unwrap();
    assert_eq!(
        String::from_utf8(out).unwrap(),
        "{\"epoch\":1,\"version\":1}\n"
    );
    assert_eq!(
        pending_epoch(&root).signed_epoch.epoch_data.revoked_devices,
        vec![cerulion_pairing::format::PublicKey(revoked)]
    );
    assert_eq!(access_requests(&desk), (1, 1, 0));
    // Due, but not refreshed, not renewed and not counted as a failure: the
    // next sync renews.
    assert!(!desk
        .log
        .lock()
        .unwrap()
        .iter()
        .any(|l| l.contains("/v1/auth/refresh")));
    assert_eq!(desk.registrations(), 0);
    assert_eq!(desk.cert_text(), before);
    assert!(!desk.failure_recorded());
}
