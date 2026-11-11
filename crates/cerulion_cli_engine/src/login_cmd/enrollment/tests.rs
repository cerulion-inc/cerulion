// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use cerulion_pairing::format::{
    AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, RobotId, RootSet, Scope,
    Validity, FORMAT_VERSION,
};
use cerulion_pairing::verify::{OwnerCertificatePresentationWire, PairingPresentation, TrustStore};
use serial_test::serial;

const ACCOUNT: AccountId = AccountId([17; 32]);
const SEED: [u8; 32] = [55; 32];
const TOKEN: &str = "enrollment-session-fixture";

struct Env(&'static str, Option<std::ffi::OsString>);
impl Env {
    fn unset(name: &'static str) -> Self {
        let previous = std::env::var_os(name);
        std::env::remove_var(name);
        Self(name, previous)
    }
    fn set(name: &'static str, value: impl AsRef<std::ffi::OsStr>) -> Self {
        let previous = std::env::var_os(name);
        std::env::set_var(name, value);
        Self(name, previous)
    }
}
impl Drop for Env {
    fn drop(&mut self) {
        if let Some(value) = &self.1 {
            std::env::set_var(self.0, value);
        } else {
            std::env::remove_var(self.0);
        }
    }
}

fn proof(seed: [u8; 32]) -> OwnerCertificatePresentationWire {
    proof_with_validity(
        seed,
        Validity {
            not_before_ns: 0,
            not_after_ns: u64::MAX,
        },
    )
}

fn proof_with_validity(seed: [u8; 32], validity: Validity) -> OwnerCertificatePresentationWire {
    let root = ed25519_dalek::SigningKey::from_bytes(&[1; 32]);
    let issuer = ed25519_dalek::SigningKey::from_bytes(&[2; 32]);
    OwnerCertificatePresentationWire::new(
        IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: PublicKey(issuer.verifying_key().to_bytes()),
            validity,
            issued_at_ns: 0,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&root]),
        DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(
                ed25519_dalek::SigningKey::from_bytes(&seed)
                    .verifying_key()
                    .to_bytes(),
            ),
            account: ACCOUNT,
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity,
            issued_at_ns: 0,
            issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
        }
        .sign(&issuer),
    )
}

#[derive(Clone, Copy)]
enum Mode {
    Allow,
    Refuse,
    WrongKey,
    ExpiredCertificate,
    Redirect,
    ChangedLogin,
    Hang,
}

struct Service {
    redirect_trap: TcpListener,
    count: Arc<AtomicUsize>,
    stopping: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
    url: String,
}
impl Service {
    fn start(mode: Mode, home: std::path::PathBuf) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        listener.set_nonblocking(true).unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let redirect_trap = TcpListener::bind("127.0.0.1:0").unwrap();
        redirect_trap.set_nonblocking(true).unwrap();
        let redirect = format!("http://{}/token-trap", redirect_trap.local_addr().unwrap());
        let count = Arc::new(AtomicUsize::new(0));
        let stopping = Arc::new(AtomicBool::new(false));
        let requests = Arc::clone(&count);
        let stop = Arc::clone(&stopping);
        let thread = std::thread::spawn(move || {
            while !stop.load(Ordering::SeqCst) {
                let (mut stream, _) = match listener.accept() {
                    Ok(pair) => pair,
                    Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(5));
                        continue;
                    }
                    Err(error) => panic!("HTTP accept: {error}"),
                };
                // macOS accepted sockets inherit the listener's nonblocking
                // flag. Headers use a bounded blocking read after accept.
                stream.set_nonblocking(false).unwrap();
                stream
                    .set_read_timeout(Some(Duration::from_secs(1)))
                    .unwrap();
                let mut reader = BufReader::new(&mut stream);
                let mut first = String::new();
                reader.read_line(&mut first).unwrap();
                let mut headers = Vec::new();
                let mut length = 0;
                loop {
                    let mut line = String::new();
                    reader.read_line(&mut line).unwrap();
                    if line == "\r\n" {
                        break;
                    }
                    if let Some(value) = line.to_ascii_lowercase().strip_prefix("content-length:") {
                        length = value.trim().parse::<usize>().unwrap();
                    }
                    headers.push(line.to_ascii_lowercase());
                }
                assert!(headers
                    .iter()
                    .any(|line| line.trim() == format!("authorization: bearer {TOKEN}")));
                assert!(headers
                    .iter()
                    .any(|line| line.trim() == "cerulion-pairing-protocol: 1"));
                let mut bytes = vec![0; length];
                reader.read_exact(&mut bytes).unwrap();
                drop(reader);
                requests.fetch_add(1, Ordering::SeqCst);
                if matches!(mode, Mode::Redirect) {
                    let response = format!("HTTP/1.1 302 Found\r\nLocation: {redirect}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
                    let _ = stream.write_all(response.as_bytes());
                    continue;
                }
                if matches!(mode, Mode::Hang) {
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }
                let (status, body) = if matches!(mode, Mode::Refuse) {
                    (
                        403,
                        serde_json::json!({"error_description": format!("enrollment refused for {TOKEN}")}),
                    )
                } else if first.starts_with("POST /v1/devices/challenge ") {
                    (
                        200,
                        serde_json::json!({"challenge":"enrollment-challenge", "account_id":URL_SAFE_NO_PAD.encode(ACCOUNT.0)}),
                    )
                } else if first.starts_with("POST /v1/devices ") {
                    let request: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
                    assert_eq!(
                        request["public_key"],
                        URL_SAFE_NO_PAD.encode(
                            ed25519_dalek::SigningKey::from_bytes(&SEED)
                                .verifying_key()
                                .to_bytes()
                        )
                    );
                    assert_eq!(request["pop_challenge"], "enrollment-challenge");
                    assert_eq!(request["principal_kind"], "human");
                    let signature = cerulion_pairing::format::Signature(
                        URL_SAFE_NO_PAD
                            .decode(request["pop_signature"].as_str().unwrap())
                            .unwrap()
                            .try_into()
                            .unwrap(),
                    );
                    assert_eq!(
                        cerulion_pairing::pop::verify_pop(
                            &PublicKey(
                                ed25519_dalek::SigningKey::from_bytes(&SEED)
                                    .verifying_key()
                                    .to_bytes()
                            ),
                            PURPOSE_DEVICE_REGISTRATION,
                            &ACCOUNT.0,
                            "enrollment-challenge",
                            &signature
                        ),
                        cerulion_pairing::pop::PopVerification::Ok
                    );
                    let certificate = proof_with_validity(
                        if matches!(mode, Mode::WrongKey) {
                            [56; 32]
                        } else {
                            SEED
                        },
                        Validity {
                            not_before_ns: 0,
                            not_after_ns: if matches!(mode, Mode::ExpiredCertificate) {
                                1
                            } else {
                                u64::MAX
                            },
                        },
                    );
                    (
                        200,
                        serde_json::json!({"device_id":"enrolled-device", "account_id":URL_SAFE_NO_PAD.encode(ACCOUNT.0), "device_cert":URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&certificate.device_cert).unwrap()), "intermediate":URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&certificate.intermediate).unwrap())}),
                    )
                } else {
                    assert!(first.starts_with("GET /v1/me "), "{first}");
                    if matches!(mode, Mode::ChangedLogin) {
                        let path = home.join("auth.json");
                        let mut changed = auth::load_from(&path).state().unwrap().clone();
                        changed.session_token = "replacement-session".into();
                        auth::with_store_lock(&path, || auth::write_to(&path, &changed)).unwrap();
                    }
                    (
                        200,
                        serde_json::json!({"account_id":URL_SAFE_NO_PAD.encode(ACCOUNT.0)}),
                    )
                };
                let body = body.to_string();
                let response = format!("HTTP/1.1 {status} Result\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: {}\r\n\r\n{body}", body.len());
                let _ = stream.write_all(response.as_bytes());
            }
        });
        Self {
            redirect_trap,
            count,
            stopping,
            thread: Some(thread),
            url,
        }
    }
}
impl Drop for Service {
    fn drop(&mut self) {
        self.stopping.store(true, Ordering::SeqCst);
        self.thread.take().unwrap().join().unwrap();
    }
}

struct Fixture {
    _environment: Vec<Env>,
    home: tempfile::TempDir,
    service: Service,
    regime: Arc<Mutex<EnrollmentRegime>>,
}
impl Fixture {
    fn new(mode: Mode, expires: bool) -> Self {
        let home = tempfile::tempdir().unwrap();
        let state = AuthState {
            account_id: URL_SAFE_NO_PAD.encode(ACCOUNT.0),
            session_token: TOKEN.into(),
            refresh_token: "enrollment-refresh-fixture".into(),
            expires_at_ns: if expires { 0 } else { u64::MAX },
            logged_in_ever: true,
            role: None,
        };
        auth::write_to(&home.path().join("auth.json"), &state).unwrap();
        auth::atomic_write_secret(&home.path().join("desk.key"), &SEED).unwrap();
        let service = Service::start(mode, home.path().to_owned());
        let environment = vec![
            Env::set("CERULION_HOME", home.path()),
            Env::set("CERULION_ACCOUNT_SERVICE", &service.url),
            Env::unset("CERULION_NETWORK"),
            Env::unset("CERULION_NETD_NETWORK"),
        ];
        Self {
            _environment: environment,
            home,
            service,
            regime: Arc::new(Mutex::new(EnrollmentRegime::default())),
        }
    }
    fn enroll(&self) -> CliResult<bool> {
        enroll_with_regime(Duration::from_secs(2), &self.regime)
    }
}

#[test]
#[serial]
#[tracing_test::traced_test]
fn a_valid_legacy_session_enrolls_the_existing_key_and_the_robot_accepts_it() {
    let fixture = Fixture::new(Mode::Allow, false);
    let auth_before = std::fs::read(fixture.home.path().join("auth.json")).unwrap();
    assert!(fixture.enroll().unwrap());
    assert_eq!(fixture.service.count.load(Ordering::SeqCst), 3);
    assert_eq!(
        std::fs::read(fixture.home.path().join("auth.json")).unwrap(),
        auth_before
    );
    assert_eq!(crate::login_cmd::ensure_device_key_seed().unwrap(), SEED);
    let enrolled = crate::owner_certificate::load().unwrap();
    assert_eq!(enrolled.device_cert.cert.account, AccountId([17; 32]));
    assert_eq!(
        enrolled.device_cert.cert.device_key,
        PublicKey(
            ed25519_dalek::SigningKey::from_bytes(&SEED)
                .verifying_key()
                .to_bytes()
        )
    );
    let issuer = ed25519_dalek::SigningKey::from_bytes(&[2; 32]);
    let root = ed25519_dalek::SigningKey::from_bytes(&[1; 32]);
    let owner = proof([44; 32]);
    let robot = RobotId([3; 32]);
    let now = auth::now_unix_ns();
    let mut store = TrustStore::provision(
        robot,
        owner.device_cert.cert.device_key,
        RootSet::new(vec![PublicKey(root.verifying_key().to_bytes())], 1).unwrap(),
        &[7; 32],
        now,
    )
    .unwrap();
    let grant = Grant {
        version: FORMAT_VERSION,
        subject: ACCOUNT,
        robot,
        scope: Scope::OWNER_FULL,
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity: Validity {
            not_before_ns: 0,
            not_after_ns: u64::MAX,
        },
        issued_at_ns: 0,
        issuer: ACCOUNT,
        issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
    }
    .sign(&issuer);
    store
        .claim_by_owner_grant(
            &PairingPresentation {
                intermediate: owner.intermediate,
                device_cert: owner.device_cert.clone(),
                grant,
                delegation: None,
            },
            &owner.device_cert.cert.device_key,
            now,
        )
        .unwrap();
    assert_eq!(
        store
            .verify_owner_certificate(&enrolled, &enrolled.device_cert.cert.device_key, now)
            .unwrap(),
        AccountId([17; 32])
    );
    let admitted = store
        .is_allowed(
            &AccountId([17; 32]),
            &enrolled.device_cert.cert.device_key,
            now,
        )
        .unwrap();
    assert_eq!(
        admitted.device,
        Some(PublicKey(
            ed25519_dalek::SigningKey::from_bytes(&[55; 32])
                .verifying_key()
                .to_bytes()
        ))
    );
    assert_eq!(
        admitted.source,
        cerulion_pairing::verify::PairingSource::CertificatePresentation
    );
    assert_eq!(admitted.scope, Scope::OWNER_FULL);
    assert!(!fixture.enroll().unwrap());
    assert_eq!(fixture.service.count.load(Ordering::SeqCst), 3);
    logs_assert(|lines| {
        let messages = lines
            .iter()
            .filter(|line| line.contains("this machine was enrolled as a device on your account"))
            .count();
        assert_eq!(messages, 1);
        Ok(())
    });
}

#[test]
#[serial]
fn an_expired_session_never_sends_a_token_and_names_the_enrollment_remedy() {
    let fixture = Fixture::new(Mode::Allow, true);
    assert_eq!(fixture.enroll().unwrap_err().to_string(), "this machine's sign-in has no device certificate, so the robot cannot verify it; run `cerulion login` again to enroll this machine");
    assert_eq!(fixture.service.count.load(Ordering::SeqCst), 0);
    assert!(!fixture.home.path().join("device.cert").exists());
}

#[test]
#[serial]
fn refused_or_wrong_key_enrollment_never_publishes_and_never_retries() {
    for mode in [
        Mode::Refuse,
        Mode::WrongKey,
        Mode::ChangedLogin,
        Mode::ExpiredCertificate,
        Mode::Redirect,
    ] {
        let fixture = Fixture::new(mode, false);
        assert!(fixture
            .enroll()
            .unwrap_err()
            .to_string()
            .contains("run `cerulion login` again to enroll this machine"));
        let requests = fixture.service.count.load(Ordering::SeqCst);
        assert!(!fixture.home.path().join("device.cert").exists());
        assert!(!fixture.home.path().join("device-chain.json").exists());
        assert!(fixture.enroll().is_err());
        assert_eq!(fixture.service.count.load(Ordering::SeqCst), requests);
        assert_eq!(
            fixture.service.redirect_trap.accept().unwrap_err().kind(),
            std::io::ErrorKind::WouldBlock
        );
        let state = fixture.regime.lock().unwrap();
        assert!(state.failures.total_failures() >= 2);
        assert!(!state
            .result
            .as_ref()
            .unwrap()
            .as_ref()
            .unwrap_err()
            .contains(TOKEN));
    }
}

#[test]
#[serial]
fn enrollment_never_creates_a_missing_device_key() {
    let fixture = Fixture::new(Mode::Allow, false);
    std::fs::remove_file(fixture.home.path().join("desk.key")).unwrap();
    assert_eq!(
        fixture.enroll().unwrap_err().to_string(),
        DEVICE_ENROLLMENT_REQUIRED
    );
    assert_eq!(fixture.service.count.load(Ordering::SeqCst), 0);
    assert!(!fixture.home.path().join("identity/device.key").exists());
    assert!(!fixture.home.path().join("device.cert").exists());
}

#[test]
#[serial]
fn a_hung_enrollment_is_bounded_and_local_scope_sends_nothing() {
    let fixture = Fixture::new(Mode::Hang, false);
    let started = Instant::now();
    assert!(enroll_with_regime(Duration::from_millis(80), &fixture.regime).is_err());
    assert!(started.elapsed() < Duration::from_secs(1));
    let _off = Env::set("CERULION_NETWORK", "off");
    let state = Arc::new(Mutex::new(EnrollmentRegime::default()));
    assert!(!enroll_with_regime(Duration::from_secs(2), &state).unwrap());
    assert!(!state.lock().unwrap().attempted);
}
