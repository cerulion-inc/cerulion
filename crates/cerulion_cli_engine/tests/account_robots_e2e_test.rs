// SPDX-License-Identifier: AGPL-3.0-only
//! The catalog client calls the real owner-only account-service HTTP router.

use std::sync::{mpsc, Arc};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, ServiceConfig, UnconfiguredResolver,
};
use cerulion_cli_engine::{account_robots, auth};
use cerulion_pairing::format::{AccountId, PublicKey, RobotId};
use serial_test::serial;

struct EnvGuard(&'static str, Option<std::ffi::OsString>);
impl EnvGuard {
    fn set(name: &'static str, value: impl AsRef<std::ffi::OsStr>) -> Self {
        let old = std::env::var_os(name);
        std::env::set_var(name, value);
        Self(name, old)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match self.1.take() {
            Some(value) => std::env::set_var(self.0, value),
            None => std::env::remove_var(self.0),
        }
    }
}

struct Server {
    stop: Option<mpsc::Sender<()>>,
    thread: Option<std::thread::JoinHandle<()>>,
    url: String,
}
impl Server {
    fn start(state: Arc<AppState>) -> Self {
        let (ready, address) = mpsc::channel();
        let (stop, stopped) = mpsc::channel();
        let thread = std::thread::spawn(move || {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move {
                    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                    ready.send(listener.local_addr().unwrap()).unwrap();
                    let shutdown = tokio::task::spawn_blocking(move || stopped.recv());
                    tokio::select! {
                        result = cerulion_accountd::serve(listener, state) => result.unwrap(),
                        result = shutdown => { result.unwrap().unwrap(); },
                    }
                });
        });
        Self {
            stop: Some(stop),
            thread: Some(thread),
            url: format!("http://{}", address.recv().unwrap()),
        }
    }
}
impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.stop.take().unwrap().send(());
        self.thread.take().unwrap().join().unwrap();
    }
}

#[test]
#[serial]
fn real_account_catalog_returns_only_owned_robot_and_refuses_revoked_session() {
    let now = auth::now_unix_ns();
    let app = Arc::new(
        AppState::dev(
            ServiceConfig::default(),
            Clock::System,
            Arc::new(CapturingEmailSender::new()),
            Arc::new(UnconfiguredResolver),
        )
        .unwrap(),
    );
    let owner = app
        .db
        .upsert_user_by_identity("email", "catalog-owner@example.test", None, 0, now)
        .unwrap();
    let other = app
        .db
        .upsert_user_by_identity("email", "catalog-other@example.test", None, 0, now)
        .unwrap();
    let robot_key = ed25519_dalek::SigningKey::from_bytes(&[0x71; 32])
        .verifying_key()
        .to_bytes();
    let foreign_key = ed25519_dalek::SigningKey::from_bytes(&[0x72; 32])
        .verifying_key()
        .to_bytes();
    let robot = app
        .db
        .register_robot(&owner.account_id, "owned-robot", &robot_key, None, now)
        .unwrap();
    app.db
        .register_robot(&other.account_id, "foreign-robot", &foreign_key, None, now)
        .unwrap();
    let token = "catalog-session-fixture";
    let expiry = now + 600_000_000_000;
    app.db
        .insert_session(
            &owner.user_id,
            &cerulion_accountd::hash_token(token),
            &cerulion_accountd::hash_token("catalog-refresh-fixture"),
            expiry,
            expiry,
            now,
        )
        .unwrap();
    let server = Server::start(Arc::clone(&app));
    let home = tempfile::tempdir().unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.path());
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", &server.url);
    let state = auth::AuthState {
        account_id: URL_SAFE_NO_PAD.encode(owner.account_id),
        session_token: token.into(),
        refresh_token: "catalog-refresh-fixture".into(),
        expires_at_ns: expiry,
        logged_in_ever: true,
        role: None,
    };
    auth::write_to(&home.path().join("auth.json"), &state).unwrap();
    std::fs::write(home.path().join("desk.key"), [0x73; 32]).unwrap();

    let directory = account_robots::fetch().unwrap();
    assert_eq!(directory.account_id, AccountId(owner.account_id));
    assert_eq!(
        directory.robots,
        vec![account_robots::AccountRobot {
            robot_id: RobotId(robot.robot_id),
            hostname: "owned-robot".into(),
            endpoint: account_robots::RobotEndpoint::Known(PublicKey(robot_key)),
        }]
    );
    directory.validate_current_login().unwrap();
    assert!(!home.path().join("device-chain.json").exists());
    assert!(!home.path().join("robots.toml").exists());
    app.db
        .revoke_session_by_token_hash(&cerulion_accountd::hash_token(token))
        .unwrap();
    let refusal = account_robots::fetch().unwrap_err().to_string();
    assert!(refusal.contains("refused the session"));
    assert!(!refusal.contains(token));
}

#[test]
#[serial]
fn redirect_does_not_make_a_second_http_request_with_the_session() {
    // This fixture speaks actual HTTP to test redirect transport behavior. It
    // does not impersonate the account router or fabricate a successful catalog.
    use std::io::{BufRead, BufReader, Write};
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    use std::time::{Duration, Instant};

    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    listener.set_nonblocking(true).unwrap();
    let address = listener.local_addr().unwrap();
    let finished = Arc::new(AtomicBool::new(false));
    let count = Arc::new(AtomicUsize::new(0));
    let worker = std::thread::spawn({
        let finished = Arc::clone(&finished);
        let count = Arc::clone(&count);
        move || {
            let deadline = Instant::now() + Duration::from_secs(5);
            while Instant::now() < deadline {
                match listener.accept() {
                    Ok((mut stream, _)) => {
                        stream
                            .set_read_timeout(Some(Duration::from_secs(1)))
                            .unwrap();
                        let mut reader = BufReader::new(&mut stream);
                        let mut line = String::new();
                        loop {
                            line.clear();
                            if reader.read_line(&mut line).unwrap() == 0 || line == "\r\n" {
                                break;
                            }
                        }
                        let previous = count.fetch_add(1, Ordering::SeqCst);
                        if previous == 0 {
                            write!(stream, "HTTP/1.1 302 Found\r\nLocation: http://{address}/redirect-target\r\nContent-Length: 0\r\nConnection: close\r\n\r\n").unwrap();
                        } else {
                            // Unpatched clients must complete so the assertion
                            // reports an extra request instead of timing out.
                            stream.write_all(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n").unwrap();
                        }
                    }
                    Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                        if finished.load(Ordering::SeqCst) {
                            break;
                        }
                        std::thread::sleep(Duration::from_millis(2));
                    }
                    Err(error) => panic!("HTTP fixture accept failed: {error}"),
                }
            }
        }
    });
    let home = tempfile::tempdir().unwrap();
    let _home = EnvGuard::set("CERULION_HOME", home.path());
    let _service = EnvGuard::set("CERULION_ACCOUNT_SERVICE", format!("http://{address}"));
    auth::write_to(
        &home.path().join("auth.json"),
        &auth::AuthState {
            account_id: URL_SAFE_NO_PAD.encode([0x11; 32]),
            session_token: "redirect-session-fixture".into(),
            refresh_token: "redirect-refresh-fixture".into(),
            expires_at_ns: auth::now_unix_ns() + 600_000_000_000,
            logged_in_ever: true,
            role: None,
        },
    )
    .unwrap();
    std::fs::write(home.path().join("desk.key"), [0x73; 32]).unwrap();
    let result = account_robots::fetch();
    finished.store(true, Ordering::SeqCst);
    worker.join().unwrap();
    let refusal = result.unwrap_err().to_string();
    assert_eq!(
        count.load(Ordering::SeqCst),
        1,
        "a redirect must not send a second request"
    );
    assert!(refusal.contains("HTTP 302"));
    assert!(!refusal.contains("redirect-session-fixture"));
}
