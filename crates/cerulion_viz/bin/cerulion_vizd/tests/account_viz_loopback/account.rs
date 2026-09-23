// SPDX-License-Identifier: AGPL-3.0-only

use std::path::Path;
use std::sync::{mpsc, Arc};

use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, ServiceConfig, UnconfiguredResolver,
};
use cerulion_cli_engine::auth;
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PrincipalKind, RobotId, Scope};

use super::support::{BOUND, DESK_SEED, ROBOT_SEED};

pub struct Account {
    pub app: Arc<AppState>,
    pub owner: AccountId,
    pub robot: RobotId,
    pub now: u64,
    pub url: String,
    stop: Option<mpsc::Sender<()>>,
    done: mpsc::Receiver<()>,
    thread: Option<std::thread::JoinHandle<()>>,
}
impl Account {
    pub fn start(home: &Path) -> Self {
        let app = Arc::new(
            AppState::dev(
                ServiceConfig::default(),
                Clock::System,
                Arc::new(CapturingEmailSender::new()),
                Arc::new(UnconfiguredResolver),
            )
            .unwrap(),
        );
        // The robot's fixed test clock must be at or after the real dev CA's
        // issuance instant, otherwise strict not-before validation must refuse.
        let now = auth::now_unix_ns();
        let user = app
            .db
            .upsert_user_by_identity("email", "loopback-owner@example.test", None, 0, now)
            .unwrap();
        let owner = AccountId(user.account_id);
        let robot = app
            .db
            .register_robot(
                &owner.0,
                "loopback-robot",
                &DeviceIdentity::from_seed(&ROBOT_SEED).public_key().0,
                None,
                now,
            )
            .unwrap();
        let token = "account-viz-test-session";
        let refresh = "account-viz-test-refresh";
        let expiry = now + 600_000_000_000;
        app.db
            .insert_session(
                &user.user_id,
                &cerulion_accountd::hash_token(token),
                &cerulion_accountd::hash_token(refresh),
                expiry,
                expiry,
                now,
            )
            .unwrap();
        let device = app.ca.issue_device_cert(
            DeviceIdentity::from_seed(&DESK_SEED).public_key(),
            owner,
            PrincipalKind::Human,
            Scope::OWNER_FULL,
            now,
        );
        let leaf = cerulion_accountd::encode_b64(&device).unwrap();
        let intermediate = cerulion_accountd::encode_b64(app.ca.intermediate()).unwrap();
        let auth_path = home.join("auth.json");
        auth::with_store_lock(&auth_path, || {
            auth::write_to(
                &auth_path,
                &auth::AuthState {
                    account_id: cerulion_accountd::encode_b64(&owner.0).unwrap(),
                    session_token: token.into(),
                    refresh_token: refresh.into(),
                    expires_at_ns: expiry,
                    logged_in_ever: true,
                    role: None,
                },
            )?;
            std::fs::write(home.join("desk.key"), DESK_SEED)?;
            std::fs::write(home.join("device.cert"), &leaf)?;
            std::fs::write(
                home.join("device-chain.json"),
                serde_json::to_vec(
                    &serde_json::json!({"device_cert":leaf,"intermediate":intermediate}),
                )
                .unwrap(),
            )
        })
        .unwrap();
        let (ready, address) = mpsc::channel();
        let (stop, stopped) = mpsc::channel();
        let (finished, done) = mpsc::channel();
        let serving = app.clone();
        let thread = std::thread::spawn(move || {
            tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .enable_all()
                .build()
                .unwrap()
                .block_on(async move {
                    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
                    ready.send(listener.local_addr().unwrap()).unwrap();
                    let shutdown = tokio::task::spawn_blocking(move || stopped.recv());
                    tokio::select! {
                        result = cerulion_accountd::serve(listener, serving) => result.unwrap(),
                        result = shutdown => { let _ = result.unwrap(); },
                    }
                });
            let _ = finished.send(());
        });
        // Own the stop sender and thread before waiting for readiness. A startup
        // refusal/timeout therefore runs the same Drop path as a later failure.
        let mut account = Self {
            app,
            owner,
            robot: RobotId(robot.robot_id),
            now,
            url: String::new(),
            stop: Some(stop),
            done,
            thread: Some(thread),
        };
        account.url = format!("http://{}", address.recv_timeout(BOUND).unwrap());
        account
    }
}
impl Drop for Account {
    fn drop(&mut self) {
        let _ = self.stop.take().unwrap().send(());
        let stopped = self.done.recv_timeout(BOUND).is_ok();
        let result = if stopped {
            self.thread.take().unwrap().join()
        } else {
            // Detach only on failure so an unrelated panic can still unwind.
            self.thread.take();
            Ok(())
        };
        if !std::thread::panicking() {
            assert!(stopped, "account service must stop within its bound");
            result.unwrap();
        }
    }
}
