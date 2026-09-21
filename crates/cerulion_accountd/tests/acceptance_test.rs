// SPDX-License-Identifier: AGPL-3.0-only
//! The acceptance test over a REAL axum server on an ephemeral port:
//!
//! 1. **Device-code login round-trips** — `device/start` → magic-link (captured
//!    from the pluggable email seam) → `magic-link/complete` → `device/poll`
//!    yields a session/refresh pair → `/v1/me` returns the account.
//! 2. **`POST /v1/devices` returns a `SignedDeviceCert` the SHIPPED verifier
//!    accepts against the SERVED root set** — the cert (from the real endpoint) +
//!    the root set (from `/.well-known/cerulion-roots`) drive
//!    `TrustStore::verify_new_pairing` to success. The grant is CA-issued via the
//!    lib (its endpoint is A4); the device cert + root set are what A0 must serve.
//! 3. Refresh + revoke round-trip and revocation actually kills the session.

use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use ed25519_dalek::SigningKey;
use serde_json::Value;

use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, ServiceConfig, UnconfiguredResolver,
};
use cerulion_pairing::format::{
    AccountId, PrincipalKind, PublicKey, RobotId, RootSet, Scope, SignedDeviceCert, SignedEpoch,
    SignedGrant, SignedIntermediateCert,
};
use cerulion_pairing::pop::{sign_pop, PURPOSE_DEVICE_REGISTRATION, PURPOSE_ROBOT_REGISTRATION};
use cerulion_pairing::verify::{OwnershipState, PairingPresentation, PairingSource, TrustStore};

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

/// Turn one entry of [`cerulion_accountd::TEAM_PAGE_ENDPOINT_EXPRESSIONS`] — a
/// JS URL *expression* the Team page builds, e.g.
/// `"/v1/robots/" + encodeURIComponent(rb.robot_id) + "/revoke"` — into the
/// concrete path this test probes.
///
/// String literals pass through; an `encodeURIComponent(…)` segment is replaced
/// by the matching probe id. An unrecognised segment PANICS rather than being
/// dropped: a silently-shortened path would probe the wrong route (or a 404
/// handler) and the 401 floor would pass without proving anything.
fn team_probe_path(expr: &str, robot_id: &str, device_id: &str) -> String {
    let mut out = String::new();
    for part in expr.split('+') {
        let p = part.trim();
        if let Some(lit) = p.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
            out.push_str(lit);
        } else if p.contains("robot_id") {
            out.push_str(robot_id);
        } else if p.contains("device_id") {
            out.push_str(device_id);
        } else {
            panic!(
                "unrecognised segment `{p}` in the endpoint expression `{expr}` — teach \
                 `team_probe_path` about it so the 401 floor keeps probing the real route"
            );
        }
    }
    out
}

/// Build a dev AppState with a capturing email seam + a real server on an
/// ephemeral port. Returns `(addr, state, captured_email_sender)`.
async fn spawn() -> (String, Arc<AppState>, Arc<CapturingEmailSender>) {
    spawn_with(ServiceConfig {
        device_code_interval_secs: 0, // no slow-down gate in the test
        verification_base_uri: "https://accounts.test".to_string(),
        ..ServiceConfig::default()
    })
    .await
}

/// Install the `ring` rustls crypto provider ONCE. Needed only when
/// reqwest was built with `rustls-no-provider` via cross-crate feature unification
/// (`cargo test -p cerulion_remoted -p cerulion_netd -p cerulion_accountd` — iroh pulls
/// rustls); a no-op / harmless otherwise. `install_default` returns `Err` if a provider
/// is already installed, which we ignore.
fn ensure_crypto_provider() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let _ = rustls::crypto::ring::default_provider().install_default();
    });
}

/// [`spawn`] with an explicit [`ServiceConfig`] (e.g. a 0-TTL PoP challenge to
/// exercise the expiry gate over HTTP).
async fn spawn_with(config: ServiceConfig) -> (String, Arc<AppState>, Arc<CapturingEmailSender>) {
    ensure_crypto_provider();
    let email = Arc::new(CapturingEmailSender::new());
    let state = Arc::new(
        AppState::dev(
            config,
            Clock::System,
            email.clone(),
            Arc::new(UnconfiguredResolver),
        )
        .expect("dev app state"),
    );

    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = format!("http://{}", listener.local_addr().unwrap());
    let serve_state = state.clone();
    tokio::spawn(async move {
        cerulion_accountd::serve(listener, serve_state)
            .await
            .unwrap();
    });
    (addr, state, email)
}

/// Drive a full hermetic device-code login for `email_addr` via the magic-link
/// seam and return `(session_token, account_id_b64)` — the shared setup for the
/// robot-ownership arm (a distinct email ⇒ a distinct account).
async fn login(
    addr: &str,
    http: &reqwest::Client,
    email: &Arc<CapturingEmailSender>,
    email_addr: &str,
) -> (String, String) {
    let start: Value = http
        .post(format!("{addr}/v1/auth/device/start"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let device_code = start["device_code"].as_str().unwrap().to_string();
    let user_code = start["user_code"].as_str().unwrap().to_string();

    let ml = http
        .post(format!("{addr}/v1/auth/magic-link/start"))
        .json(&serde_json::json!({ "email": email_addr, "user_code": user_code }))
        .send()
        .await
        .unwrap();
    assert_eq!(ml.status(), 202);
    let link = email.last_link().expect("a magic link was captured");
    let token = url::Url::parse(&link)
        .unwrap()
        .query_pairs()
        .find(|(k, _)| k == "token")
        .map(|(_, v)| v.into_owned())
        .expect("token in link");
    let complete = http
        .get(format!(
            "{addr}/v1/auth/magic-link/complete?token={token}&user_code={user_code}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(complete.status(), 200);

    let poll: Value = http
        .post(format!("{addr}/v1/auth/device/poll"))
        .json(&serde_json::json!({ "device_code": device_code }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let session = poll["session_token"].as_str().unwrap().to_string();

    let me: Value = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let account = me["account_id"].as_str().unwrap().to_string();
    (session, account)
}

/// A3 — fetch a proof-of-possession challenge for `session` and sign it with
/// `device_sk` (the private half of the key being registered) for `purpose`. Returns
/// `(pop_challenge, pop_signature_b64)` ready to attach to `POST /v1/robots` (purpose
/// = [`PURPOSE_ROBOT_REGISTRATION`]) or `POST /v1/devices` ([`PURPOSE_DEVICE_REGISTRATION`]).
/// The challenge response carries the account the challenge is bound to, so the signed
/// message matches exactly what the server rebuilds.
async fn pop_for(
    addr: &str,
    http: &reqwest::Client,
    session: &str,
    device_sk: &SigningKey,
    purpose: &str,
) -> (String, String) {
    let ch: Value = http
        .post(format!("{addr}/v1/devices/challenge"))
        .bearer_auth(session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let challenge = ch["challenge"].as_str().unwrap().to_string();
    let account_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(ch["account_id"].as_str().unwrap())
        .unwrap()
        .try_into()
        .unwrap();
    let pk = device_sk.verifying_key().to_bytes();
    let sig = sign_pop(device_sk, purpose, &account_bytes, &pk, &challenge);
    (challenge, URL_SAFE_NO_PAD.encode(sig.0))
}

/// Register `device_sk`'s public key as a device (`POST /v1/devices`) with a valid
/// device-registration PoP, returning the parsed JSON response. The SAME key signs the
/// PoP and is registered — one machine, one key.
async fn register_device_with_pop(
    addr: &str,
    http: &reqwest::Client,
    session: &str,
    device_sk: &SigningKey,
    principal_kind: &str,
) -> Value {
    let key_b64 = URL_SAFE_NO_PAD.encode(device_sk.verifying_key().to_bytes());
    let (challenge, signature) =
        pop_for(addr, http, session, device_sk, PURPOSE_DEVICE_REGISTRATION).await;
    http.post(format!("{addr}/v1/devices"))
        .bearer_auth(session)
        .json(&serde_json::json!({
            "public_key": key_b64,
            "principal_kind": principal_kind,
            "pop_challenge": challenge,
            "pop_signature": signature,
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap()
}

#[tokio::test]
async fn device_code_login_round_trips_and_issued_cert_is_verifier_accepted() {
    let (addr, state, email) = spawn().await;
    let http = reqwest::Client::new();

    // --- 0. served root set (bootstrap) ------------------------------------
    let roots: Value = http
        .get(format!("{addr}/.well-known/cerulion-roots"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let served_root_set: RootSet =
        cerulion_accountd::decode_b64(roots["root_set"].as_str().unwrap()).unwrap();
    // The served bytes round-trip to the CA's actual root set.
    assert_eq!(&served_root_set, state.ca.root_set());

    // --- 1. device-authorization start -------------------------------------
    let start: Value = http
        .post(format!("{addr}/v1/auth/device/start"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let device_code = start["device_code"].as_str().unwrap().to_string();
    let user_code = start["user_code"].as_str().unwrap().to_string();
    assert!(!device_code.is_empty() && !user_code.is_empty());

    // --- 2. a first poll is pending (not yet authorized) -------------------
    let pending = http
        .post(format!("{addr}/v1/auth/device/poll"))
        .json(&serde_json::json!({ "device_code": device_code }))
        .send()
        .await
        .unwrap();
    assert_eq!(pending.status(), 400);
    let pending_body: Value = pending.json().await.unwrap();
    assert_eq!(pending_body["error"], "authorization_pending");

    // --- 3. the browser authorizes via a magic link ------------------------
    let ml = http
        .post(format!("{addr}/v1/auth/magic-link/start"))
        .json(&serde_json::json!({ "email": "pilot@example.com", "user_code": user_code }))
        .send()
        .await
        .unwrap();
    assert_eq!(ml.status(), 202);

    // Read the link from the captured email seam + extract its token.
    let link = email.last_link().expect("a magic link was captured");
    let parsed = url::Url::parse(&link).unwrap();
    let token = parsed
        .query_pairs()
        .find(|(k, _)| k == "token")
        .map(|(_, v)| v.into_owned())
        .expect("token in link");

    let complete = http
        .get(format!(
            "{addr}/v1/auth/magic-link/complete?token={token}&user_code={user_code}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(complete.status(), 200);
    assert_eq!(
        complete.json::<Value>().await.unwrap()["status"],
        "authorized"
    );

    // --- 4. poll now yields the token pair ---------------------------------
    let poll: Value = http
        .post(format!("{addr}/v1/auth/device/poll"))
        .json(&serde_json::json!({ "device_code": device_code }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let session_token = poll["session_token"].as_str().unwrap().to_string();
    let refresh_token = poll["refresh_token"].as_str().unwrap().to_string();
    assert_eq!(poll["token_type"], "Bearer");

    // A second poll is now redeemed (single-use).
    let reused = http
        .post(format!("{addr}/v1/auth/device/poll"))
        .json(&serde_json::json!({ "device_code": device_code }))
        .send()
        .await
        .unwrap();
    assert_eq!(reused.status(), 400);

    // --- 5. /v1/me with the session token ----------------------------------
    let me: Value = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(&session_token)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let me_account_b64 = me["account_id"].as_str().unwrap().to_string();
    assert_eq!(me["principal_kind"], "human");
    assert_eq!(me["email"], "pilot@example.com");
    assert!(me["orgs"].as_array().unwrap().is_empty());

    // --- 6. POST /v1/devices → a SignedDeviceCert (with a device-reg PoP) ---
    let device_sk = SigningKey::from_bytes(&[7u8; 32]);
    let device_pub = device_sk.verifying_key().to_bytes();
    let device_pub_b64 = URL_SAFE_NO_PAD.encode(device_pub);

    let dev = register_device_with_pop(&addr, &http, &session_token, &device_sk, "human").await;
    let signed_cert: SignedDeviceCert =
        cerulion_accountd::decode_b64(dev["device_cert"].as_str().unwrap()).unwrap();
    let signed_inter: SignedIntermediateCert =
        cerulion_accountd::decode_b64(dev["intermediate"].as_str().unwrap()).unwrap();

    // The cert binds the key we sent to the caller's account (== /v1/me's).
    assert_eq!(signed_cert.cert.device_key, PublicKey(device_pub));
    assert_eq!(
        URL_SAFE_NO_PAD.encode(signed_cert.cert.account.0),
        me_account_b64
    );

    // --- 7. the SHIPPED verifier accepts the served cert + served root set --
    let now = now_ns();
    let subject = signed_cert.cert.account;
    let robot = RobotId([77u8; 32]);
    // The grant's endpoint is A4; issue it via the CA lib for this cross-check.
    let grant = state
        .ca
        .issue_grant(subject, robot, Scope::OWNER_FULL, PrincipalKind::Human, now);

    let chassis = b"acceptance-chassis";
    let mut store = TrustStore::provision(
        robot,
        PublicKey([9u8; 32]),
        served_root_set, // the root set fetched over HTTP
        chassis,
        now,
    )
    .unwrap();
    store
        .claim(AccountId([1u8; 32]), chassis, PrincipalKind::Human, now)
        .unwrap();

    let pres = PairingPresentation {
        intermediate: signed_inter,
        device_cert: signed_cert.clone(),
        grant,
        delegation: None,
    };
    let verified = store
        .verify_new_pairing(&pres, &PublicKey(device_pub), now)
        .expect("shipped verifier must accept the HTTP-issued cert + served root set");
    assert_eq!(verified.account(), subject);
    assert_eq!(verified.scope(), Scope::OWNER_FULL);

    // --- 8. GET /v1/devices lists the registered device --------------------
    let list: Value = http
        .get(format!("{addr}/v1/devices"))
        .bearer_auth(&session_token)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let devices = list["devices"].as_array().unwrap();
    assert_eq!(devices.len(), 1);
    assert_eq!(devices[0]["public_key"], device_pub_b64);

    // --- 9. refresh rotates the session, revoke kills it -------------------
    let refreshed: Value = http
        .post(format!("{addr}/v1/auth/refresh"))
        .json(&serde_json::json!({ "refresh_token": refresh_token }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let new_session = refreshed["session_token"].as_str().unwrap().to_string();
    // The new session authenticates.
    assert_eq!(
        http.get(format!("{addr}/v1/me"))
            .bearer_auth(&new_session)
            .send()
            .await
            .unwrap()
            .status(),
        200
    );

    // Revoke the new session → it no longer authenticates.
    let rev = http
        .post(format!("{addr}/v1/auth/revoke"))
        .json(&serde_json::json!({ "token": new_session }))
        .send()
        .await
        .unwrap();
    assert_eq!(rev.status(), 204);
    assert_eq!(
        http.get(format!("{addr}/v1/me"))
            .bearer_auth(&new_session)
            .send()
            .await
            .unwrap()
            .status(),
        401
    );
}

/// A2 — login-gated ownership at install, end-to-end over HTTP:
/// `POST /v1/robots` returns an OWNER_FULL owner grant, and feeding that grant
/// (with the robot's device cert + the served root set) into the SHIPPED
/// `TrustStore::claim_by_owner_grant` flips an unclaimed robot to
/// `Claimed(installer_account)`. Plus the ownership-scoping guards:
/// owner-only `GET`, foreign-key `POST` conflict.
#[tokio::test]
async fn post_v1_robots_owner_grant_claims_an_unclaimed_robot_end_to_end() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();

    // The served root set (the robot's factory trust anchor).
    let roots: Value = http
        .get(format!("{addr}/.well-known/cerulion-roots"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let served_root_set: RootSet =
        cerulion_accountd::decode_b64(roots["root_set"].as_str().unwrap()).unwrap();

    // The installer logs in on the robot.
    let (session, installer_account_b64) =
        login(&addr, &http, &email, "installer@robot.example").await;

    // The robot's OWN transport key is registered as a device (the key that logs in
    // on the robot IS the robot's transport identity — shell access is authority).
    let robot_sk = SigningKey::from_bytes(&[0x5A; 32]);
    let robot_transport_key = robot_sk.verifying_key().to_bytes();
    let robot_key_b64 = URL_SAFE_NO_PAD.encode(robot_transport_key);
    let dev = register_device_with_pop(&addr, &http, &session, &robot_sk, "human").await;
    let device_cert: SignedDeviceCert =
        cerulion_accountd::decode_b64(dev["device_cert"].as_str().unwrap()).unwrap();

    // POST /v1/robots binds the installer as owner and returns the owner grant. It
    // requires a proof-of-possession of the robot's transport key: the
    // installer signs a fresh challenge with the SAME key it registered.
    let (pop_challenge, pop_signature) = pop_for(
        &addr,
        &http,
        &session,
        &robot_sk,
        PURPOSE_ROBOT_REGISTRATION,
    )
    .await;
    let reg: Value = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "hostname": "orin-lab-01", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": robot_key_b64,
            "pop_challenge": pop_challenge,
            "pop_signature": pop_signature,
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let robot_id_b64 = reg["robot_id"].as_str().unwrap().to_string();
    // The owner grant binds the installer's account (== /v1/me's).
    assert_eq!(reg["account_id"], installer_account_b64);
    let owner_grant: SignedGrant =
        cerulion_accountd::decode_b64(reg["owner_grant"].as_str().unwrap()).unwrap();
    let inter: SignedIntermediateCert =
        cerulion_accountd::decode_b64(reg["intermediate"].as_str().unwrap()).unwrap();
    let robot_id_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(&robot_id_b64)
        .unwrap()
        .try_into()
        .unwrap();
    let installer_account_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(&installer_account_b64)
        .unwrap()
        .try_into()
        .unwrap();
    assert_eq!(
        owner_grant.grant.subject,
        AccountId(installer_account_bytes)
    );
    assert_eq!(owner_grant.grant.robot, RobotId(robot_id_bytes));
    assert_eq!(owner_grant.grant.scope, Scope::OWNER_FULL);

    // The robot's factory-provisioned UNCLAIMED store (its transport key + the
    // served root set) claims ownership OFFLINE from the returned grant.
    let now = now_ns();
    let chassis = b"a2-acceptance-chassis";
    let mut store = TrustStore::provision(
        RobotId(robot_id_bytes),
        PublicKey(robot_transport_key),
        served_root_set,
        chassis,
        now,
    )
    .unwrap();
    assert!(!store.is_claimed(), "the robot ships unclaimed");

    let pres = PairingPresentation {
        intermediate: inter,
        device_cert,
        grant: owner_grant,
        delegation: None,
    };
    let owner = store
        .claim_by_owner_grant(&pres, &PublicKey(robot_transport_key), now)
        .expect("the served owner grant must claim the unclaimed robot");

    // The robot is now Claimed(installer_account) with an OWNER_FULL owner row.
    assert_eq!(owner, AccountId(installer_account_bytes));
    assert_eq!(
        store.ownership(),
        OwnershipState::Claimed(AccountId(installer_account_bytes))
    );
    let row = store
        .is_allowed(&AccountId(installer_account_bytes), 0)
        .unwrap();
    assert_eq!(row.scope, Scope::OWNER_FULL);
    assert_eq!(row.source, PairingSource::OwnerGrant);

    // GET /v1/robots/{id} is owner-only: the installer reads it, a DIFFERENT
    // account gets 404 (the robot's existence is not revealed to non-owners).
    let got: Value = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(got["owner_account_id"], installer_account_b64);
    assert_eq!(got["hostname"], "orin-lab-01");

    let (other_session, _other_account) =
        login(&addr, &http, &email, "stranger@desk.example").await;
    let forbidden = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}"))
        .bearer_auth(&other_session)
        .send()
        .await
        .unwrap();
    assert_eq!(forbidden.status(), 404, "a non-owner cannot read the robot");

    // A second account claiming the SAME transport key is a 409 conflict (one key,
    // one owner — never a silent foreign owner grant). Even a caller that CAN prove
    // possession (this test holds `robot_sk`) is refused once the key is owned — the
    // PoP proves key ownership; the 409 enforces one-owner-per-key ON TOP of it.
    let (other_ch, other_sig) = pop_for(
        &addr,
        &http,
        &other_session,
        &robot_sk,
        PURPOSE_ROBOT_REGISTRATION,
    )
    .await;
    let conflict = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&other_session)
        .json(&serde_json::json!({
            "hostname": "impostor", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": robot_key_b64,
            "pop_challenge": other_ch,
            "pop_signature": other_sig,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(conflict.status(), 409);
}

/// Register `robot_sk`'s key as a robot for `session`, returning `robot_id_b64`. The
/// key is first registered as a device (its transport identity), then bound as a robot
/// via a PoP-signed `POST /v1/robots` — the A2 install funnel, reused by the A6 tests.
async fn register_robot_http(
    addr: &str,
    http: &reqwest::Client,
    session: &str,
    robot_sk: &SigningKey,
    hostname: &str,
) -> String {
    register_device_with_pop(addr, http, session, robot_sk, "human").await;
    let (ch, sig) = pop_for(addr, http, session, robot_sk, PURPOSE_ROBOT_REGISTRATION).await;
    let reg: Value = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(session)
        .json(&serde_json::json!({
            "hostname": hostname,
            "robot_transport_key": URL_SAFE_NO_PAD.encode(robot_sk.verifying_key().to_bytes()),
            "pop_challenge": ch,
            "pop_signature": sig,
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    reg["robot_id"].as_str().unwrap().to_string()
}

/// A6 — the owner-driven robot revocation surface end-to-end over HTTP:
/// `POST /v1/robots/{id}/revoke` (device + account), `GET /v1/robots/{id}/access`,
/// and the owner-only + exactly-one-target authz. The returned `signed_epoch` decodes
/// to a real `SignedEpoch` carrying the revoked device/account — the artifact the
/// robot applies via `TrustStore::apply_epoch`.
#[tokio::test]
async fn robot_revoke_surface_is_owner_only_and_bumps_signed_epochs() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();

    let (session, owner_account_b64) = login(&addr, &http, &email, "owner@robot.example").await;
    let robot_sk = SigningKey::from_bytes(&[0x5B; 32]);
    let robot_id_b64 = register_robot_http(&addr, &http, &session, &robot_sk, "orin-a6").await;
    let robot_id_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(&robot_id_b64)
        .unwrap()
        .try_into()
        .unwrap();

    // Owner self-lockout guard: revoking the robot's OWN owner account is REFUSED with
    // a 400 (it would permanently lock the robot out) — and mints NO epoch.
    let self_revoke = http
        .post(format!("{addr}/v1/robots/{robot_id_b64}/revoke"))
        .bearer_auth(&session)
        .json(&serde_json::json!({ "account_id": owner_account_b64 }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        self_revoke.status(),
        400,
        "revoking the robot's own owner account must be refused"
    );

    // No revocations yet: GET /access → epoch 0, empty sets, no signed epoch (the
    // refused self-revoke minted nothing).
    let access0: Value = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}/access"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(access0["epoch"], 0);
    assert!(access0["signed_epoch"].is_null());

    // Revoke a DEVICE key → epoch 1, the key appears in revoked_devices, and the
    // returned signed epoch is a real SignedEpoch carrying it.
    let desk_a_key = SigningKey::from_bytes(&[0x71; 32])
        .verifying_key()
        .to_bytes();
    let desk_a_b64 = URL_SAFE_NO_PAD.encode(desk_a_key);
    let rev1: Value = http
        .post(format!("{addr}/v1/robots/{robot_id_b64}/revoke"))
        .bearer_auth(&session)
        .json(&serde_json::json!({ "device_key": desk_a_b64 }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(rev1["epoch"], 1);
    assert_eq!(rev1["revoked_devices"][0], desk_a_b64);
    let signed1: SignedEpoch =
        cerulion_accountd::decode_b64(rev1["signed_epoch"].as_str().unwrap()).unwrap();
    assert_eq!(signed1.epoch_data.epoch, 1);
    assert_eq!(signed1.epoch_data.robot, RobotId(robot_id_bytes));
    assert_eq!(
        signed1.epoch_data.revoked_devices,
        vec![PublicKey(desk_a_key)]
    );
    assert!(signed1.epoch_data.revoked_accounts.is_empty());

    // Revoke an ACCOUNT → epoch 2, both sets present (the sets ACCUMULATE across
    // revokes — the device from epoch 1 is still there).
    let guest_account = SigningKey::from_bytes(&[0x99; 32])
        .verifying_key()
        .to_bytes();
    let guest_b64 = URL_SAFE_NO_PAD.encode(guest_account);
    let rev2: Value = http
        .post(format!("{addr}/v1/robots/{robot_id_b64}/revoke"))
        .bearer_auth(&session)
        .json(&serde_json::json!({ "account_id": guest_b64 }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(rev2["epoch"], 2);
    assert_eq!(rev2["revoked_accounts"][0], guest_b64);
    assert_eq!(rev2["revoked_devices"][0], desk_a_b64);

    // Re-revoking the SAME device is idempotent: no new epoch (still 2).
    let rev_again: Value = http
        .post(format!("{addr}/v1/robots/{robot_id_b64}/revoke"))
        .bearer_auth(&session)
        .json(&serde_json::json!({ "device_key": desk_a_b64 }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(rev_again["epoch"], 2, "re-revoke mints no new epoch");

    // GET /access reflects the latest epoch + a decodable signed epoch.
    let access2: Value = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}/access"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(access2["epoch"], 2);
    let signed2: SignedEpoch =
        cerulion_accountd::decode_b64(access2["signed_epoch"].as_str().unwrap()).unwrap();
    assert_eq!(signed2.epoch_data.epoch, 2);

    // Exactly-one-target: both / neither is a 400.
    for body in [
        serde_json::json!({ "device_key": desk_a_b64, "account_id": guest_b64 }),
        serde_json::json!({}),
    ] {
        let bad = http
            .post(format!("{addr}/v1/robots/{robot_id_b64}/revoke"))
            .bearer_auth(&session)
            .json(&body)
            .send()
            .await
            .unwrap();
        assert_eq!(bad.status(), 400, "exactly one of device_key/account_id");
    }

    // Owner-only: a STRANGER can neither revoke nor read the robot's access (404 —
    // the robot's existence is not revealed to non-owners).
    let (stranger, _sa) = login(&addr, &http, &email, "stranger@desk.example").await;
    let strv_revoke = http
        .post(format!("{addr}/v1/robots/{robot_id_b64}/revoke"))
        .bearer_auth(&stranger)
        .json(&serde_json::json!({ "device_key": desk_a_b64 }))
        .send()
        .await
        .unwrap();
    assert_eq!(strv_revoke.status(), 404, "a non-owner cannot revoke");
    let strv_access = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}/access"))
        .bearer_auth(&stranger)
        .send()
        .await
        .unwrap();
    assert_eq!(strv_access.status(), 404, "a non-owner cannot read access");
    // The stranger's revoke attempt minted NO epoch (owner's GET still reads 2).
    let access_after: Value = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}/access"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(access_after["epoch"], 2);
}

/// A6 — device self-revoke over HTTP: a caller may revoke its OWN device
/// (`POST /v1/devices/{id}/revoke`); another account's device is indistinguishably
/// 404; the revocation shows up in `GET /v1/devices`.
#[tokio::test]
async fn device_self_revoke_is_account_scoped() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();

    let (session, _a) = login(&addr, &http, &email, "alice@desk.example").await;
    let laptop_sk = SigningKey::from_bytes(&[0x61; 32]);
    let dev = register_device_with_pop(&addr, &http, &session, &laptop_sk, "human").await;
    let device_id = dev["device_id"].as_str().unwrap().to_string();

    // Self-revoke → 200, revoked == true.
    let revoked: Value = http
        .post(format!("{addr}/v1/devices/{device_id}/revoke"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(revoked["revoked"], true);
    assert_eq!(revoked["device_id"], device_id);

    // The enumeration reflects it.
    let list: Value = http
        .get(format!("{addr}/v1/devices"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let entry = list["devices"]
        .as_array()
        .unwrap()
        .iter()
        .find(|d| d["device_id"] == device_id.as_str())
        .unwrap();
    assert_eq!(entry["revoked"], true);

    // A DIFFERENT account cannot revoke Alice's device — 404 (never learns it exists).
    let (bob, _b) = login(&addr, &http, &email, "bob@desk.example").await;
    let forbidden = http
        .post(format!("{addr}/v1/devices/{device_id}/revoke"))
        .bearer_auth(&bob)
        .send()
        .await
        .unwrap();
    assert_eq!(
        forbidden.status(),
        404,
        "a stranger cannot revoke the device"
    );
}

/// A6 — device self-revoke is REAL enforcement, not a cosmetic flag: it fans
/// the device into every OWNED robot's revocation epoch AND refuses any future
/// re-registration of the key.
#[tokio::test]
async fn device_self_revoke_fans_into_owned_robots_and_blocks_reregistration() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();

    let (session, _acct) = login(&addr, &http, &email, "owner2@robot.example").await;

    // The account OWNS a robot (its own machine) + registers a second desk device.
    let robot_sk = SigningKey::from_bytes(&[0x6C; 32]);
    let robot_id_b64 =
        register_robot_http(&addr, &http, &session, &robot_sk, "orin-a6-fanout").await;
    let desk_sk = SigningKey::from_bytes(&[0x73; 32]);
    let desk_key_b64 = URL_SAFE_NO_PAD.encode(desk_sk.verifying_key().to_bytes());
    let dev = register_device_with_pop(&addr, &http, &session, &desk_sk, "human").await;
    let device_id = dev["device_id"].as_str().unwrap().to_string();

    // Self-revoke the desk device → it is fanned into the OWNED robot's revocation epoch.
    let revoked: Value = http
        .post(format!("{addr}/v1/devices/{device_id}/revoke"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(revoked["revoked"], true);
    assert_eq!(
        revoked["robots_updated"], 1,
        "the device is cut on the 1 robot the account owns"
    );

    // The owned robot's access epoch now lists the revoked device key (the enforcement
    // primitive: the robot refuses this key at its accept gate once it syncs).
    let access: Value = http
        .get(format!("{addr}/v1/robots/{robot_id_b64}/access"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert!(
        access["revoked_devices"]
            .as_array()
            .unwrap()
            .iter()
            .any(|d| d == &Value::String(desk_key_b64.clone())),
        "the owned robot's epoch must list the self-revoked device"
    );

    // A future RE-registration of the revoked key is refused (409) — the key is dead
    // until a fresh one is provisioned; it can never obtain a new device cert.
    let (challenge, signature) = pop_for(
        &addr,
        &http,
        &session,
        &desk_sk,
        PURPOSE_DEVICE_REGISTRATION,
    )
    .await;
    let rereg = http
        .post(format!("{addr}/v1/devices"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "public_key": desk_key_b64,
            "principal_kind": "human",
            "pop_challenge": challenge,
            "pop_signature": signature,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        rereg.status(),
        409,
        "a revoked device cannot obtain a fresh device cert"
    );

    // Idempotent: a second self-revoke mints no NEW epoch (robots_updated == 0).
    let revoked_again: Value = http
        .post(format!("{addr}/v1/devices/{device_id}/revoke"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    assert_eq!(
        revoked_again["robots_updated"], 0,
        "re-revoking a device already in the robot's epoch mints no new epoch"
    );
}

/// A2 — the robot endpoints refuse an UNAUTHENTICATED request with 401
/// (the auth gate is the only authorization primitive; a robot may only be
/// registered or read behind a valid session, invariant I4). A well-formed JSON
/// body is sent so the refusal is genuinely the missing session, not a body-parse
/// error.
#[tokio::test]
async fn robot_endpoints_refuse_unauthenticated_requests_with_401() {
    let (addr, _state, _email) = spawn().await;
    let http = reqwest::Client::new();

    // POST /v1/robots with NO bearer token → 401 (a valid body isolates the auth
    // gate from the Json extractor — the body must deserialize, incl. the A3 PoP
    // fields, so the handler's auth check runs and 401s rather than a 422 body reject).
    let robot_key_b64 = URL_SAFE_NO_PAD.encode([0x5Au8; 32]);
    let post = http
        .post(format!("{addr}/v1/robots"))
        .json(&serde_json::json!({
            "hostname": "orin-lab-01", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": robot_key_b64,
            "pop_challenge": "unused-no-session",
            "pop_signature": URL_SAFE_NO_PAD.encode([0u8; 64]),
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        post.status(),
        401,
        "POST /v1/robots without a session is unauthorized"
    );

    // GET /v1/robots/{id} with NO bearer token → 401 (auth is checked before the
    // robot id is even parsed, so any well-formed id 401s).
    let some_robot_id = URL_SAFE_NO_PAD.encode([0x11u8; 32]);
    let get = http
        .get(format!("{addr}/v1/robots/{some_robot_id}"))
        .send()
        .await
        .unwrap();
    assert_eq!(
        get.status(),
        401,
        "GET /v1/robots/{{id}} without a session is unauthorized"
    );
}

/// A2 — `POST /v1/robots` rejects malformed input with HTTP 400 at the
/// HTTP layer (behind a VALID session, so the refusal is the input, not auth): a
/// non-base64url / wrong-length transport key, and an empty hostname (the robot
/// identity).
#[tokio::test]
async fn post_v1_robots_rejects_malformed_input_with_400() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();
    let (session, _account) = login(&addr, &http, &email, "installer@robot.example").await;

    // A transport key that is not valid base64url (32 bytes) → 400. Dummy PoP fields
    // are attached only so the body DESERIALIZES; the key parse (before the PoP
    // checks) is what 400s.
    let dummy_sig = URL_SAFE_NO_PAD.encode([0u8; 64]);
    let bad_key = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "hostname": "orin-lab-01", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": "not-a-valid-key!!!",
            "pop_challenge": "dummy",
            "pop_signature": dummy_sig,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        bad_key.status(),
        400,
        "a malformed transport key is a bad request"
    );
    assert_eq!(
        bad_key.json::<Value>().await.unwrap()["error"],
        "invalid_request"
    );

    // A well-formed key but an EMPTY (whitespace) hostname → 400 (the hostname is
    // the robot identity; checked before the PoP).
    let good_key_b64 = URL_SAFE_NO_PAD.encode([0x5Au8; 32]);
    let empty_host = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "hostname": "   ",
            "robot_transport_key": good_key_b64,
            "pop_challenge": "dummy",
            "pop_signature": dummy_sig,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        empty_host.status(),
        400,
        "an empty hostname is a bad request"
    );
    assert_eq!(
        empty_host.json::<Value>().await.unwrap()["error"],
        "invalid_request"
    );
}

/// A3 — the proof-of-possession gate on `POST /v1/robots`, end-to-end over
/// HTTP. Closes the registration-squatting hole: a caller can only register a
/// transport key whose private half it actually holds, each challenge is single-use
/// (no replay), and a challenge is bound to its issuing account.
#[tokio::test]
async fn post_v1_robots_proof_of_possession_gate_over_http() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();
    let (session, account_b64) = login(&addr, &http, &email, "owner@robot.example").await;

    let post_robot = |host: &str, key_b64: String, challenge: String, sig_b64: String| {
        let http = http.clone();
        let addr = addr.clone();
        let session = session.clone();
        let host = host.to_string();
        async move {
            http.post(format!("{addr}/v1/robots"))
                .bearer_auth(&session)
                .json(&serde_json::json!({
                    "hostname": host,
                    "robot_transport_key": key_b64,
                    "pop_challenge": challenge,
                    "pop_signature": sig_b64,
                }))
                .send()
                .await
                .unwrap()
        }
    };

    // --- SQUATTING: register a key the caller does NOT hold → 403 --------------
    // The caller signs a PoP message claiming `victim_key` using its OWN key. The
    // server verifies against `victim_key` (which the caller cannot sign for), so the
    // signature does not verify — the squat is refused, the slot is NEVER burned.
    let victim_key = SigningKey::from_bytes(&[0xEE; 32])
        .verifying_key()
        .to_bytes();
    let victim_key_b64 = URL_SAFE_NO_PAD.encode(victim_key);
    let attacker_sk = SigningKey::from_bytes(&[0x11; 32]);
    let account_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(&account_b64)
        .unwrap()
        .try_into()
        .unwrap();
    // Fetch a real challenge for this account, but sign the WRONG key's message.
    let (squat_challenge, _throwaway) = pop_for(
        &addr,
        &http,
        &session,
        &attacker_sk,
        PURPOSE_ROBOT_REGISTRATION,
    )
    .await;
    let squat_sig = sign_pop(
        &attacker_sk,
        PURPOSE_ROBOT_REGISTRATION,
        &account_bytes,
        &victim_key, // claims the victim key...
        &squat_challenge,
    );
    let squat = post_robot(
        "squatted",
        victim_key_b64.clone(),
        squat_challenge,
        URL_SAFE_NO_PAD.encode(squat_sig.0),
    )
    .await;
    assert_eq!(
        squat.status(),
        403,
        "registering a key you do not hold must be refused (no squatting)"
    );
    assert_eq!(
        squat.json::<Value>().await.unwrap()["error"],
        "proof_of_possession_failed"
    );

    // --- MALFORMED signature → 400 (a bad-request, not a PoP failure) ----------
    // A non-64-byte signature is refused at the parse step (before any PoP check),
    // so no challenge is even needed here.
    let bad_sig = post_robot(
        "malformed",
        victim_key_b64.clone(),
        "some-challenge".to_string(),
        "not-64-bytes".to_string(),
    )
    .await;
    assert_eq!(
        bad_sig.status(),
        400,
        "a malformed signature is a bad request"
    );
    assert_eq!(
        bad_sig.json::<Value>().await.unwrap()["error"],
        "invalid_request"
    );

    // --- VALID registration with a held key succeeds --------------------------
    let robot_sk = SigningKey::from_bytes(&[0x5A; 32]);
    let robot_key_b64 = URL_SAFE_NO_PAD.encode(robot_sk.verifying_key().to_bytes());
    let (ch, sig) = pop_for(
        &addr,
        &http,
        &session,
        &robot_sk,
        PURPOSE_ROBOT_REGISTRATION,
    )
    .await;
    let ok = post_robot("orin-01", robot_key_b64.clone(), ch.clone(), sig.clone()).await;
    assert_eq!(ok.status(), 200, "a valid held-key registration succeeds");

    // --- REPLAY: the SAME (challenge, signature) again → 403 (single-use) ------
    // The signature still verifies, but the challenge was consumed by the first
    // register, so the spend CAS fails — a captured PoP cannot be replayed.
    let replay = post_robot("orin-01-replay", robot_key_b64.clone(), ch, sig).await;
    assert_eq!(
        replay.status(),
        403,
        "a replayed (already-consumed) challenge is refused"
    );
    assert_eq!(
        replay.json::<Value>().await.unwrap()["error"],
        "proof_of_possession_failed"
    );

    // --- WRONG ACCOUNT: account B spends account A's challenge → 403 -----------
    // A fetches a challenge (bound to A). B signs it with B's OWN key+account (so the
    // signature verifies), but the challenge belongs to A, so the account-bound spend
    // CAS fails — one account can never spend another's challenge.
    let (b_session, b_account_b64) = login(&addr, &http, &email, "other@desk.example").await;
    // A's challenge:
    let a_ch: Value = http
        .post(format!("{addr}/v1/devices/challenge"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let a_challenge = a_ch["challenge"].as_str().unwrap().to_string();
    // B signs A's challenge string with B's own key + B's account.
    let b_sk = SigningKey::from_bytes(&[0x7B; 32]);
    let b_key_b64 = URL_SAFE_NO_PAD.encode(b_sk.verifying_key().to_bytes());
    let b_account_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(&b_account_b64)
        .unwrap()
        .try_into()
        .unwrap();
    let b_sig = sign_pop(
        &b_sk,
        PURPOSE_ROBOT_REGISTRATION,
        &b_account_bytes,
        &b_sk.verifying_key().to_bytes(),
        &a_challenge,
    );
    let cross = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&b_session)
        .json(&serde_json::json!({
            "hostname": "b-robot", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": b_key_b64,
            "pop_challenge": a_challenge,
            "pop_signature": URL_SAFE_NO_PAD.encode(b_sig.0),
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        cross.status(),
        403,
        "one account cannot spend another account's challenge"
    );
}

/// A3 — an EXPIRED PoP challenge is refused over HTTP. A 0-TTL config makes
/// every issued challenge expire immediately, so a valid signature over a freshly
/// issued (but already-expired) challenge is still refused at the spend gate.
#[tokio::test]
async fn post_v1_robots_expired_pop_challenge_is_refused() {
    let (addr, _state, email) = spawn_with(ServiceConfig {
        device_code_interval_secs: 0,
        pop_challenge_ttl_ns: 0, // every challenge is born expired
        verification_base_uri: "https://accounts.test".to_string(),
        ..ServiceConfig::default()
    })
    .await;
    let http = reqwest::Client::new();
    let (session, _account) = login(&addr, &http, &email, "owner@robot.example").await;

    let robot_sk = SigningKey::from_bytes(&[0x5A; 32]);
    let robot_key_b64 = URL_SAFE_NO_PAD.encode(robot_sk.verifying_key().to_bytes());
    let (challenge, signature) = pop_for(
        &addr,
        &http,
        &session,
        &robot_sk,
        PURPOSE_ROBOT_REGISTRATION,
    )
    .await;
    let resp = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "hostname": "orin-01", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": robot_key_b64,
            "pop_challenge": challenge,
            "pop_signature": signature,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        403,
        "an expired challenge is refused even with a valid signature"
    );
    assert_eq!(
        resp.json::<Value>().await.unwrap()["error"],
        "proof_of_possession_failed"
    );
}

/// The verify-BEFORE-spend ordering invariant. A request
/// carrying a valid, account-owned challenge but a WRONG signature is refused (403)
/// WITHOUT consuming the challenge, so the legitimate owner's retry with the CORRECT
/// signature over the SAME challenge succeeds. A spend-then-verify reorder would burn
/// the challenge on the bad-signature attempt and fail the retry.
#[tokio::test]
async fn a_bad_signature_does_not_burn_the_challenge_verify_precedes_spend() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();
    let (session, _account) = login(&addr, &http, &email, "owner@robot.example").await;

    let robot_sk = SigningKey::from_bytes(&[0x5A; 32]);
    let robot_key_b64 = URL_SAFE_NO_PAD.encode(robot_sk.verifying_key().to_bytes());

    // Fetch ONE challenge (bound to this account).
    let ch: Value = http
        .post(format!("{addr}/v1/devices/challenge"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let challenge = ch["challenge"].as_str().unwrap().to_string();
    let account_bytes: [u8; 32] = URL_SAFE_NO_PAD
        .decode(ch["account_id"].as_str().unwrap())
        .unwrap()
        .try_into()
        .unwrap();
    let robot_pub = robot_sk.verifying_key().to_bytes();

    // A WRONG (but well-formed) signature: the CORRECT message signed by a DIFFERENT
    // key → BadSignature, not BadKey. Present it for the same challenge.
    let wrong_sk = SigningKey::from_bytes(&[0xC3; 32]);
    let wrong_sig = sign_pop(
        &wrong_sk,
        PURPOSE_ROBOT_REGISTRATION,
        &account_bytes,
        &robot_pub,
        &challenge,
    );
    let post = |challenge: String, sig_b64: String| {
        let http = http.clone();
        let addr = addr.clone();
        let session = session.clone();
        let key = robot_key_b64.clone();
        async move {
            http.post(format!("{addr}/v1/robots"))
                .bearer_auth(&session)
                .json(&serde_json::json!({
                    "hostname": "orin-01", // leak-scan: allow host-field an invented robot name
                    "robot_transport_key": key,
                    "pop_challenge": challenge,
                    "pop_signature": sig_b64,
                }))
                .send()
                .await
                .unwrap()
        }
    };
    let bad = post(challenge.clone(), URL_SAFE_NO_PAD.encode(wrong_sig.0)).await;
    assert_eq!(bad.status(), 403, "a wrong signature is refused");
    assert_eq!(
        bad.json::<Value>().await.unwrap()["error"],
        "proof_of_possession_failed"
    );

    // The CORRECT signature over the SAME challenge now succeeds — proving the bad
    // attempt did NOT burn the challenge (verify ran before the spend).
    let good_sig = sign_pop(
        &robot_sk,
        PURPOSE_ROBOT_REGISTRATION,
        &account_bytes,
        &robot_pub,
        &challenge,
    );
    let ok = post(challenge, URL_SAFE_NO_PAD.encode(good_sig.0)).await;
    assert_eq!(
        ok.status(),
        200,
        "the correct signature over the SAME challenge must succeed (verify precedes spend)"
    );
}

/// A3 — the breaking `/v1/robots` contract (findings 5 + 7). A pre-A3 client
/// that omits the PoP fields must get the account service's STABLE, NAMED error body
/// (400 `invalid_request` pointing at `POST /v1/devices/challenge`) via the custom
/// `PopJson` extractor — NOT a bare framework 422 with a raw serde string.
#[tokio::test]
async fn omitting_pop_fields_yields_a_named_error_not_a_bare_422() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();
    let (session, _account) = login(&addr, &http, &email, "owner@robot.example").await;

    // A session-authed POST /v1/robots with NO pop_challenge/pop_signature (the pre-A3
    // body shape). The body deserializes as far as the required PoP fields, then fails.
    let robot_key_b64 = URL_SAFE_NO_PAD.encode([0x5Au8; 32]);
    let resp = http
        .post(format!("{addr}/v1/robots"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "hostname": "orin-01", // leak-scan: allow host-field an invented robot name
            "robot_transport_key": robot_key_b64,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        400,
        "an omitted-PoP body is a bad request, not a bare 422"
    );
    let body: Value = resp.json().await.unwrap();
    assert_eq!(body["error"], "invalid_request");
    assert!(
        body["error_description"]
            .as_str()
            .unwrap()
            .contains("/v1/devices/challenge"),
        "the error names the challenge-acquisition step: {}",
        body["error_description"]
    );

    // The SAME holds for POST /v1/devices (its PoP is also required).
    let dev_key_b64 = URL_SAFE_NO_PAD.encode([0x77u8; 32]);
    let dev_resp = http
        .post(format!("{addr}/v1/devices"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "public_key": dev_key_b64,
            "principal_kind": "human",
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(
        dev_resp.status(),
        400,
        "an omitted-PoP device body is a 400"
    );
    assert_eq!(
        dev_resp.json::<Value>().await.unwrap()["error"],
        "invalid_request"
    );
}

/// A6: `GET /v1/robots`, the Team & access page's entry
/// query: the caller's OWNED robots and nothing else.
///
/// Account-scoped by construction (`WHERE owner_account_id = <caller>`), so this pins
/// the isolation against a HAND oracle rather than a self-compare: two accounts each
/// register robots over the real HTTP surface, and each `GET /v1/robots` must return
/// EXACTLY its own set — by robot id AND hostname. Plus the authz floor: no bearer and
/// a REVOKED session are both 401 (never a silent empty list, which would read as
/// "you own no robots" and quietly hide a real fleet).
#[tokio::test]
async fn list_robots_is_owner_scoped_and_never_leaks_another_account() {
    let (addr, _state, email) = spawn().await;
    let http = reqwest::Client::new();

    // --- account A owns TWO robots ------------------------------------------
    let (session_a, acct_a) = login(&addr, &http, &email, "a@fleet.example").await;
    let a1_sk = SigningKey::from_bytes(&[0xA1; 32]);
    let a2_sk = SigningKey::from_bytes(&[0xA2; 32]);
    let a1 = register_robot_http(&addr, &http, &session_a, &a1_sk, "orin-alpha").await;
    let a2 = register_robot_http(&addr, &http, &session_a, &a2_sk, "orin-beta").await;

    // --- account B owns ONE (a distinct email ⇒ a distinct account) ---------
    let (session_b, _acct_b) = login(&addr, &http, &email, "b@fleet.example").await;
    let b1_sk = SigningKey::from_bytes(&[0xB1; 32]);
    let b1 = register_robot_http(&addr, &http, &session_b, &b1_sk, "go2-gamma").await;

    // `GET /v1/robots` as `session`, returned as a sorted (robot_id, hostname) list.
    async fn list(addr: &str, http: &reqwest::Client, session: &str) -> Vec<(String, String)> {
        let resp = http
            .get(format!("{addr}/v1/robots"))
            .bearer_auth(session)
            .send()
            .await
            .unwrap();
        assert_eq!(resp.status(), 200);
        let body: Value = resp.json().await.unwrap();
        let mut pairs: Vec<(String, String)> = body["robots"]
            .as_array()
            .unwrap()
            .iter()
            .map(|r| {
                (
                    r["robot_id"].as_str().unwrap().to_string(),
                    r["hostname"].as_str().unwrap().to_string(),
                )
            })
            .collect();
        pairs.sort();
        pairs
    }

    // Hand oracle: A sees exactly {a1, a2}; B sees exactly {b1}.
    let mut want_a = vec![
        (a1.clone(), "orin-alpha".to_string()),
        (a2.clone(), "orin-beta".to_string()),
    ];
    want_a.sort();
    assert_eq!(
        list(&addr, &http, &session_a).await,
        want_a,
        "GET /v1/robots must list EXACTLY the caller's own robots"
    );
    assert_eq!(
        list(&addr, &http, &session_b).await,
        vec![(b1.clone(), "go2-gamma".to_string())],
        "account B must see only its own robot — never A's"
    );
    // The fixtures are genuinely distinct, so the two oracles above are not vacuously
    // satisfied by both accounts seeing the same (or an empty) set.
    assert!(b1 != a1 && b1 != a2, "the fixtures must be distinct robots");

    // Every row carries the CALLER as owner (the page never renders foreign ownership)
    // and a real registration timestamp.
    let a_body: Value = http
        .get(format!("{addr}/v1/robots"))
        .bearer_auth(&session_a)
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    for r in a_body["robots"].as_array().unwrap() {
        assert_eq!(r["owner_account_id"], acct_a);
        assert!(r["created_at_ns"].as_u64().unwrap() > 0);
    }

    // --- authz floor: no bearer, and a REVOKED session, are both 401 --------
    let anon = http.get(format!("{addr}/v1/robots")).send().await.unwrap();
    assert_eq!(
        anon.status(),
        401,
        "an unauthenticated robot list must be 401, NEVER an empty 200 (which would \
         read as 'you own no robots' and hide a real fleet)"
    );
    assert_eq!(anon.json::<Value>().await.unwrap()["error"], "unauthorized");

    let (doomed, _) = login(&addr, &http, &email, "doomed@fleet.example").await;
    let revoke = http
        .post(format!("{addr}/v1/auth/revoke"))
        .json(&serde_json::json!({ "token": doomed }))
        .send()
        .await
        .unwrap();
    assert_eq!(revoke.status(), 204);
    assert_eq!(
        http.get(format!("{addr}/v1/robots"))
            .bearer_auth(&doomed)
            .send()
            .await
            .unwrap()
            .status(),
        401,
        "a revoked session must not list robots"
    );
}

/// A6: `GET /team` serves the Team & access page.
///
/// The document is deliberately NOT session-gated (a browser navigation cannot carry a
/// bearer header, and the HTML holds no account data), so this test pins BOTH halves of
/// that trade: the page is served openly AND every datum it renders is gated. It also
/// pins the served hardening headers and byte-identity with the compiled-in asset — a
/// handler that started templating account data into the page would fail the equality.
#[tokio::test]
async fn team_page_is_served_openly_while_its_data_stays_session_gated() {
    let (addr, _state, _email) = spawn().await;
    let http = reqwest::Client::new();

    // --- the document itself: open, HTML, uncached, hardened ---------------
    let resp = http.get(format!("{addr}/team")).send().await.unwrap();
    assert_eq!(
        resp.status(),
        200,
        "the Team page is served without a session"
    );
    let headers = resp.headers().clone();
    assert_eq!(
        headers.get("content-type").unwrap(),
        "text/html; charset=utf-8"
    );
    assert_eq!(headers.get("cache-control").unwrap(), "no-store");
    assert_eq!(
        headers.get("content-security-policy").unwrap(),
        cerulion_accountd::TEAM_PAGE_CSP,
        "the self-containment CSP must be served with the document"
    );
    assert_eq!(headers.get("x-content-type-options").unwrap(), "nosniff");
    assert_eq!(headers.get("referrer-policy").unwrap(), "no-referrer");

    let body = resp.text().await.unwrap();
    assert_eq!(
        body,
        cerulion_accountd::TEAM_PAGE_HTML,
        "the served page must be the compiled-in asset VERBATIM — no per-request \
         templating, so no account data can ever be baked into the open document"
    );
    // The open document carries no credential, and it announces itself to its host
    // (the credential-free handshake — the token arrives over IPC, never a URL).
    assert!(!body.contains("session_token="));
    assert!(body.contains("cmd: \"team_ready\""));

    // --- the other half: EVERY endpoint the page calls IS session-gated ----
    //
    // All six, not a sample. The two routes matter most: an ungated
    // `/revoke` would let any unauthenticated caller cut a desk (or an account)
    // off every robot it can name, and the open document is justified precisely
    // by the claim that none of its endpoints are open. Robot ids are arbitrary
    // here on purpose — auth is checked before the id is looked up, so a
    // nonexistent robot still 401s (the "with a session" arm below proves that
    // 401 really is the auth gate and not a blanket refusal).
    // The probed set is DERIVED from `TEAM_PAGE_ENDPOINT_EXPRESSIONS` — the same
    // constant `team.rs::page_calls_only_the_session_authed_endpoints` asserts is
    // EXACTLY the set of `api(` calls in the asset. So the coupling is mechanical
    // in both directions: an endpoint added to the page but not the constant fails
    // there, and once it is in the constant it is automatically probed here. (The
    // previous form hand-listed six calls and then asserted `calls.len() == 6`
    // against its own literal — always true, and carrying no information: a
    // seventh endpoint could ship with its auth gate entirely unproven.)
    let robot_id = URL_SAFE_NO_PAD.encode([0x77u8; 32]);
    let device_id = "device-that-does-not-exist";
    struct Call {
        method: reqwest::Method,
        path: String,
    }
    let calls: Vec<Call> = cerulion_accountd::TEAM_PAGE_ENDPOINT_EXPRESSIONS
        .iter()
        .map(|expr| {
            let path = team_probe_path(expr, &robot_id, device_id);
            let method = if path.ends_with("/revoke") {
                reqwest::Method::POST
            } else {
                reqwest::Method::GET
            };
            Call { method, path }
        })
        .collect();
    // Sanity on the derivation itself: the two mutation routes must be present and
    // classified as POSTs (they are the rows that matter most — an ungated
    // `/revoke` would let any unauthenticated caller cut a desk off every robot).
    assert_eq!(
        calls
            .iter()
            .filter(|c| c.method == reqwest::Method::POST)
            .count(),
        2,
        "expected exactly the two /revoke mutation routes among the derived calls: {:?}",
        calls.iter().map(|c| &c.path).collect::<Vec<_>>()
    );
    assert!(
        calls
            .iter()
            .any(|c| c.path == format!("/v1/robots/{robot_id}/revoke"))
            && calls
                .iter()
                .any(|c| c.path == format!("/v1/devices/{device_id}/revoke")),
        "the derivation must reach both mutation routes: {:?}",
        calls.iter().map(|c| &c.path).collect::<Vec<_>>()
    );

    for c in &calls {
        // (a) no credential at all.
        let anon = http
            .request(c.method.clone(), format!("{addr}{}", c.path))
            .json(&serde_json::json!({ "account_id": URL_SAFE_NO_PAD.encode([0x01u8; 32]) }))
            .send()
            .await
            .unwrap();
        assert_eq!(
            anon.status(),
            401,
            "{} {} must refuse an unauthenticated caller — the Team page can be open \
             precisely BECAUSE its data endpoints are not",
            c.method,
            c.path
        );

        // (b) a well-formed but bogus bearer — the realistic attack, and the arm
        // that proves the gate verifies the token rather than merely requiring
        // the header to be present.
        let bogus = http
            .request(c.method.clone(), format!("{addr}{}", c.path))
            .bearer_auth("not-a-real-session-token")
            .json(&serde_json::json!({ "account_id": URL_SAFE_NO_PAD.encode([0x01u8; 32]) }))
            .send()
            .await
            .unwrap();
        assert_eq!(
            bogus.status(),
            401,
            "{} {} must refuse a forged bearer",
            c.method,
            c.path
        );
    }

    // --- anti-tautology: 401 is the AUTH gate, not a blanket refusal -------
    //
    // With a REAL session the same two mutation routes get past auth and fail on
    // the target instead (404 — the owner-scoped routes never confirm existence).
    // Without this arm a handler that 401'd unconditionally — including for a
    // legitimate owner — would pass the floor above.
    let (session, _account) = login(&addr, &http, &_email, "gate-probe@example.com").await;
    for path in [
        format!("/v1/robots/{robot_id}/revoke"),
        format!("/v1/devices/{device_id}/revoke"),
    ] {
        let authed = http
            .post(format!("{addr}{path}"))
            .bearer_auth(&session)
            .json(&serde_json::json!({ "account_id": URL_SAFE_NO_PAD.encode([0x01u8; 32]) }))
            .send()
            .await
            .unwrap();
        assert_eq!(
            authed.status(),
            404,
            "{path} with a VALID session must get past auth and fail on the unknown target \
             — otherwise the 401s above prove nothing about the auth gate"
        );
    }
}
