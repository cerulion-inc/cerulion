// SPDX-License-Identifier: AGPL-3.0-only
//! HTTP-surface coverage for the root fixes, over a REAL axum server:
//! - **E** negative-auth on the authed surface (absent / garbage / expired token).
//! - **F** refresh rotation is single-use; revoke-then-refresh is rejected.
//! - **B** two concurrent device polls of one authorized code mint EXACTLY one
//!   session (the atomic redeem).
//! - **A** cross-account device registration is a 409; a same-account re-register
//!   mints the cert from the STORED principal.
//! - **G** magic-link completion is single-use end to end.

use std::sync::Arc;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use ed25519_dalek::SigningKey;
use serde_json::Value;

use cerulion_accountd::{
    AppState, CapturingEmailSender, Clock, ServiceConfig, UnconfiguredResolver,
};
use cerulion_pairing::format::PrincipalKind;
use cerulion_pairing::pop::{sign_pop, PURPOSE_DEVICE_REGISTRATION};

/// Fetch a device-registration proof-of-possession for `sk` (the key
/// being registered). `POST /v1/devices` requires it, so every device-registration
/// arm must present a valid PoP over the SAME key it registers.
async fn device_pop(
    http: &reqwest::Client,
    addr: &str,
    session: &str,
    sk: &SigningKey,
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
    let account: [u8; 32] = URL_SAFE_NO_PAD
        .decode(ch["account_id"].as_str().unwrap())
        .unwrap()
        .try_into()
        .unwrap();
    let pk = sk.verifying_key().to_bytes();
    let sig = sign_pop(sk, PURPOSE_DEVICE_REGISTRATION, &account, &pk, &challenge);
    (challenge, URL_SAFE_NO_PAD.encode(sig.0))
}

fn test_config() -> ServiceConfig {
    ServiceConfig {
        device_code_interval_secs: 0, // no slow-down gate
        verification_base_uri: "https://accounts.test".to_string(),
        ..ServiceConfig::default()
    }
}

/// Install the `ring` rustls crypto provider ONCE. Needed only when
/// reqwest is built with `rustls-no-provider` via cross-crate feature unification with
/// iroh-pulling crates; a no-op otherwise.
fn ensure_crypto_provider() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let _ = rustls::crypto::ring::default_provider().install_default();
    });
}

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

async fn spawn() -> (String, Arc<AppState>, Arc<CapturingEmailSender>) {
    spawn_with(test_config()).await
}

fn token_from_link(link: &str) -> String {
    url::Url::parse(link)
        .unwrap()
        .query_pairs()
        .find(|(k, _)| k == "token")
        .map(|(_, v)| v.into_owned())
        .expect("token in link")
}

/// A full device-code + magic-link login. Returns `(session_token, refresh_token)`.
async fn login(
    http: &reqwest::Client,
    addr: &str,
    email_capture: &CapturingEmailSender,
    user_email: &str,
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

    http.post(format!("{addr}/v1/auth/magic-link/start"))
        .json(&serde_json::json!({ "email": user_email, "user_code": user_code }))
        .send()
        .await
        .unwrap();
    let token = token_from_link(&email_capture.last_link().expect("captured link"));
    http.get(format!(
        "{addr}/v1/auth/magic-link/complete?token={token}&user_code={user_code}"
    ))
    .send()
    .await
    .unwrap();

    let poll: Value = http
        .post(format!("{addr}/v1/auth/device/poll"))
        .json(&serde_json::json!({ "device_code": device_code }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    (
        poll["session_token"].as_str().unwrap().to_string(),
        poll["refresh_token"].as_str().unwrap().to_string(),
    )
}

// ============================================================================
// E — negative auth
// ============================================================================

#[tokio::test]
async fn me_without_bearer_is_401() {
    let (addr, _s, _e) = spawn().await;
    let resp = reqwest::Client::new()
        .get(format!("{addr}/v1/me"))
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), 401);
}

#[tokio::test]
async fn me_with_garbage_bearer_is_401() {
    let (addr, _s, _e) = spawn().await;
    let resp = reqwest::Client::new()
        .get(format!("{addr}/v1/me"))
        .bearer_auth("not-a-real-session-token")
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), 401);
}

#[tokio::test]
async fn me_returns_null_email_when_user_has_no_email() {
    let (addr, state, _e) = spawn().await;
    let user = state
        .db
        .upsert_user_by_identity(
            "test",
            "user-without-email",
            None,
            PrincipalKind::Human as u8,
            state.clock.now_ns(),
        )
        .unwrap();
    let session_token = "session-without-email";
    let refresh_token = "refresh-without-email";
    let now = state.clock.now_ns();
    state
        .db
        .insert_session(
            &user.user_id,
            &cerulion_accountd::hash_token(session_token),
            &cerulion_accountd::hash_token(refresh_token),
            now + state.config.session_ttl_ns,
            now + state.config.refresh_ttl_ns,
            now,
        )
        .unwrap();

    let response = reqwest::Client::new()
        .get(format!("{addr}/v1/me"))
        .bearer_auth(session_token)
        .send()
        .await
        .unwrap();
    assert_eq!(response.status(), reqwest::StatusCode::OK);
    let me: Value = response.json().await.unwrap();
    assert_eq!(me.get("email"), Some(&Value::Null));
}

#[tokio::test]
async fn devices_endpoints_without_bearer_are_401() {
    let (addr, _s, _e) = spawn().await;
    let http = reqwest::Client::new();
    assert_eq!(
        http.get(format!("{addr}/v1/devices"))
            .send()
            .await
            .unwrap()
            .status(),
        401
    );
    // A well-formed body (incl. the PoP fields) isolates the auth gate: it
    // deserializes past the `PopJson` extractor so the handler's bearer check runs and
    // 401s, rather than the extractor rejecting a missing field first.
    assert_eq!(
        http.post(format!("{addr}/v1/devices"))
            .json(&serde_json::json!({
                "public_key": URL_SAFE_NO_PAD.encode([0x33u8; 32]),
                "principal_kind": "human",
                "pop_challenge": "unused-no-session",
                "pop_signature": URL_SAFE_NO_PAD.encode([0u8; 64]),
            }))
            .send()
            .await
            .unwrap()
            .status(),
        401
    );
}

#[tokio::test]
async fn me_with_expired_session_is_401() {
    // session_ttl 0 → the session issued at poll time is already expired by the
    // time /v1/me runs (ms later).
    let config = ServiceConfig {
        session_ttl_ns: 0,
        ..test_config()
    };
    let (addr, _s, email) = spawn_with(config).await;
    let http = reqwest::Client::new();
    let (session, _refresh) = login(&http, &addr, &email, "expired@example.com").await;
    let resp = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), 401);
}

// ============================================================================
// F — refresh rotation single-use
// ============================================================================

#[tokio::test]
async fn old_refresh_token_is_dead_after_rotation() {
    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();
    let (_session, refresh1) = login(&http, &addr, &email, "rotate@example.com").await;

    // Rotate: refresh1 → a new session + refresh2.
    let rotated: Value = http
        .post(format!("{addr}/v1/auth/refresh"))
        .json(&serde_json::json!({ "refresh_token": refresh1 }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let refresh2 = rotated["refresh_token"].as_str().unwrap().to_string();

    // Replaying the ORIGINAL refresh token is now rejected (rotation is single-use).
    let replay = http
        .post(format!("{addr}/v1/auth/refresh"))
        .json(&serde_json::json!({ "refresh_token": refresh1 }))
        .send()
        .await
        .unwrap();
    assert_eq!(replay.status(), 401, "the old refresh token must be dead");

    // The NEW refresh token still works.
    let ok = http
        .post(format!("{addr}/v1/auth/refresh"))
        .json(&serde_json::json!({ "refresh_token": refresh2 }))
        .send()
        .await
        .unwrap();
    assert_eq!(ok.status(), 200);
}

#[tokio::test]
async fn revoke_then_refresh_is_rejected() {
    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();
    let (session, refresh) = login(&http, &addr, &email, "revoke@example.com").await;

    // Revoke the session server-side.
    let rev = http
        .post(format!("{addr}/v1/auth/revoke"))
        .json(&serde_json::json!({ "token": session }))
        .send()
        .await
        .unwrap();
    assert_eq!(rev.status(), 204);

    // A refresh with the paired refresh token is now rejected (shared revoked flag).
    let refreshed = http
        .post(format!("{addr}/v1/auth/refresh"))
        .json(&serde_json::json!({ "refresh_token": refresh }))
        .send()
        .await
        .unwrap();
    assert_eq!(refreshed.status(), 401);
}

#[tokio::test]
async fn concurrent_refreshes_with_one_token_yield_exactly_one_winner() {
    // Two concurrent refreshes carrying the SAME refresh token must NOT
    // both succeed (the second would silently invalidate the first). The rotate CAS
    // (WHERE refresh_token_hash = old) makes it single-use → exactly one 200.
    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();
    let (_session, refresh) = login(&http, &addr, &email, "concurrent-refresh@example.com").await;

    let do_refresh = |tok: String| {
        let http = http.clone();
        let addr = addr.clone();
        async move {
            http.post(format!("{addr}/v1/auth/refresh"))
                .json(&serde_json::json!({ "refresh_token": tok }))
                .send()
                .await
                .unwrap()
        }
    };
    let (r1, r2) = tokio::join!(do_refresh(refresh.clone()), do_refresh(refresh.clone()));

    let statuses = [r1.status().as_u16(), r2.status().as_u16()];
    assert_eq!(
        statuses.iter().filter(|&&s| s == 200).count(),
        1,
        "exactly one refresh must win, got {statuses:?}"
    );
    assert_eq!(
        statuses.iter().filter(|&&s| s == 401).count(),
        1,
        "the losing refresh must be Unauthorized"
    );
}

// ============================================================================
// B — concurrent device-poll redemption mints exactly one session
// ============================================================================

#[tokio::test]
async fn concurrent_polls_of_one_authorized_code_mint_exactly_one_session() {
    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();

    // Start + authorize a device code (do NOT poll yet).
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

    http.post(format!("{addr}/v1/auth/magic-link/start"))
        .json(&serde_json::json!({ "email": "race@example.com", "user_code": user_code }))
        .send()
        .await
        .unwrap();
    let token = token_from_link(&email.last_link().unwrap());
    http.get(format!(
        "{addr}/v1/auth/magic-link/complete?token={token}&user_code={user_code}"
    ))
    .send()
    .await
    .unwrap();

    // Fire TWO polls concurrently for the same authorized code.
    let poll = |dc: String| {
        let http = http.clone();
        let addr = addr.clone();
        async move {
            http.post(format!("{addr}/v1/auth/device/poll"))
                .json(&serde_json::json!({ "device_code": dc }))
                .send()
                .await
                .unwrap()
        }
    };
    let (r1, r2) = tokio::join!(poll(device_code.clone()), poll(device_code.clone()));

    // EXACTLY one winner (200 with a token); the other is 400 (AlreadyRedeemed).
    let statuses = [r1.status().as_u16(), r2.status().as_u16()];
    let ok_count = statuses.iter().filter(|&&s| s == 200).count();
    let denied_count = statuses.iter().filter(|&&s| s == 400).count();
    assert_eq!(
        ok_count, 1,
        "exactly one poll must win the redemption, got statuses {statuses:?}"
    );
    assert_eq!(denied_count, 1, "the losing poll must be denied");
}

// ============================================================================
// A — cross-account device registration + cert-from-stored-row (handler)
// ============================================================================

#[tokio::test]
async fn cross_account_registration_of_one_key_is_a_conflict() {
    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();

    let (session_a, _) = login(&http, &addr, &email, "owner-a@example.com").await;
    let (session_b, _) = login(&http, &addr, &email, "owner-b@example.com").await;

    // A real keypair (both accounts CAN prove possession of it in the test) so the 409
    // is reached AFTER the PoP passes — the PoP proves key ownership; the 409
    // enforces one-key-one-account ON TOP of it.
    let sk = SigningKey::from_bytes(&[0x33u8; 32]);
    let key = URL_SAFE_NO_PAD.encode(sk.verifying_key().to_bytes());

    // A registers the key with a valid PoP → OK.
    let (a_ch, a_sig) = device_pop(&http, &addr, &session_a, &sk).await;
    let reg_a = http
        .post(format!("{addr}/v1/devices"))
        .bearer_auth(&session_a)
        .json(&serde_json::json!({
            "public_key": key,
            "principal_kind": "human",
            "pop_challenge": a_ch,
            "pop_signature": a_sig,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(reg_a.status(), 200);

    // B registers the SAME key (with B's own valid PoP) → 409 Conflict (one key ↔ one
    // account), refused only after the PoP is accepted.
    let (b_ch, b_sig) = device_pop(&http, &addr, &session_b, &sk).await;
    let reg_b = http
        .post(format!("{addr}/v1/devices"))
        .bearer_auth(&session_b)
        .json(&serde_json::json!({
            "public_key": key,
            "principal_kind": "machine",
            "pop_challenge": b_ch,
            "pop_signature": b_sig,
        }))
        .send()
        .await
        .unwrap();
    assert_eq!(reg_b.status(), 409);
    assert_eq!(reg_b.json::<Value>().await.unwrap()["error"], "conflict");
}

#[tokio::test]
async fn reregistration_mints_the_cert_from_the_stored_principal() {
    use cerulion_pairing::format::SignedDeviceCert;

    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();
    let (session, _) = login(&http, &addr, &email, "stored@example.com").await;
    // A real keypair — the same key signs the PoP each time (a fresh single-use
    // challenge per call).
    let sk = SigningKey::from_bytes(&[0x44u8; 32]);
    let key = URL_SAFE_NO_PAD.encode(sk.verifying_key().to_bytes());

    // First registration: HUMAN.
    let (ch1, sig1) = device_pop(&http, &addr, &session, &sk).await;
    let first: Value = http
        .post(format!("{addr}/v1/devices"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "public_key": key,
            "principal_kind": "human",
            "pop_challenge": ch1,
            "pop_signature": sig1,
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let device_id_1 = first["device_id"].as_str().unwrap().to_string();

    // Re-register the SAME key claiming MACHINE (a fresh PoP challenge).
    let (ch2, sig2) = device_pop(&http, &addr, &session, &sk).await;
    let second: Value = http
        .post(format!("{addr}/v1/devices"))
        .bearer_auth(&session)
        .json(&serde_json::json!({
            "public_key": key,
            "principal_kind": "machine",
            "pop_challenge": ch2,
            "pop_signature": sig2,
        }))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    // Same device row.
    assert_eq!(second["device_id"].as_str().unwrap(), device_id_1);
    // The cert's principal_kind is the STORED one (Human), NOT the request (Machine).
    let cert: SignedDeviceCert =
        cerulion_accountd::decode_b64(second["device_cert"].as_str().unwrap()).unwrap();
    assert_eq!(cert.cert.principal_kind, PrincipalKind::Human);
}

// ============================================================================
// G — magic-link completion is single-use (e2e)
// ============================================================================

#[tokio::test]
async fn magic_link_completion_is_single_use_e2e() {
    let (addr, _s, email) = spawn().await;
    let http = reqwest::Client::new();

    let start: Value = http
        .post(format!("{addr}/v1/auth/device/start"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let user_code = start["user_code"].as_str().unwrap().to_string();

    http.post(format!("{addr}/v1/auth/magic-link/start"))
        .json(&serde_json::json!({ "email": "single@example.com", "user_code": user_code }))
        .send()
        .await
        .unwrap();
    let token = token_from_link(&email.last_link().unwrap());
    let url = format!("{addr}/v1/auth/magic-link/complete?token={token}&user_code={user_code}");

    // First completion succeeds; the second is rejected (single-use token).
    assert_eq!(http.get(&url).send().await.unwrap().status(), 200);
    assert_eq!(http.get(&url).send().await.unwrap().status(), 400);
}
