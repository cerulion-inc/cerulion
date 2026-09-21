// SPDX-License-Identifier: AGPL-3.0-only
//! OAuth provider tests.
//!
//! - Pure: Google/GitHub authorization-URL construction, PKCE-S256 challenge
//!   derivation, and provider→`Identity` mapping (hand oracles).
//! - HTTP: the callback path authorizes a device code via an injected identity
//!   resolver (the external IdP boundary stood in with a real in-test resolver —
//!   the code path is genuine, only the third-party network hop is stubbed).
//! - Loud-refusal: an unconfigured provider's start endpoint refuses; the
//!   default resolver refuses the callback (never a silent bypass, by design).

use std::sync::Arc;

use sha2::{Digest, Sha256};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use serde_json::Value;

use cerulion_accountd::{
    pkce_challenge, AppState, Clock, GitHubProvider, GoogleProvider, Identity, IdentityResolver,
    LoggingEmailSender, OAuthConfig, Provider, ProviderCreds, ServiceConfig, UnconfiguredResolver,
};

fn creds() -> ProviderCreds {
    ProviderCreds {
        client_id: "test-client-id".to_string(),
        client_secret: "test-secret".to_string(),
        redirect_uri: "https://accounts.test/v1/auth/oauth/google/callback".to_string(),
    }
}

// ============================================================================
// pure
// ============================================================================

#[test]
fn google_authorize_url_carries_pkce_and_openid_scope() {
    let url = GoogleProvider.authorize_url(&creds(), "state-abc", "challenge-xyz");
    let parsed = url::Url::parse(&url).unwrap();
    assert_eq!(parsed.host_str(), Some("accounts.google.com"));
    let q: std::collections::HashMap<_, _> = parsed.query_pairs().into_owned().collect();
    assert_eq!(q["client_id"], "test-client-id");
    assert_eq!(q["response_type"], "code");
    assert_eq!(q["scope"], "openid email");
    assert_eq!(q["state"], "state-abc");
    assert_eq!(q["code_challenge"], "challenge-xyz");
    assert_eq!(q["code_challenge_method"], "S256");
    assert_eq!(
        q["redirect_uri"],
        "https://accounts.test/v1/auth/oauth/google/callback"
    );
}

#[test]
fn github_authorize_url_carries_state_and_scopes() {
    let url = GitHubProvider.authorize_url(&creds(), "state-def", "unused-challenge");
    let parsed = url::Url::parse(&url).unwrap();
    assert_eq!(parsed.host_str(), Some("github.com"));
    let q: std::collections::HashMap<_, _> = parsed.query_pairs().into_owned().collect();
    assert_eq!(q["client_id"], "test-client-id");
    assert_eq!(q["scope"], "read:user user:email");
    assert_eq!(q["state"], "state-def");
    // GitHub's classic flow carries no PKCE challenge.
    assert!(!q.contains_key("code_challenge"));
}

#[test]
fn pkce_challenge_matches_the_s256_oracle() {
    let verifier = "a-known-pkce-verifier-string-value";
    let oracle = URL_SAFE_NO_PAD.encode(Sha256::digest(verifier.as_bytes()));
    assert_eq!(pkce_challenge(verifier), oracle);
}

#[test]
fn google_map_identity_reads_sub_and_email() {
    let userinfo = serde_json::json!({ "sub": "google-subject-123", "email": "u@example.com" });
    let id = GoogleProvider.map_identity(&userinfo).unwrap();
    assert_eq!(
        id,
        Identity {
            provider: "google".to_string(),
            subject: "google-subject-123".to_string(),
            email: Some("u@example.com".to_string()),
        }
    );
}

#[test]
fn github_map_identity_stringifies_numeric_id_and_tolerates_null_email() {
    let userinfo = serde_json::json!({ "id": 424242, "login": "octocat", "email": null });
    let id = GitHubProvider.map_identity(&userinfo).unwrap();
    assert_eq!(id.provider, "github");
    assert_eq!(id.subject, "424242");
    assert_eq!(id.email, None);
}

#[test]
fn google_map_identity_missing_sub_is_an_error() {
    let userinfo = serde_json::json!({ "email": "u@example.com" });
    assert!(GoogleProvider.map_identity(&userinfo).is_err());
}

// ============================================================================
// HTTP: callback authorizes a device code (injected resolver)
// ============================================================================

/// A test resolver standing in for the external IdP's token exchange. Returns a
/// canned identity — a real impl of the seam, not fake account data (the account
/// is minted by the genuine `upsert_user_by_identity` path).
struct StaticResolver {
    identity: Identity,
}

impl IdentityResolver for StaticResolver {
    fn resolve(
        &self,
        _provider: &str,
        _code: &str,
        _pkce_verifier: &str,
    ) -> cerulion_accountd::Result<Identity> {
        Ok(self.identity.clone())
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

async fn spawn(
    config: ServiceConfig,
    resolver: Arc<dyn IdentityResolver>,
) -> (String, Arc<AppState>) {
    ensure_crypto_provider();
    let state = Arc::new(
        AppState::dev(
            config,
            Clock::System,
            Arc::new(LoggingEmailSender),
            resolver,
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
    (addr, state)
}

fn google_configured() -> ServiceConfig {
    ServiceConfig {
        device_code_interval_secs: 0,
        oauth: OAuthConfig {
            google: Some(creds()),
            ..Default::default()
        },
        ..Default::default()
    }
}

#[tokio::test]
async fn oauth_callback_authorizes_the_device_code_and_mints_a_session() {
    let resolver = Arc::new(StaticResolver {
        identity: Identity {
            provider: "google".to_string(),
            subject: "google-sub-777".to_string(),
            email: Some("teammate@example.com".to_string()),
        },
    });
    let (addr, _state) = spawn(google_configured(), resolver).await;
    let http = reqwest::Client::new();

    // Start a device code.
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

    // Begin the OAuth flow carrying the device-flow user_code.
    let ostart: Value = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/start?user_code={user_code}"
        ))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let oauth_state = ostart["state"].as_str().unwrap().to_string();
    assert!(ostart["authorize_url"]
        .as_str()
        .unwrap()
        .contains("accounts.google.com"));

    // The provider redirects back to the callback with a code + our state.
    let callback = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/callback?code=fake-auth-code&state={oauth_state}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(callback.status(), 200);
    assert_eq!(
        callback.json::<Value>().await.unwrap()["status"],
        "authorized"
    );

    // The device code is now redeemable.
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

    // The minted session belongs to the resolver's identity's account.
    let me = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(&session)
        .send()
        .await
        .unwrap();
    assert_eq!(me.status(), 200);
}

#[tokio::test]
async fn unconfigured_provider_start_refuses_loudly() {
    // github is NOT configured (only google is) → its start endpoint refuses.
    let (addr, _state) = spawn(google_configured(), Arc::new(UnconfiguredResolver)).await;
    let http = reqwest::Client::new();
    let resp = http
        .get(format!("{addr}/v1/auth/oauth/github/start"))
        .send()
        .await
        .unwrap();
    assert_eq!(resp.status(), 501);
    assert_eq!(
        resp.json::<Value>().await.unwrap()["error"],
        "provider_not_configured"
    );
}

#[tokio::test]
async fn default_resolver_refuses_the_callback_no_silent_bypass() {
    // google configured but the default resolver is installed → the callback
    // refuses loudly rather than fabricating an identity.
    let (addr, _state) = spawn(google_configured(), Arc::new(UnconfiguredResolver)).await;
    let http = reqwest::Client::new();

    let ostart: Value = http
        .get(format!("{addr}/v1/auth/oauth/google/start"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let oauth_state = ostart["state"].as_str().unwrap().to_string();

    let callback = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/callback?code=fake&state={oauth_state}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(callback.status(), 501);
    assert_eq!(
        callback.json::<Value>().await.unwrap()["error"],
        "identity_resolver_unavailable"
    );
}

// ============================================================================
// A failed exchange leaves the flow retryable; success consumes it
// ============================================================================

#[tokio::test]
async fn a_failed_exchange_leaves_the_oauth_flow_retryable() {
    // google configured but the default resolver refuses → the exchange FAILS. The
    // flow row must NOT be consumed, so the login is retryable (an atomic
    // DELETE...RETURNING here would consume it on failure too).
    let (addr, _state) = spawn(google_configured(), Arc::new(UnconfiguredResolver)).await;
    let http = reqwest::Client::new();

    let ostart: Value = http
        .get(format!("{addr}/v1/auth/oauth/google/start"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let oauth_state = ostart["state"].as_str().unwrap().to_string();

    // First callback: the exchange refuses → 501, flow row NOT consumed.
    let r1 = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/callback?code=c1&state={oauth_state}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(r1.status(), 501);

    // Retry with the SAME state: the row survived, so the exchange is attempted
    // AGAIN → 501 `identity_resolver_unavailable` (a CONSUMED row would instead give
    // 400 `invalid_request` "unknown or expired oauth state").
    let r2 = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/callback?code=c2&state={oauth_state}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(
        r2.status(),
        501,
        "a failed exchange must leave the flow retryable"
    );
    assert_eq!(
        r2.json::<Value>().await.unwrap()["error"],
        "identity_resolver_unavailable"
    );
}

#[tokio::test]
async fn a_successful_exchange_consumes_the_oauth_flow_single_use() {
    let resolver = Arc::new(StaticResolver {
        identity: Identity {
            provider: "google".to_string(),
            subject: "sub-consume".to_string(),
            email: Some("consume@example.com".to_string()),
        },
    });
    let (addr, _state) = spawn(google_configured(), resolver).await;
    let http = reqwest::Client::new();

    let ostart: Value = http
        .get(format!("{addr}/v1/auth/oauth/google/start"))
        .send()
        .await
        .unwrap()
        .json()
        .await
        .unwrap();
    let oauth_state = ostart["state"].as_str().unwrap().to_string();

    // First callback succeeds (200) → consumes the flow.
    let r1 = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/callback?code=ok&state={oauth_state}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(r1.status(), 200);

    // Replaying the same state → 400 (consumed; single-use).
    let r2 = http
        .get(format!(
            "{addr}/v1/auth/oauth/google/callback?code=ok&state={oauth_state}"
        ))
        .send()
        .await
        .unwrap();
    assert_eq!(r2.status(), 400);
}
