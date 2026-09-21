// SPDX-License-Identifier: AGPL-3.0-only
//! Supabase Auth JWT exchange coverage.

use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use jsonwebtoken::jwk::JwkSet;
use jsonwebtoken::{encode, Algorithm, EncodingKey, Header};
use serde_json::Value;

use cerulion_accountd::Result;
use cerulion_accountd::{
    AppState, Clock, JwksSource, ServiceConfig, StaticJwksSource, SupabaseConfig, SupabaseVerifier,
    UnconfiguredResolver, SEC_NS,
};

const ISSUER: &str = "https://example.supabase.co/auth/v1";
const AUDIENCE: &str = "authenticated";
const KID: &str = "test-key";
const PRIVATE_KEY_DER_B64: &str =
    "MIGHAgEAMBMGByqGSM49AgEGCCqGSM49AwEHBG0wawIBAQQgWTFfCGljY6aw3HrtkHmPRiazukxPLb6ilpRAewjW8nihRANCAATDskChT+Altkm9X7MI69T3IUmrQU0L950IxEzvw/x5BMEINRMrXLBJhqzO9Bm+d6JbqA21YQmd1Kt4RzLJR1W+";
const PUBLIC_X: &str = "w7JAoU_gJbZJvV-zCOvU9yFJq0FNC_edCMRM78P8eQQ";
const PUBLIC_Y: &str = "wQg1EytcsEmGrM70Gb53oluoDbVhCZ3Uq3hHMslHVb4";

#[derive(Clone)]
struct CountingSource {
    jwks: JwkSet,
    calls: Arc<AtomicUsize>,
}

#[derive(Clone)]
struct FailingSource {
    calls: Arc<AtomicUsize>,
}

#[derive(Clone)]
struct FailAfterFirstSource {
    jwks: JwkSet,
    calls: Arc<AtomicUsize>,
}

#[derive(Clone)]
struct AdvancingSource {
    jwks: JwkSet,
    clock_ns: Arc<AtomicU64>,
    next_ns: u64,
}

impl JwksSource for FailingSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        self.calls.fetch_add(1, Ordering::SeqCst);
        Box::pin(async {
            Err(cerulion_accountd::AccountdError::Internal(
                "fixture JWKS failure".into(),
            ))
        })
    }
}

impl JwksSource for FailAfterFirstSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        let call = self.calls.fetch_add(1, Ordering::SeqCst);
        let result = (call == 0).then(|| self.jwks.clone()).ok_or_else(|| {
            cerulion_accountd::AccountdError::Internal("fixture JWKS failure".into())
        });
        Box::pin(async move { result })
    }
}

impl JwksSource for CountingSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        self.calls.fetch_add(1, Ordering::SeqCst);
        let jwks = self.jwks.clone();
        Box::pin(async move { Ok(jwks) })
    }
}

impl JwksSource for AdvancingSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        let clock_ns = self.clock_ns.clone();
        let next_ns = self.next_ns;
        let jwks = self.jwks.clone();
        Box::pin(async move {
            clock_ns.store(next_ns, Ordering::SeqCst);
            tokio::task::yield_now().await;
            Ok(jwks)
        })
    }
}

#[derive(Clone)]
struct SequenceSource {
    sets: Arc<Vec<Option<JwkSet>>>,
    calls: Arc<AtomicUsize>,
}

impl JwksSource for SequenceSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        let index = self.calls.fetch_add(1, Ordering::SeqCst);
        let jwks = self
            .sets
            .get(index)
            .or_else(|| self.sets.last())
            .expect("sequence has a JWKS")
            .clone();
        Box::pin(async move {
            jwks.ok_or_else(|| {
                cerulion_accountd::AccountdError::Internal("fixture JWKS failure".into())
            })
        })
    }
}

async fn scripted_verification_sequence(source: Arc<SequenceSource>) -> Vec<(bool, usize)> {
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source.clone(), monotonic_ns.clone());
    let known = token(Algorithm::ES256, "known-sub", ISSUER, AUDIENCE, 20_000);
    let unknown = token_with_kid(
        Algorithm::ES256,
        "unknown-sub",
        ISSUER,
        AUDIENCE,
        20_000,
        Some("authenticated"),
        Some("unknown-kid"),
    );
    let mut outcomes = Vec::new();
    for (jwt, now_ns) in [
        (&known, 0),
        (&unknown, 30 * SEC_NS),
        (&known, 61 * SEC_NS),
        (&known, 3_600 * SEC_NS),
        (&known, 3_601 * SEC_NS),
    ] {
        monotonic_ns.store(now_ns, Ordering::SeqCst);
        outcomes.push((
            verifier.verify(jwt, now_ns).await.is_ok(),
            source.calls.load(Ordering::SeqCst),
        ));
    }
    outcomes
}

fn jwks_for(kid: &str, alg: Option<&str>) -> JwkSet {
    let mut key = serde_json::json!({
        "kty": "EC",
        "crv": "P-256",
        "x": PUBLIC_X,
        "y": PUBLIC_Y,
        "kid": kid,
        "use": "sig"
    });
    if let Some(alg) = alg {
        key["alg"] = Value::String(alg.to_string());
    }
    serde_json::from_value(serde_json::json!({
        "keys": [key]
    }))
    .expect("valid JWK fixture")
}

fn jwks() -> JwkSet {
    jwks_for(KID, Some("ES256"))
}

fn oct_jwks() -> JwkSet {
    serde_json::from_value(serde_json::json!({
        "keys": [{
            "kty": "oct",
            "k": "bGVnYWN5LXNlY3JldA",
            "alg": "HS256",
            "kid": KID,
            "use": "sig"
        }]
    }))
    .expect("valid octet JWK fixture")
}

fn config(secret: Option<&str>) -> ServiceConfig {
    ServiceConfig {
        supabase: Some(SupabaseConfig {
            issuer: ISSUER.to_string(),
            audience: AUDIENCE.to_string(),
            jwks_url: Some(format!("{ISSUER}/.well-known/jwks.json")),
            hs256_secret: secret.map(str::to_string),
        }),
        device_code_interval_secs: 0,
        ..ServiceConfig::default()
    }
}

fn verifier_with_monotonic(
    source: Arc<dyn JwksSource>,
    monotonic_ns: Arc<AtomicU64>,
) -> SupabaseVerifier {
    SupabaseVerifier::with_monotonic(
        config(None).supabase.expect("configured Supabase"),
        source,
        Arc::new(move || monotonic_ns.load(Ordering::SeqCst)),
    )
}

/// Install the ring provider for test binaries when Cargo unifies reqwest with
/// its `rustls-no-provider` feature through another workspace crate.
fn ensure_crypto_provider() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let _ = rustls::crypto::ring::default_provider().install_default();
    });
}

fn token_with_kid(
    algorithm: Algorithm,
    subject: &str,
    issuer: &str,
    audience: &str,
    exp: u64,
    role: Option<&str>,
    kid: Option<&str>,
) -> String {
    let mut header = Header::new(algorithm);
    header.kid = kid.map(str::to_string);
    let claims = serde_json::json!({
        "sub": subject,
        "email": "person@example.com",
        "exp": exp,
        "iat": exp.saturating_sub(60),
        "iss": issuer,
        "aud": audience,
        "role": role
    });
    let key = if algorithm == Algorithm::HS256 {
        EncodingKey::from_secret(b"legacy-secret")
    } else {
        let der = base64::engine::general_purpose::STANDARD
            .decode(PRIVATE_KEY_DER_B64)
            .expect("private key fixture");
        EncodingKey::from_ec_der(&der)
    };
    encode(&header, &claims, &key).expect("signed fixture token")
}

fn token_with_anonymous(subject: &str, is_anonymous: bool) -> String {
    let mut header = Header::new(Algorithm::ES256);
    header.kid = Some(KID.to_string());
    let claims = serde_json::json!({
        "sub": subject,
        "email": "person@example.com",
        "exp": 20_000,
        "iat": 19_940,
        "iss": ISSUER,
        "aud": AUDIENCE,
        "role": "authenticated",
        "is_anonymous": is_anonymous
    });
    let der = base64::engine::general_purpose::STANDARD
        .decode(PRIVATE_KEY_DER_B64)
        .expect("private key fixture");
    encode(&header, &claims, &EncodingKey::from_ec_der(&der)).expect("signed fixture token")
}

fn token_with_role(
    algorithm: Algorithm,
    subject: &str,
    issuer: &str,
    audience: &str,
    exp: u64,
    role: Option<&str>,
) -> String {
    token_with_kid(
        algorithm,
        subject,
        issuer,
        audience,
        exp,
        role,
        (algorithm == Algorithm::ES256).then_some(KID),
    )
}

fn token(algorithm: Algorithm, subject: &str, issuer: &str, audience: &str, exp: u64) -> String {
    token_with_role(
        algorithm,
        subject,
        issuer,
        audience,
        exp,
        Some("authenticated"),
    )
}

fn hs256_token(subject: &str, secret: &[u8]) -> String {
    let header = Header::new(Algorithm::HS256);
    let claims = serde_json::json!({
        "sub": subject,
        "email": "person@example.com",
        "exp": 20_000,
        "iat": 19_940,
        "iss": ISSUER,
        "aud": AUDIENCE,
        "role": "authenticated"
    });
    encode(&header, &claims, &EncodingKey::from_secret(secret)).expect("signed HS256 fixture token")
}

async fn spawn(
    config: ServiceConfig,
    clock: Clock,
    source: Arc<dyn JwksSource>,
) -> (String, Arc<AppState>) {
    ensure_crypto_provider();
    let verifier_config = config.supabase.clone();
    let email = Arc::new(cerulion_accountd::CapturingEmailSender::new());
    let mut app =
        AppState::dev(config, clock, email, Arc::new(UnconfiguredResolver)).expect("dev app state");
    if let Some(verifier_config) = verifier_config {
        app.supabase_verifier = Some(Arc::new(SupabaseVerifier::new(verifier_config, source)));
    }
    let state = Arc::new(app);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
        .await
        .expect("bind test server");
    let addr = format!("http://{}", listener.local_addr().expect("local address"));
    let serve_state = state.clone();
    tokio::spawn(async move {
        cerulion_accountd::serve(listener, serve_state)
            .await
            .expect("serve test server");
    });
    (addr, state)
}

async fn device_exchange_status_after_jwks_advance(
    fetch_now_ns: u64,
    device_code_ttl_ns: u64,
) -> (reqwest::StatusCode, bool) {
    let clock_ns = Arc::new(AtomicU64::new(100 * SEC_NS));
    let (addr, state) = spawn(
        ServiceConfig {
            device_code_ttl_ns,
            device_code_interval_secs: 0,
            ..config(None)
        },
        Clock::Shared(clock_ns.clone()),
        Arc::new(AdvancingSource {
            jwks: jwks(),
            clock_ns,
            next_ns: fetch_now_ns,
        }),
    )
    .await;
    let http = reqwest::Client::new();
    let start: Value = http
        .post(format!("{addr}/v1/auth/device/start"))
        .send()
        .await
        .expect("device start")
        .json()
        .await
        .expect("device json");
    let response = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token(Algorithm::ES256, "slow-device-sub", ISSUER, AUDIENCE, 200),
            "user_code": start["user_code"]
        }))
        .send()
        .await
        .expect("exchange");
    let status = response.status();
    let body = response.text().await.expect("exchange body");
    if status == reqwest::StatusCode::BAD_REQUEST {
        assert!(body.contains("unknown or expired user_code"), "{body}");
    }
    let user_created = state
        .db
        .user_by_identity("supabase", "slow-device-sub")
        .expect("identity lookup")
        .is_some();
    (status, user_created)
}

#[tokio::test]
async fn slow_jwks_fetch_after_device_code_expiry_is_rejected_deterministically() {
    let start_ns = 100 * SEC_NS;
    let ttl_ns = 10 * SEC_NS;
    let outcomes = [
        device_exchange_status_after_jwks_advance(start_ns + ttl_ns + 1, ttl_ns).await,
        device_exchange_status_after_jwks_advance(start_ns + ttl_ns + 1, ttl_ns).await,
    ];

    assert_eq!(outcomes[0], outcomes[1]);
    assert_eq!(outcomes[0].0, reqwest::StatusCode::BAD_REQUEST);
    assert!(!outcomes[0].1);
}

#[tokio::test]
async fn slow_jwks_fetch_before_device_code_expiry_is_authorized() {
    let start_ns = 100 * SEC_NS;
    let ttl_ns = 10 * SEC_NS;
    let outcomes = [
        device_exchange_status_after_jwks_advance(start_ns + ttl_ns - 1, ttl_ns).await,
        device_exchange_status_after_jwks_advance(start_ns + ttl_ns - 1, ttl_ns).await,
    ];

    assert_eq!(outcomes[0], outcomes[1]);
    assert_eq!(outcomes[0].0, reqwest::StatusCode::OK);
    assert!(outcomes[0].1);
}

#[tokio::test]
async fn valid_es256_exchange_issues_session_and_reuses_identity() {
    let (addr, state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let http = reqwest::Client::new();
    let jwt = token(Algorithm::ES256, "supabase-sub", ISSUER, AUDIENCE, 200);

    let first: Value = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({ "access_token": jwt }))
        .send()
        .await
        .expect("exchange")
        .json()
        .await
        .expect("exchange json");
    assert_eq!(first["token_type"], "Bearer");
    let first_me: Value = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(first["session_token"].as_str().expect("session token"))
        .send()
        .await
        .expect("me")
        .json()
        .await
        .expect("me json");
    assert_eq!(first_me["email"], "person@example.com");
    let identity = state
        .db
        .user_by_identity("supabase", "supabase-sub")
        .expect("identity lookup")
        .expect("persisted Supabase identity");
    assert_eq!(identity.provider, "supabase");
    assert_eq!(identity.subject, "supabase-sub");

    let second: Value = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token(Algorithm::ES256, "supabase-sub", ISSUER, AUDIENCE, 200)
        }))
        .send()
        .await
        .expect("exchange")
        .json()
        .await
        .expect("exchange json");
    let second_me: Value = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(second["session_token"].as_str().expect("session token"))
        .send()
        .await
        .expect("me")
        .json()
        .await
        .expect("me json");
    assert_eq!(first_me["user_id"], second_me["user_id"]);
    assert_eq!(first_me["principal_kind"], "human");
}

#[tokio::test]
async fn expired_wrong_issuer_and_wrong_audience_tokens_are_unauthorized() {
    let (addr, _state) = spawn(
        config(None),
        Clock::Fixed(10_000 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let http = reqwest::Client::new();
    for jwt in [
        token(Algorithm::ES256, "expired", ISSUER, AUDIENCE, 1),
        token(
            Algorithm::ES256,
            "wrong-issuer",
            "https://other.example/auth/v1",
            AUDIENCE,
            20_000,
        ),
        token(
            Algorithm::ES256,
            "wrong-audience",
            ISSUER,
            "other-audience",
            20_000,
        ),
    ] {
        let response = http
            .post(format!("{addr}/v1/auth/supabase/exchange"))
            .json(&serde_json::json!({ "access_token": jwt }))
            .send()
            .await
            .expect("exchange");
        assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
    }
}

#[tokio::test]
async fn anonymous_supabase_tokens_are_unauthorized() {
    let verifier = SupabaseVerifier::new(
        config(None).supabase.expect("configured Supabase"),
        Arc::new(CountingSource {
            jwks: jwks(),
            calls: Arc::new(AtomicUsize::new(0)),
        }),
    );
    let anonymous = token_with_anonymous("anonymous-sub", true);
    let explicit_non_anonymous = token_with_anonymous("real-sub", false);

    assert!(verifier.verify(&anonymous, 100 * SEC_NS).await.is_err());
    assert!(verifier
        .verify(&explicit_non_anonymous, 100 * SEC_NS)
        .await
        .is_ok());
}

#[tokio::test]
async fn cold_unknown_kid_fetches_once_and_does_not_leak_claims() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(CountingSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let (addr, _state) = spawn(config(None), Clock::Fixed(100 * SEC_NS), source).await;
    let http = reqwest::Client::new();
    let jwt = token_with_kid(
        Algorithm::ES256,
        "secret-subject",
        ISSUER,
        AUDIENCE,
        200,
        Some("authenticated"),
        Some("unknown-kid"),
    );
    let response = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({ "access_token": jwt }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
    let body = response.text().await.expect("error body");
    assert!(!body.contains("secret-subject"));
    assert!(!body.contains("person@example.com"));
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    let response = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({ "access_token": jwt }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
    assert_eq!(calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn warm_unknown_kid_refetches_after_sixty_seconds() {
    const NEW_KID: &str = "rotated-key";
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(SequenceSource {
        sets: Arc::new(vec![Some(jwks()), Some(jwks_for(NEW_KID, None))]),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let unknown = token_with_kid(
        Algorithm::ES256,
        "unknown-sub",
        ISSUER,
        AUDIENCE,
        200,
        Some("authenticated"),
        Some(NEW_KID),
    );
    monotonic_ns.store(100 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&unknown, 100 * SEC_NS).await.is_err());
    monotonic_ns.store(159 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&unknown, 159 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 1);

    let rotated = token_with_kid(
        Algorithm::ES256,
        "rotated-sub",
        ISSUER,
        AUDIENCE,
        300,
        Some("authenticated"),
        Some(NEW_KID),
    );
    monotonic_ns.store(161 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&rotated, 161 * SEC_NS).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
}

#[tokio::test]
async fn removed_key_is_rejected_after_the_cache_max_age() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(SequenceSource {
        sets: Arc::new(vec![Some(jwks()), Some(jwks_for("other", None))]),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(Algorithm::ES256, "removed-sub", ISSUER, AUDIENCE, 20_000);

    monotonic_ns.store(100 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 100 * SEC_NS).await.is_ok());
    monotonic_ns.store((100 + 3_599) * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, (100 + 3_599) * SEC_NS).await.is_ok());
    monotonic_ns.store((100 + 3_600) * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, (100 + 3_600) * SEC_NS).await.is_err());
    monotonic_ns.store((100 + 3_601) * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, (100 + 3_601) * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
}

#[tokio::test]
async fn known_key_is_served_from_cache_within_max_age() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(CountingSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(Algorithm::ES256, "cached-sub", ISSUER, AUDIENCE, 20_000);
    for seconds in [
        0, 1, 60, 120, 300, 600, 900, 1_200, 1_500, 1_800, 2_100, 2_400, 2_700, 3_000, 3_100,
        3_200, 3_300, 3_400, 3_500, 3_599,
    ] {
        monotonic_ns.store(seconds * SEC_NS, Ordering::SeqCst);
        assert!(verifier.verify(&jwt, seconds * SEC_NS).await.is_ok());
    }
    assert_eq!(calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn rotated_key_survives_refresh() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(SequenceSource {
        sets: Arc::new(vec![Some(jwks()), Some(jwks())]),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(Algorithm::ES256, "rotated-sub", ISSUER, AUDIENCE, 20_000);

    monotonic_ns.store(0, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 0).await.is_ok());
    monotonic_ns.store(3_600 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 3_600 * SEC_NS).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
}

#[tokio::test]
async fn wall_clock_rollback_does_not_affect_cached_keys() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(CountingSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(Algorithm::ES256, "rollback-sub", ISSUER, AUDIENCE, 20_000);

    monotonic_ns.store(0, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 1_000 * SEC_NS).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    assert!(verifier.verify(&jwt, 999 * SEC_NS).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn wall_clock_forward_jump_does_not_evict_cached_keys() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(CountingSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(
        Algorithm::ES256,
        "forward-jump-sub",
        ISSUER,
        AUDIENCE,
        20_000,
    );

    monotonic_ns.store(0, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 1_000 * SEC_NS).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    monotonic_ns.store(30 * SEC_NS, Ordering::SeqCst);
    assert!(verifier
        .verify(&jwt, (1_000 + 3_600) * SEC_NS)
        .await
        .is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn wall_clock_rollback_does_not_bypass_refetch_throttle() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(CountingSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns);
    let unknown = token_with_kid(
        Algorithm::ES256,
        "rollback-throttle-sub",
        ISSUER,
        AUDIENCE,
        20_000,
        Some("authenticated"),
        Some("unknown-kid"),
    );

    assert!(verifier.verify(&unknown, 100 * SEC_NS).await.is_err());
    assert!(verifier.verify(&unknown, 99 * SEC_NS).await.is_err());
    assert!(verifier.verify(&unknown, 98 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
}

#[tokio::test]
async fn monotonic_advance_allows_refetch_after_wall_clock_rollback() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(CountingSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let unknown = token_with_kid(
        Algorithm::ES256,
        "rollback-throttle-retry-sub",
        ISSUER,
        AUDIENCE,
        20_000,
        Some("authenticated"),
        Some("unknown-kid"),
    );

    assert!(verifier.verify(&unknown, 100 * SEC_NS).await.is_err());
    monotonic_ns.store(60 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&unknown, 99 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
}

#[tokio::test]
async fn verification_sequence_is_deterministic() {
    let source_one = Arc::new(SequenceSource {
        sets: Arc::new(vec![Some(jwks()), Some(jwks_for("other", None))]),
        calls: Arc::new(AtomicUsize::new(0)),
    });
    let source_two = Arc::new(SequenceSource {
        sets: Arc::new(vec![Some(jwks()), Some(jwks_for("other", None))]),
        calls: Arc::new(AtomicUsize::new(0)),
    });
    let first = scripted_verification_sequence(source_one).await;
    let second = scripted_verification_sequence(source_two).await;
    let expected = vec![(true, 1), (false, 1), (true, 1), (false, 2), (false, 2)];

    assert_eq!(first, second);
    assert_eq!(first, expected);
}

#[tokio::test]
async fn failed_revalidation_does_not_renew_stale_key_trust() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(FailAfterFirstSource {
        jwks: jwks(),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(Algorithm::ES256, "stale-sub", ISSUER, AUDIENCE, 20_000);

    monotonic_ns.store(0, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 0).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    monotonic_ns.store(3_600 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 3_600 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    monotonic_ns.store(3_630 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 3_630 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    monotonic_ns.store(3_661 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 3_661 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 3);
}

#[tokio::test]
async fn successful_revalidation_after_failure_restores_trust() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(SequenceSource {
        sets: Arc::new(vec![Some(jwks()), None, Some(jwks())]),
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token(Algorithm::ES256, "restored-sub", ISSUER, AUDIENCE, 20_000);

    monotonic_ns.store(0, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 0).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    monotonic_ns.store(3_600 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 3_600 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    monotonic_ns.store(3_661 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 3_661 * SEC_NS).await.is_ok());
    assert_eq!(calls.load(Ordering::SeqCst), 3);
}

#[tokio::test]
async fn failed_jwks_fetch_is_rate_limited() {
    let calls = Arc::new(AtomicUsize::new(0));
    let source = Arc::new(FailingSource {
        calls: calls.clone(),
    });
    let monotonic_ns = Arc::new(AtomicU64::new(0));
    let verifier = verifier_with_monotonic(source, monotonic_ns.clone());
    let jwt = token_with_kid(
        Algorithm::ES256,
        "failed-fetch-sub",
        ISSUER,
        AUDIENCE,
        200,
        Some("authenticated"),
        Some("missing-key"),
    );
    monotonic_ns.store(0, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 100 * SEC_NS).await.is_err());
    monotonic_ns.store(59 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 159 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    monotonic_ns.store(61 * SEC_NS, Ordering::SeqCst);
    assert!(verifier.verify(&jwt, 161 * SEC_NS).await.is_err());
    assert_eq!(calls.load(Ordering::SeqCst), 2);
}

#[tokio::test]
async fn jwks_without_alg_infers_es256_and_oct_keys_are_rejected() {
    let (addr, _state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks_for(KID, None))),
    )
    .await;
    let response = reqwest::Client::new()
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token(Algorithm::ES256, "inferred-sub", ISSUER, AUDIENCE, 200)
        }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::OK);

    let (addr, _state) = spawn(
        config(Some("not-the-oct-jwk-secret")),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(oct_jwks())),
    )
    .await;
    let response = reqwest::Client::new()
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token_with_kid(
                Algorithm::HS256,
                "oct-sub",
                ISSUER,
                AUDIENCE,
                200,
                Some("authenticated"),
                Some(KID)
            )
        }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn unsupported_and_tampered_tokens_are_unauthorized() {
    let (addr, _state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let http = reqwest::Client::new();
    let none = format!(
        "{}.{}.",
        URL_SAFE_NO_PAD.encode(r#"{"alg":"none","typ":"JWT"}"#),
        URL_SAFE_NO_PAD.encode(r#"{"sub":"none","exp":200,"iat":100,"iss":"https://example.supabase.co/auth/v1","aud":"authenticated"}"#)
    );
    let mut tampered = token(Algorithm::ES256, "tampered", ISSUER, AUDIENCE, 200);
    tampered.push('x');
    for jwt in [none, tampered] {
        let response = http
            .post(format!("{addr}/v1/auth/supabase/exchange"))
            .json(&serde_json::json!({ "access_token": jwt }))
            .send()
            .await
            .expect("exchange");
        assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
    }
}

#[tokio::test]
async fn non_authenticated_role_is_unauthorized() {
    let (addr, _state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let http = reqwest::Client::new();
    for role in [Some("service_role"), None] {
        let response = http
            .post(format!("{addr}/v1/auth/supabase/exchange"))
            .json(&serde_json::json!({
                "access_token": token_with_role(
                    Algorithm::ES256,
                    "role-sub",
                    ISSUER,
                    AUDIENCE,
                    200,
                    role
                )
            }))
            .send()
            .await
            .expect("exchange");
        assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
    }
}

#[tokio::test]
async fn hs256_requires_configured_secret_and_succeeds_when_configured() {
    let (addr, _state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let http = reqwest::Client::new();
    let jwt = token(Algorithm::HS256, "hs-sub", ISSUER, AUDIENCE, 200);
    let response = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({ "access_token": jwt }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);

    let (addr, _state) = spawn(
        config(Some("legacy-secret")),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let response = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token(Algorithm::HS256, "hs-sub", ISSUER, AUDIENCE, 200)
        }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::OK);
}

#[tokio::test]
async fn empty_hs256_secret_rejects_hs256_tokens() {
    let (addr, _state) = spawn(
        config(Some("")),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let response = reqwest::Client::new()
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": hs256_token("empty-secret-sub", b"")
        }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn user_code_exchange_authorizes_device_code() {
    let (addr, _state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let http = reqwest::Client::new();
    let start: Value = http
        .post(format!("{addr}/v1/auth/device/start"))
        .send()
        .await
        .expect("device start")
        .json()
        .await
        .expect("device json");
    let device_code = start["device_code"]
        .as_str()
        .expect("device code")
        .to_string();
    let response: Value = http
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token(Algorithm::ES256, "device-sub", ISSUER, AUDIENCE, 200),
            "user_code": start["user_code"]
        }))
        .send()
        .await
        .expect("exchange")
        .json()
        .await
        .expect("exchange json");
    assert_eq!(response["status"], "authorized");
    let user_id = response["user_id"].as_str().expect("authorized user id");
    assert!(!user_id.is_empty());

    let poll = http
        .post(format!("{addr}/v1/auth/device/poll"))
        .json(&serde_json::json!({ "device_code": device_code }))
        .send()
        .await
        .expect("device poll");
    assert_eq!(poll.status(), reqwest::StatusCode::OK);
    let poll: Value = poll.json().await.expect("device poll json");
    let session_token = poll["session_token"].as_str().expect("session token");
    let me = http
        .get(format!("{addr}/v1/me"))
        .bearer_auth(session_token)
        .send()
        .await
        .expect("me");
    assert_eq!(me.status(), reqwest::StatusCode::OK);
    let me: Value = me.json().await.expect("me json");
    assert_eq!(me["user_id"], user_id);
}

#[tokio::test]
async fn rejected_user_code_does_not_create_supabase_account() {
    let (addr, state) = spawn(
        config(None),
        Clock::Fixed(100 * SEC_NS),
        Arc::new(StaticJwksSource(jwks())),
    )
    .await;
    let response = reqwest::Client::new()
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({
            "access_token": token(Algorithm::ES256, "rejected-sub", ISSUER, AUDIENCE, 200),
            "user_code": "not-a-real-code"
        }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::BAD_REQUEST);
    assert!(state
        .db
        .user_by_identity("supabase", "rejected-sub")
        .expect("identity lookup")
        .is_none());
}

#[tokio::test]
async fn unconfigured_supabase_refuses_loudly() {
    ensure_crypto_provider();
    let email = Arc::new(cerulion_accountd::CapturingEmailSender::new());
    let state = Arc::new(
        AppState::dev(
            ServiceConfig::default(),
            Clock::Fixed(100 * SEC_NS),
            email,
            Arc::new(UnconfiguredResolver),
        )
        .expect("dev app state"),
    );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = format!("http://{}", listener.local_addr().unwrap());
    tokio::spawn(cerulion_accountd::serve(listener, state));
    let response = reqwest::Client::new()
        .post(format!("{addr}/v1/auth/supabase/exchange"))
        .json(&serde_json::json!({ "access_token": "not-used" }))
        .send()
        .await
        .expect("exchange");
    assert_eq!(response.status(), reqwest::StatusCode::NOT_IMPLEMENTED);
    let body = response.text().await.unwrap();
    assert!(body.contains("provider_not_configured"));
}
