// SPDX-License-Identifier: AGPL-3.0-only
//! The HTTP API surface (axum): auth/session, devices + certs, `/v1/me`, and the
//! `/.well-known/cerulion-roots` bootstrap.
//!
//! Handlers are thin: they parse the request, call the pure state machines
//! ([`crate::device_code`] / [`crate::session`]) and the DB, and shape a response.
//! Every fallible path returns [`AccountdError`], whose [`IntoResponse`] mapping
//! produces the stable JSON error body.

use std::sync::Arc;

use axum::body::Bytes;
use axum::extract::{FromRequest, Path, Query, Request, State};
use axum::http::header::AUTHORIZATION;
use axum::http::{HeaderMap, StatusCode};
use axum::routing::{get, post};
use axum::{Json, Router};
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use cerulion_pairing::format::{AccountId, PrincipalKind, PublicKey, RobotId, Scope, Signature};
use cerulion_pairing::pop::{
    verify_pop, PopVerification, PURPOSE_DEVICE_REGISTRATION, PURPOSE_ROBOT_REGISTRATION,
};

use crate::codec;
use crate::config::SEC_NS;
use crate::db::{RevocationTarget, UserRow};
use crate::device_code::{poll_outcome, should_record_poll, DeviceCodeSnapshot, PollOutcome};
use crate::error::{AccountdError, Result};
use crate::hash::hash_token;
use crate::rng;
use crate::session::{authenticate_session, evaluate_refresh, RefreshDecision, SessionAuth};
use crate::AppState;

/// How many fresh `user_code` candidates `device/start` tries before giving up on
/// a (rare) short-code collision. A few attempts drive the practical failure
/// probability to nil while keeping the loop bounded.
const USER_CODE_MAX_ATTEMPTS: usize = 8;

/// Build the account-service router over an [`AppState`].
pub fn build_router(state: Arc<AppState>) -> Router {
    Router::new()
        .route("/healthz", get(healthz))
        .route("/.well-known/cerulion-roots", get(well_known_roots))
        .route("/v1/auth/device/start", post(device_start))
        .route("/v1/auth/device/poll", post(device_poll))
        .route("/v1/auth/supabase/exchange", post(supabase_exchange))
        .route("/v1/auth/refresh", post(refresh))
        .route("/v1/auth/revoke", post(revoke))
        .route("/v1/auth/magic-link/start", post(magic_link_start))
        .route("/v1/auth/magic-link/complete", get(magic_link_complete))
        .route("/v1/auth/oauth/{provider}/start", get(oauth_start))
        .route("/v1/auth/oauth/{provider}/callback", get(oauth_callback))
        .route("/v1/me", get(me))
        .route("/v1/devices", post(register_device).get(list_devices))
        .route("/v1/devices/challenge", post(device_challenge))
        .route("/v1/devices/{device_id}/revoke", post(revoke_device))
        .route("/v1/robots", post(register_robot).get(list_robots))
        .route("/v1/robots/{robot_id}", get(get_robot))
        .route("/v1/robots/{robot_id}/access", get(robot_access))
        .route("/v1/robots/{robot_id}/revoke", post(robot_revoke))
        // The owner-facing Team & access PAGE. A
        // hosted web surface Studio renders in an in-app webview — see
        // [`crate::team`] for why it is hosted rather than bundled, and why the
        // document itself is deliberately not session-gated (it carries no account
        // data; every datum it shows comes from the session-authed endpoints above).
        .route("/team", get(crate::team::team_page))
        .with_state(state)
}

// ============================================================================
// helpers
// ============================================================================

fn bearer(headers: &HeaderMap) -> Option<String> {
    headers
        .get(AUTHORIZATION)?
        .to_str()
        .ok()?
        .strip_prefix("Bearer ")
        .map(str::to_string)
}

/// Authenticate a request's `Authorization: Bearer <session_token>` and resolve
/// the owning user. The only authorization primitive on the API surface.
fn authenticate(state: &AppState, headers: &HeaderMap) -> Result<UserRow> {
    let token = bearer(headers).ok_or(AccountdError::Unauthorized("missing bearer token"))?;
    let lookup = state
        .db
        .session_by_session_hash(&hash_token(&token))?
        .ok_or(AccountdError::Unauthorized("unknown session token"))?;
    match authenticate_session(&lookup.snapshot, state.clock.now_ns()) {
        SessionAuth::Valid => {}
        SessionAuth::Expired => return Err(AccountdError::Unauthorized("session expired")),
        SessionAuth::Revoked => return Err(AccountdError::Unauthorized("session revoked")),
    }
    state
        .db
        .get_user(&lookup.user_id)?
        .ok_or(AccountdError::Unauthorized("session user no longer exists"))
}

/// A freshly-minted session + refresh token pair (returned to the client once;
/// only the hashes are persisted).
struct MintedSession {
    session_token: String,
    refresh_token: String,
    expires_in_secs: u64,
}

fn mint_tokens(
    state: &AppState,
    now_ns: u64,
) -> Result<(String, String, String, String, u64, u64)> {
    let session_token = rng::opaque_token()?;
    let refresh_token = rng::opaque_token()?;
    let session_exp = now_ns.saturating_add(state.config.session_ttl_ns);
    let refresh_exp = now_ns.saturating_add(state.config.refresh_ttl_ns);
    Ok((
        hash_token(&session_token),
        hash_token(&refresh_token),
        session_token,
        refresh_token,
        session_exp,
        refresh_exp,
    ))
}

fn issue_session(state: &AppState, user_id: &str, now_ns: u64) -> Result<MintedSession> {
    let (session_hash, refresh_hash, session_token, refresh_token, session_exp, refresh_exp) =
        mint_tokens(state, now_ns)?;
    state.db.insert_session(
        user_id,
        &session_hash,
        &refresh_hash,
        session_exp,
        refresh_exp,
        now_ns,
    )?;
    Ok(MintedSession {
        session_token,
        refresh_token,
        expires_in_secs: state.config.session_ttl_ns / SEC_NS,
    })
}

fn parse_pubkey_b64(s: &str) -> Result<PublicKey> {
    Ok(PublicKey(parse_key32_b64(s, "public_key")?))
}

/// Parse a base64url (no-padding) 32-byte value, naming `field` in the error.
fn parse_key32_b64(s: &str, field: &'static str) -> Result<[u8; 32]> {
    let bytes = URL_SAFE_NO_PAD.decode(s.as_bytes()).map_err(|_| {
        AccountdError::BadRequest(format!("{field} must be base64url (no padding)"))
    })?;
    bytes
        .try_into()
        .map_err(|_| AccountdError::BadRequest(format!("{field} must be 32 bytes")))
}

/// Parse a base64url (no-padding) 64-byte ed25519 signature, naming `field` in the
/// error. A malformed signature is a `BadRequest` (400) — distinct from a well-formed
/// signature that does not VERIFY (a `ProofOfPossessionFailed` 403).
fn parse_sig64_b64(s: &str, field: &'static str) -> Result<Signature> {
    let bytes = URL_SAFE_NO_PAD.decode(s.as_bytes()).map_err(|_| {
        AccountdError::BadRequest(format!("{field} must be base64url (no padding)"))
    })?;
    let arr: [u8; 64] = bytes
        .try_into()
        .map_err(|_| AccountdError::BadRequest(format!("{field} must be 64 bytes")))?;
    Ok(Signature(arr))
}

/// A JSON body extractor for the PoP-gated registration endpoints (`POST
/// /v1/devices`, `POST /v1/robots`). Unlike the stock `Json<T>` extractor — whose
/// deserialization failure surfaces as a framework-default 422 with a bare serde
/// string — a parse failure here maps to the crate's STABLE
/// [`AccountdError::BadRequest`] body (`{error:"invalid_request", error_description}`)
/// that NAMES the missing proof-of-possession requirement + the challenge endpoint.
/// This keeps a client that omits `pop_challenge`/`pop_signature` on the
/// account service's loud-actionable-error norm rather than an unnamed 4xx.
struct PopJson<T>(T);

impl<T, S> FromRequest<S> for PopJson<T>
where
    T: DeserializeOwned,
    S: Send + Sync,
{
    type Rejection = AccountdError;

    async fn from_request(req: Request, state: &S) -> std::result::Result<Self, Self::Rejection> {
        let bytes = Bytes::from_request(req, state)
            .await
            .map_err(|_| AccountdError::BadRequest("could not read the request body".into()))?;
        let value = serde_json::from_slice::<T>(&bytes).map_err(|e| {
            AccountdError::BadRequest(format!(
                "invalid request body: {e}. This endpoint requires a proof of possession — request \
                 a challenge from POST /v1/devices/challenge, sign it with the key you are \
                 registering, and include `pop_challenge` + `pop_signature`."
            ))
        })?;
        Ok(PopJson(value))
    }
}

/// Enforce a proof-of-possession for `key`: the caller must have signed
/// `challenge` (issued to `account`) with `key`'s private half. TWO independent checks
/// that BOTH must pass:
///
/// 1. VERIFY the signature (stateless, no side effect, so a transient bad signature
///    does not burn the challenge). A structurally-invalid key is a `BadRequest`
///    (400, naming `key_field`); a valid key with a non-verifying signature is a
///    `ProofOfPossessionFailed` (403) — the enum's already-computed distinction is
///    preserved so the remediation is accurate (a bad KEY is not fixable by re-signing).
/// 2. SPEND the challenge (single-use + account-bound + expiry-gated CAS). A `false`
///    means the challenge is unknown / already spent (replay) / expired / issued to
///    another account — all rejected identically as a 403. Spending AFTER a valid
///    signature means a genuine PoP consumes exactly one challenge and a replay fails.
#[allow(clippy::too_many_arguments)]
fn enforce_pop(
    state: &AppState,
    account: &[u8; 32],
    key: &[u8; 32],
    purpose: &str,
    challenge: &str,
    signature: &Signature,
    key_field: &'static str,
    now_ns: u64,
) -> Result<()> {
    match verify_pop(&PublicKey(*key), purpose, account, challenge, signature) {
        PopVerification::Ok => {}
        PopVerification::BadKey => {
            return Err(AccountdError::BadRequest(format!(
                "{key_field} is not a valid ed25519 public key"
            )));
        }
        PopVerification::BadSignature => {
            return Err(AccountdError::ProofOfPossessionFailed(format!(
                "the signature does not prove possession of {key_field}'s private key \
                 (request a fresh challenge from POST /v1/devices/challenge and sign it with the key)"
            )));
        }
    }
    if !state
        .db
        .consume_device_challenge(&hash_token(challenge), account, now_ns)?
    {
        return Err(AccountdError::ProofOfPossessionFailed(
            "the proof-of-possession challenge is unknown, expired, already used, or was issued to \
             a different account — request a fresh one from POST /v1/devices/challenge"
                .into(),
        ));
    }
    Ok(())
}

fn parse_principal(s: &str) -> Result<PrincipalKind> {
    match s.to_ascii_lowercase().as_str() {
        "human" => Ok(PrincipalKind::Human),
        "machine" => Ok(PrincipalKind::Machine),
        _ => Err(AccountdError::BadRequest(
            "principal_kind must be `human` or `machine`".into(),
        )),
    }
}

fn principal_str(kind: u8) -> &'static str {
    match kind {
        2 => "machine",
        _ => "human",
    }
}

fn principal_from_u8(kind: u8) -> PrincipalKind {
    match kind {
        2 => PrincipalKind::Machine,
        _ => PrincipalKind::Human,
    }
}

// ============================================================================
// health + bootstrap
// ============================================================================

async fn healthz() -> &'static str {
    "ok"
}

/// The pinned root set the signed installer verifies against.
#[derive(Serialize)]
struct RootsResponse {
    /// `base64url(postcard(RootSet))`.
    root_set: String,
    /// The pairing wire `FORMAT_VERSION`.
    format_version: u16,
}

async fn well_known_roots(State(state): State<Arc<AppState>>) -> Result<Json<RootsResponse>> {
    Ok(Json(RootsResponse {
        root_set: codec::encode_b64(state.ca.root_set())?,
        format_version: cerulion_pairing::format::FORMAT_VERSION,
    }))
}

// ============================================================================
// device-authorization grant (RFC 8628)
// ============================================================================

#[derive(Serialize)]
struct DeviceStartResponse {
    device_code: String,
    user_code: String,
    verification_uri: String,
    verification_uri_complete: String,
    expires_in: u64,
    interval: u64,
}

async fn device_start(State(state): State<Arc<AppState>>) -> Result<Json<DeviceStartResponse>> {
    let now = state.clock.now_ns();
    // Opportunistic housekeeping (best-effort — never fail the request): sweep
    // expired device codes / magic links / oauth flows so these tables stay bounded.
    if let Err(e) = state.db.sweep_expired(now) {
        tracing::warn!(error = %e, "expired-row sweep failed (non-fatal)");
    }
    let device_code = rng::opaque_token()?;
    let expires_at = now.saturating_add(state.config.device_code_ttl_ns);
    // Allocate a UNIQUE user_code with bounded retry: a rare birthday collision on
    // the short human code is retried with a fresh candidate rather than surfacing
    // as a 500.
    let user_code = state.db.insert_device_code_retrying(
        &hash_token(&device_code),
        expires_at,
        state.config.device_code_interval_secs,
        now,
        USER_CODE_MAX_ATTEMPTS,
        rng::user_code,
    )?;
    let verification_uri = state.config.verification_uri();
    let verification_uri_complete = format!("{verification_uri}?user_code={user_code}");
    Ok(Json(DeviceStartResponse {
        device_code,
        user_code,
        verification_uri,
        verification_uri_complete,
        expires_in: state.config.device_code_ttl_ns / SEC_NS,
        interval: state.config.device_code_interval_secs,
    }))
}

#[derive(Deserialize)]
struct DevicePollRequest {
    device_code: String,
}

#[derive(Serialize)]
struct TokenResponse {
    session_token: String,
    refresh_token: String,
    token_type: &'static str,
    expires_in: u64,
}

async fn device_poll(
    State(state): State<Arc<AppState>>,
    Json(req): Json<DevicePollRequest>,
) -> Result<Json<TokenResponse>> {
    let now = state.clock.now_ns();
    let hash = hash_token(&req.device_code);
    let snap: DeviceCodeSnapshot = state
        .db
        .device_code_snapshot(&hash)?
        .ok_or(AccountdError::BadRequest("unknown device_code".into()))?;
    let outcome = poll_outcome(&snap, now);
    // Record this poll so the NEXT one is interval-gated — but NEVER on a SlowDown:
    // recording a throttled poll resets `last_poll_at` and would permanently lock
    // out a client polling just inside the interval (each SlowDown pushes the
    // reference forward). Only a non-throttled outcome advances the gate.
    if should_record_poll(&outcome) {
        state.db.touch_device_code_poll(&hash, now)?;
    }
    match outcome {
        PollOutcome::AuthorizationPending => Err(AccountdError::AuthorizationPending),
        PollOutcome::SlowDown => Err(AccountdError::SlowDown),
        PollOutcome::Expired => Err(AccountdError::ExpiredToken),
        PollOutcome::AlreadyRedeemed => Err(AccountdError::AlreadyRedeemed),
        PollOutcome::Authorized { user_id } => {
            // Atomic single-use redemption: flip authorized→consumed as a
            // compare-and-swap and issue a session ONLY if THIS poll won the race.
            // Two concurrent polls of one authorized code → exactly one session
            // (the loser sees AlreadyRedeemed). Consume-then-issue closes the
            // TOCTOU window in which both could mint before either consumed.
            if !state.db.redeem_device_code(&hash)? {
                return Err(AccountdError::AlreadyRedeemed);
            }
            let minted = issue_session(&state, &user_id, now)?;
            Ok(Json(TokenResponse {
                session_token: minted.session_token,
                refresh_token: minted.refresh_token,
                token_type: "Bearer",
                expires_in: minted.expires_in_secs,
            }))
        }
    }
}

#[derive(Deserialize)]
struct SupabaseExchangeRequest {
    access_token: String,
    user_code: Option<String>,
}

async fn supabase_exchange(
    State(state): State<Arc<AppState>>,
    Json(req): Json<SupabaseExchangeRequest>,
) -> Result<Json<serde_json::Value>> {
    let verifier = state
        .supabase_verifier
        .as_ref()
        .ok_or_else(|| AccountdError::ProviderNotConfigured("supabase".into()))?;
    let now = state.clock.now_ns();
    let claims = verifier
        .verify(&req.access_token, now)
        .await
        .map_err(|err| match err {
            AccountdError::Internal(_) => err,
            _ => AccountdError::Unauthorized("invalid supabase token"),
        })?;
    let now = state.clock.now_ns();
    if let Some(user_code) = req.user_code {
        let user = state
            .db
            .upsert_user_for_device_code(
                "supabase",
                &claims.sub,
                claims.email.as_deref(),
                PrincipalKind::Human as u8,
                &user_code,
                now,
            )?
            .ok_or(AccountdError::BadRequest(
                "unknown or expired user_code".into(),
            ))?;
        return Ok(Json(serde_json::json!({
            "status": "authorized",
            "user_id": user.user_id,
        })));
    }
    let user = state.db.upsert_user_by_identity(
        "supabase",
        &claims.sub,
        claims.email.as_deref(),
        PrincipalKind::Human as u8,
        now,
    )?;
    let minted = issue_session(&state, &user.user_id, now)?;
    Ok(Json(serde_json::json!({
        "session_token": minted.session_token,
        "refresh_token": minted.refresh_token,
        "token_type": "Bearer",
        "expires_in": minted.expires_in_secs,
    })))
}

// ============================================================================
// refresh + revoke
// ============================================================================

#[derive(Deserialize)]
struct RefreshRequest {
    refresh_token: String,
}

async fn refresh(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RefreshRequest>,
) -> Result<Json<TokenResponse>> {
    let now = state.clock.now_ns();
    let old_refresh_hash = hash_token(&req.refresh_token);
    let lookup = state
        .db
        .session_by_refresh_hash(&old_refresh_hash)?
        .ok_or(AccountdError::Unauthorized("unknown refresh token"))?;
    // Fast-path rejection on a clearly expired/revoked snapshot (the CAS below is
    // the authoritative guard under concurrency).
    match evaluate_refresh(&lookup.snapshot, now) {
        RefreshDecision::Ok => {}
        RefreshDecision::Expired => {
            return Err(AccountdError::Unauthorized("refresh token expired"))
        }
        RefreshDecision::Revoked => return Err(AccountdError::Unauthorized("session revoked")),
    }
    // Rotate both tokens onto the existing session row via an atomic CAS: the
    // rotation only succeeds if the row is STILL un-revoked and STILL carries the
    // OLD refresh hash. This makes refresh single-use (two concurrent refreshes
    // with the same token → one wins) and closes the concurrent-revoke clobber
    // (Findings 3 + 4). The loser is Unauthorized.
    let (
        new_session_hash,
        new_refresh_hash,
        session_token,
        refresh_token,
        session_exp,
        refresh_exp,
    ) = mint_tokens(&state, now)?;
    let won = state.db.rotate_session(
        &lookup.id,
        &old_refresh_hash,
        &new_session_hash,
        &new_refresh_hash,
        session_exp,
        refresh_exp,
    )?;
    if !won {
        return Err(AccountdError::Unauthorized(
            "refresh token already used or session revoked",
        ));
    }
    Ok(Json(TokenResponse {
        session_token,
        refresh_token,
        token_type: "Bearer",
        expires_in: state.config.session_ttl_ns / SEC_NS,
    }))
}

#[derive(Deserialize)]
struct RevokeRequest {
    token: String,
}

async fn revoke(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RevokeRequest>,
) -> Result<StatusCode> {
    // RFC 7009: revocation is idempotent — an unknown token still returns success.
    let _ = state
        .db
        .revoke_session_by_token_hash(&hash_token(&req.token))?;
    Ok(StatusCode::NO_CONTENT)
}

// ============================================================================
// magic-link login (the hermetic login path)
// ============================================================================

#[derive(Deserialize)]
struct MagicLinkStartRequest {
    email: String,
    user_code: String,
}

async fn magic_link_start(
    State(state): State<Arc<AppState>>,
    Json(req): Json<MagicLinkStartRequest>,
) -> Result<(StatusCode, Json<serde_json::Value>)> {
    if req.email.is_empty() || !req.email.contains('@') {
        return Err(AccountdError::BadRequest(
            "a valid email is required".into(),
        ));
    }
    let now = state.clock.now_ns();
    let token = rng::opaque_token()?;
    let expires_at = now.saturating_add(state.config.magic_link_ttl_ns);
    state
        .db
        .insert_magic_link(&hash_token(&token), &req.email, &req.user_code, expires_at)?;
    let base = state.config.verification_base_uri.trim_end_matches('/');
    let link = format!(
        "{base}/v1/auth/magic-link/complete?token={token}&user_code={}",
        req.user_code
    );
    state.email.send_magic_link(&req.email, &link)?;
    Ok((
        StatusCode::ACCEPTED,
        Json(serde_json::json!({ "status": "sent" })),
    ))
}

#[derive(Deserialize)]
struct MagicLinkCompleteParams {
    token: String,
    #[allow(dead_code)]
    user_code: Option<String>,
}

async fn magic_link_complete(
    State(state): State<Arc<AppState>>,
    Query(params): Query<MagicLinkCompleteParams>,
) -> Result<Json<serde_json::Value>> {
    let now = state.clock.now_ns();
    let (email, stored_user_code) = state
        .db
        .consume_magic_link(&hash_token(&params.token), now)?
        .ok_or(AccountdError::BadRequest(
            "magic link is invalid, already used, or expired".into(),
        ))?;
    // The email IS the provider subject for the magic-link provider.
    let user = state.db.upsert_user_by_identity(
        "email",
        &email,
        Some(&email),
        PrincipalKind::Human as u8,
        now,
    )?;
    // Authorize the device code the link was minted for (the stored code is
    // authoritative — the query copy is advisory).
    let authorized = state
        .db
        .authorize_device_code(&stored_user_code, &user.user_id, now)?;
    Ok(Json(serde_json::json!({
        "status": if authorized { "authorized" } else { "logged_in" },
        "user_id": user.user_id,
    })))
}

// ============================================================================
// OAuth (Google / GitHub) — abstraction + config-gated start/callback
// ============================================================================

#[derive(Deserialize)]
struct OAuthStartParams {
    user_code: Option<String>,
}

#[derive(Serialize)]
struct OAuthStartResponse {
    authorize_url: String,
    state: String,
}

async fn oauth_start(
    State(state): State<Arc<AppState>>,
    Path(provider): Path<String>,
    Query(params): Query<OAuthStartParams>,
) -> Result<Json<OAuthStartResponse>> {
    let prov = state
        .providers
        .by_name(&provider)
        .ok_or_else(|| AccountdError::NotFound("unknown oauth provider"))?;
    let creds = match provider.as_str() {
        "google" => state.config.oauth.google.as_ref(),
        "github" => state.config.oauth.github.as_ref(),
        _ => None,
    }
    .ok_or_else(|| AccountdError::ProviderNotConfigured(provider.clone()))?;

    let now = state.clock.now_ns();
    let csrf_state = rng::opaque_token()?;
    let (verifier, challenge) = crate::oauth::pkce_pair()?;
    let expires_at = now.saturating_add(state.config.oauth_flow_ttl_ns);
    state.db.insert_oauth_flow(
        &csrf_state,
        &provider,
        &verifier,
        params.user_code.as_deref(),
        expires_at,
        now,
    )?;
    let authorize_url = prov.authorize_url(creds, &csrf_state, &challenge);
    Ok(Json(OAuthStartResponse {
        authorize_url,
        state: csrf_state,
    }))
}

#[derive(Deserialize)]
struct OAuthCallbackParams {
    code: String,
    state: String,
}

async fn oauth_callback(
    State(state): State<Arc<AppState>>,
    Path(provider): Path<String>,
    Query(params): Query<OAuthCallbackParams>,
) -> Result<Json<serde_json::Value>> {
    let now = state.clock.now_ns();
    // PEEK the flow (validate exists + not expired) WITHOUT consuming it — the row
    // is only consumed AFTER a successful exchange, so a failed exchange leaves the
    // login retryable. The expiry/CSRF guard rides the peek.
    let (flow_provider, pkce_verifier, user_code) = state
        .db
        .peek_oauth_flow(&params.state, now)?
        .ok_or(AccountdError::BadRequest(
        "unknown or expired oauth state".into(),
    ))?;
    if flow_provider != provider {
        return Err(AccountdError::BadRequest("oauth provider mismatch".into()));
    }
    // The live token exchange is the injectable identity-resolver seam (the service
    // ships the loud default; the reqwest-backed resolver is installed over it). A failure here
    // returns WITHOUT consuming the flow — the client can retry.
    let identity = state
        .identity_resolver
        .resolve(&provider, &params.code, &pkce_verifier)?;
    // Exchange succeeded → consume the flow (atomic single-use; the loser of a
    // concurrent double-callback is rejected).
    if !state.db.consume_oauth_flow(&params.state)? {
        return Err(AccountdError::BadRequest(
            "oauth state already consumed".into(),
        ));
    }
    let user = state.db.upsert_user_by_identity(
        &identity.provider,
        &identity.subject,
        identity.email.as_deref(),
        PrincipalKind::Human as u8,
        now,
    )?;
    let authorized = if let Some(code) = user_code {
        state.db.authorize_device_code(&code, &user.user_id, now)?
    } else {
        false
    };
    Ok(Json(serde_json::json!({
        "status": if authorized { "authorized" } else { "logged_in" },
        "user_id": user.user_id,
    })))
}

// ============================================================================
// /v1/me
// ============================================================================

#[derive(Serialize)]
struct MeResponse {
    user_id: String,
    account_id: String,
    orgs: Vec<String>,
    principal_kind: &'static str,
    email: Option<String>,
}

async fn me(State(state): State<Arc<AppState>>, headers: HeaderMap) -> Result<Json<MeResponse>> {
    let user = authenticate(&state, &headers)?;
    Ok(Json(MeResponse {
        user_id: user.user_id,
        account_id: URL_SAFE_NO_PAD.encode(user.account_id),
        // Org membership is dormant: nothing populates it.
        orgs: Vec::new(),
        principal_kind: principal_str(user.principal_kind),
        email: user.email,
    }))
}

// ============================================================================
// /v1/devices
// ============================================================================

#[derive(Deserialize)]
struct RegisterDeviceRequest {
    /// The ed25519 device public key, base64url (no padding).
    public_key: String,
    /// `human` | `machine`.
    principal_kind: String,
    /// The proof-of-possession challenge (from `POST /v1/devices/challenge`) the
    /// caller signed with the private half of `public_key`.
    pop_challenge: String,
    /// The ed25519 signature over the PoP message, base64url (no pad). Proves the
    /// caller holds the private half of `public_key` before the account service certs
    /// it — closing the device-key registration-squatting DoS (an attacker cannot
    /// pre-register a victim's public EndpointId under its own account, which would
    /// permanently block the victim from ever obtaining a cert for its OWN key).
    pop_signature: String,
}

#[derive(Serialize)]
struct RegisterDeviceResponse {
    device_id: String,
    /// `base64url(postcard(SignedDeviceCert))`.
    device_cert: String,
    /// `base64url(postcard(SignedIntermediateCert))` — the chain link the client
    /// presents alongside the device cert.
    intermediate: String,
    /// `base64url(AccountId)`.
    account_id: String,
}

async fn register_device(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    PopJson(req): PopJson<RegisterDeviceRequest>,
) -> Result<Json<RegisterDeviceResponse>> {
    let user = authenticate(&state, &headers)?;
    let public_key = parse_pubkey_b64(&req.public_key)?;
    let requested_principal = parse_principal(&req.principal_kind)?;
    let pop_signature = parse_sig64_b64(&req.pop_signature, "pop_signature")?;
    let now = state.clock.now_ns();

    // Proof-of-possession: the caller must hold the private half of the
    // key it is certifying. Closes the device-key registration-squatting DoS — an
    // attacker cannot bind a victim's public EndpointId to its own account (which,
    // because db.register_device 409s any cross-account re-use and there is no un-bind
    // endpoint, would permanently wedge the victim's OWN device-cert acquisition).
    enforce_pop(
        &state,
        &user.account_id,
        &public_key.0,
        PURPOSE_DEVICE_REGISTRATION,
        &req.pop_challenge,
        &pop_signature,
        "public_key",
        now,
    )?;

    // register_device refuses (409) a key already owned by a DIFFERENT account, so
    // the returned row is guaranteed to belong to the caller. A same-account
    // re-registration returns the STORED row (its original principal_kind), so we
    // mint the cert from the STORED row — never the request — keeping the issued
    // cert, the DB registry, and the response mutually consistent (a re-register
    // cannot silently re-bind the account or flip principal_kind).
    let device = state.db.register_device(
        &user.account_id,
        &public_key.0,
        requested_principal as u8,
        now,
    )?;

    // A REVOKED device (a lost/decommissioned desk the caller self-revoked)
    // must not be able to obtain a FRESH device cert — that would re-admit the cut key.
    // The `register_device` DB call is idempotent (returns the stored row for a
    // same-account re-registration), so a revoked stored row surfaces here as a loud
    // refusal. A device is un-revoked only by re-provisioning a new key.
    if device.revoked {
        return Err(AccountdError::Conflict(
            "this device key is REVOKED — it cannot obtain a new device cert. \
             Register a fresh device key instead."
                .into(),
        ));
    }
    let stored_principal = principal_from_u8(device.principal_kind);

    // Issue the offline device cert binding the key to the account. The device
    // may exercise its account's full scope; per-robot authorization is bounded
    // by grants, not here.
    let signed = state.ca.issue_device_cert(
        public_key,
        AccountId(device.account_id),
        stored_principal,
        Scope::OWNER_FULL,
        now,
    );

    Ok(Json(RegisterDeviceResponse {
        device_id: device.device_id,
        device_cert: codec::encode_b64(&signed)?,
        intermediate: codec::encode_b64(state.ca.intermediate())?,
        account_id: URL_SAFE_NO_PAD.encode(device.account_id),
    }))
}

#[derive(Serialize)]
struct DeviceEntry {
    device_id: String,
    public_key: String,
    principal_kind: &'static str,
    created_at_ns: u64,
    revoked: bool,
}

#[derive(Serialize)]
struct ListDevicesResponse {
    devices: Vec<DeviceEntry>,
}

async fn list_devices(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
) -> Result<Json<ListDevicesResponse>> {
    let user = authenticate(&state, &headers)?;
    let devices = state
        .db
        .list_devices(&user.account_id)?
        .into_iter()
        .map(|d| DeviceEntry {
            device_id: d.device_id,
            public_key: URL_SAFE_NO_PAD.encode(d.public_key),
            principal_kind: principal_str(d.principal_kind),
            created_at_ns: d.created_at_ns,
            revoked: d.revoked,
        })
        .collect();
    Ok(Json(ListDevicesResponse { devices }))
}

// ============================================================================
// /v1/devices/challenge — proof-of-possession challenge
// ============================================================================

#[derive(Serialize)]
struct DeviceChallengeResponse {
    /// The opaque single-use challenge bearer the client signs with its device key.
    challenge: String,
    /// `base64url(AccountId)` — the account this challenge is bound to. The client
    /// signs the PoP message with EXACTLY this account so the issuer's rebuilt message
    /// matches (the server also enforces the binding on spend).
    account_id: String,
    /// Seconds until the challenge expires.
    expires_in: u64,
}

/// `POST /v1/devices/challenge` — issue a fresh proof-of-possession challenge
/// (session-authed). The caller signs the returned `challenge` with the
/// private half of the device/transport key it is about to register and presents the
/// signature to a PoP-gated endpoint (`POST /v1/robots`). The challenge is SINGLE-USE,
/// bound to the caller's account, and short-lived; only its SHA-256 hash is stored.
async fn device_challenge(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
) -> Result<Json<DeviceChallengeResponse>> {
    let user = authenticate(&state, &headers)?;
    let now = state.clock.now_ns();
    // Opportunistic housekeeping so the challenge table stays bounded.
    if let Err(e) = state.db.sweep_expired(now) {
        tracing::warn!(error = %e, "expired-row sweep failed (non-fatal)");
    }
    let challenge = rng::opaque_token()?;
    let expires_at = now.saturating_add(state.config.pop_challenge_ttl_ns);
    state
        .db
        .insert_device_challenge(&hash_token(&challenge), &user.account_id, expires_at, now)?;
    Ok(Json(DeviceChallengeResponse {
        challenge,
        account_id: URL_SAFE_NO_PAD.encode(user.account_id),
        expires_in: state.config.pop_challenge_ttl_ns / SEC_NS,
    }))
}

// ============================================================================
// /v1/robots — login-gated ownership at install
// ============================================================================

#[derive(Deserialize)]
struct RegisterRobotRequest {
    /// The robot's human name (its stamped hostname — the robot identity).
    hostname: String,
    /// The robot's transport public key (its iroh EndpointId), base64url (no pad).
    robot_transport_key: String,
    /// The proof-of-possession challenge (from `POST /v1/devices/challenge`) the
    /// caller signed with the private half of `robot_transport_key`.
    pop_challenge: String,
    /// The ed25519 signature over the PoP message, base64url (no pad). Proves the
    /// caller holds the private half of `robot_transport_key`.
    pop_signature: String,
    /// The owning org (optional; bound verbatim — org membership is NOT
    /// validated here).
    #[serde(default)]
    org_id: Option<String>,
}

#[derive(Serialize)]
struct RegisterRobotResponse {
    /// `base64url(RobotId)`.
    robot_id: String,
    /// `base64url(postcard(SignedGrant))` — the installer's OWNER_FULL owner grant,
    /// which the robot verifies OFFLINE to write its owner access row.
    owner_grant: String,
    /// `base64url(postcard(SignedIntermediateCert))` — the chain link the robot
    /// presents alongside the grant + device cert when verifying the owner claim.
    intermediate: String,
    /// `base64url(AccountId)` — the installer account now bound as owner.
    account_id: String,
}

/// `POST /v1/robots` — bind the installer's account as the robot's owner (decision
/// 2, login-gated ownership at install). Session-authed; the caller account owns
/// the robot. Returns the minted `robot_id` + an OWNER_FULL owner `SignedGrant`
/// (subject = caller account) that the robot verifies offline to write its owner
/// access row (via `TrustStore::claim_by_owner_grant`).
///
/// PROOF OF POSSESSION: the caller MUST prove it holds the PRIVATE half
/// of `robot_transport_key` before the key is bound as a robot. It does so by signing
/// a fresh, single-use, account-bound challenge (from `POST /v1/devices/challenge`)
/// with the key and presenting the signature (`pop_challenge` + `pop_signature`). The
/// service (1) verifies the signature against the presented key over the canonical
/// PoP message (`cerulion_pairing::pop::verify_pop`), and (2) atomically SPENDS the
/// challenge (single-use + account-bound + expiry-gated). Both must pass. This closes
/// the registration-squatting hole: a caller cannot burn the
/// `one key = one robot owner` slot for a key it does not control — the signature is
/// only producible by the key holder.
///
/// `org_id` is bound verbatim with NO org-membership check — validating that the
/// caller belongs to the named org is NOT done here (the team page owns org
/// membership); a `None` org is a personally-owned robot.
async fn register_robot(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    PopJson(req): PopJson<RegisterRobotRequest>,
) -> Result<Json<RegisterRobotResponse>> {
    let user = authenticate(&state, &headers)?;
    if req.hostname.trim().is_empty() {
        return Err(AccountdError::BadRequest(
            "hostname must not be empty (it is the robot identity)".into(),
        ));
    }
    let transport_key = parse_key32_b64(&req.robot_transport_key, "robot_transport_key")?;
    let pop_signature = parse_sig64_b64(&req.pop_signature, "pop_signature")?;
    let now = state.clock.now_ns();

    // Proof-of-possession: verify (BadKey → 400, BadSignature → 403) then
    // spend the single-use, account-bound challenge — both must pass. Verify precedes
    // spend so a transient bad signature does not burn the challenge; the spend proves
    // the challenge was server-issued to THIS account and is fresh (replay-safe). This
    // closes the robot-registration squatting hole.
    enforce_pop(
        &state,
        &user.account_id,
        &transport_key,
        PURPOSE_ROBOT_REGISTRATION,
        &req.pop_challenge,
        &pop_signature,
        "robot_transport_key",
        now,
    )?;

    // register_robot refuses (409) a transport key already owned by a DIFFERENT
    // account, so the returned row is guaranteed to belong to the caller. A
    // same-owner re-registration returns the STORED row, so the owner grant is
    // always minted for the row's actual owner (== the caller here).
    let robot = state.db.register_robot(
        &user.account_id,
        req.hostname.trim(),
        &transport_key,
        req.org_id.as_deref(),
        now,
    )?;

    // Mint the OWNER_FULL owner grant for the installer's account. The device cert
    // (from POST /v1/devices) + this grant + the intermediate form the offline
    // presentation the robot verifies to claim ownership.
    let owner_grant = state.ca.issue_grant(
        AccountId(robot.owner_account_id),
        RobotId(robot.robot_id),
        Scope::OWNER_FULL,
        principal_from_u8(user.principal_kind),
        now,
    );

    Ok(Json(RegisterRobotResponse {
        robot_id: URL_SAFE_NO_PAD.encode(robot.robot_id),
        owner_grant: codec::encode_b64(&owner_grant)?,
        intermediate: codec::encode_b64(state.ca.intermediate())?,
        account_id: URL_SAFE_NO_PAD.encode(robot.owner_account_id),
    }))
}

#[derive(Serialize)]
struct RobotResponse {
    robot_id: String,
    hostname: String,
    /// `base64url(AccountId)` of the owner.
    owner_account_id: String,
    org_id: Option<String>,
    created_at_ns: u64,
}

/// `GET /v1/robots/{robot_id}` — read a robot's ownership. Session-authed +
/// OWNER-ONLY: a caller that is not the robot's owner gets a 404 (the robot's
/// existence is not revealed to non-owners). An org-scoped read is not
/// supported.
async fn get_robot(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Path(robot_id): Path<String>,
) -> Result<Json<RobotResponse>> {
    let user = authenticate(&state, &headers)?;
    let id = parse_key32_b64(&robot_id, "robot_id")?;
    // Owner-only: a non-owner (or unknown robot) is indistinguishably NotFound so a
    // valid session cannot enumerate robots it does not own.
    let robot = state
        .db
        .get_robot(&id)?
        .filter(|r| r.owner_account_id == user.account_id)
        .ok_or(AccountdError::NotFound("robot"))?;
    Ok(Json(RobotResponse {
        robot_id: URL_SAFE_NO_PAD.encode(robot.robot_id),
        hostname: robot.hostname,
        owner_account_id: URL_SAFE_NO_PAD.encode(robot.owner_account_id),
        org_id: robot.org_id,
        created_at_ns: robot.created_at_ns,
    }))
}

#[derive(Serialize)]
struct ListRobotsResponse {
    robots: Vec<RobotResponse>,
}

/// `GET /v1/robots` — every robot the CALLER OWNS (the Team & access
/// page's entry query). Session-authed and account-scoped by construction: it lists
/// `robots WHERE owner_account_id = <caller>`, so there is nothing to leak and no id
/// to guess — a caller simply cannot see a robot it does not own.
///
/// This is the enumeration the `team` module's page needs and `GET /v1/robots/{id}`
/// cannot provide (that one answers about a robot id you must already know). Robots
/// the account is only a GUEST on are NOT listed — those grants are desk-carried and
/// off-cloud, so the service genuinely does not know about them (see
/// `docs/revocation.md`'s scope note).
async fn list_robots(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
) -> Result<Json<ListRobotsResponse>> {
    let user = authenticate(&state, &headers)?;
    let robots = state
        .db
        .list_robots_by_owner(&user.account_id)?
        .into_iter()
        .map(|r| RobotResponse {
            robot_id: URL_SAFE_NO_PAD.encode(r.robot_id),
            hostname: r.hostname,
            owner_account_id: URL_SAFE_NO_PAD.encode(r.owner_account_id),
            org_id: r.org_id,
            created_at_ns: r.created_at_ns,
        })
        .collect();
    Ok(Json(ListRobotsResponse { robots }))
}

// ============================================================================
// /v1/devices/{device_id}/revoke — self-revoke a device
// ============================================================================

#[derive(Serialize)]
struct RevokeDeviceResponse {
    device_id: String,
    public_key: String,
    revoked: bool,
    /// How many robots OWNED by the caller had this device added to their revocation
    /// epoch. These robots refuse the device at their accept gate once
    /// they sync the epoch. Robots the account is a GUEST on (granted-but-not-owned)
    /// are NOT covered here — the owner of THOSE robots must revoke the device via
    /// `POST /v1/robots/{robot_id}/revoke` (the owner-only surface).
    robots_updated: usize,
    /// `base64url(RobotId)` of any OWNED robots whose epoch bump FAILED.
    /// The device is still revoked (cloud record + re-registration
    /// blocked); re-run the revoke (idempotent) to reach these robots. Empty on the
    /// happy path (the epoch bump is atomic + race-free, so this is only a genuine DB
    /// error).
    robots_failed: Vec<String>,
}

/// `POST /v1/devices/{device_id}/revoke` — SELF-revoke a device the caller owns (a
/// lost/decommissioned desk). Session-authed; the revoke is guarded on BOTH the
/// device id AND the caller's account, so a caller can only revoke its OWN device
/// (another account's device is indistinguishably `NotFound`). Idempotent.
///
/// Real enforcement, not a cosmetic flag: beyond flipping the cloud
/// `devices.revoked` record — which also refuses any FUTURE re-registration of the
/// key (`register_device`) — this FANS the device out into the revocation epoch of
/// every robot the CALLER OWNS, so those robots refuse it at their accept gate once
/// they sync (reusing the exact `robot_revoke` epoch machinery). `robots_updated`
/// counts them. **Scope:** a device the account is only a GUEST on (a robot it
/// does not own) is NOT cut here — that robot's OWNER must revoke it via
/// `POST /v1/robots/{robot_id}/revoke`; and enforcement lands only once each robot
/// SYNCS the epoch (the offline gap — see docs/revocation.md).
async fn revoke_device(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Path(device_id): Path<String>,
) -> Result<Json<RevokeDeviceResponse>> {
    let user = authenticate(&state, &headers)?;
    let device = state
        .db
        .revoke_device(&user.account_id, &device_id)?
        .ok_or(AccountdError::NotFound("device"))?;

    // Fan the revoked device out into every OWNED robot's revocation epoch so the
    // revoke actually cuts the desk (not just a cosmetic flag). Idempotent per robot
    // (an already-revoked device mints no new epoch there).
    //
    // ALL-OR-REPORT: the device is ALREADY revoked in the
    // cloud record (above), so a per-robot epoch-bump failure must NOT abort the loop
    // (that would leave the caller with a 409/500 implying total failure while the
    // device IS revoked and some robots WERE epoch'd). Instead we continue past a
    // failed robot and report it in `robots_failed` — the caller re-runs the revoke
    // (idempotent) to retry the stragglers. The per-robot bump is itself atomic + race-
    // free (`Db::add_revocation`), so this failure path is only a genuine DB error.
    let now = state.clock.now_ns();
    let target = RevocationTarget::Device(device.public_key);
    let mut robots_updated = 0usize;
    let mut robots_failed: Vec<String> = Vec::new();
    for robot in state.db.list_robots_by_owner(&user.account_id)? {
        match add_robot_revocation(&state, &robot.robot_id, target, now) {
            Ok(true) => robots_updated += 1,
            Ok(false) => {} // already revoked on this robot — no new epoch
            Err(e) => {
                let robot_b64 = URL_SAFE_NO_PAD.encode(robot.robot_id);
                tracing::error!(
                    device_id = %device.device_id,
                    robot = %robot_b64,
                    error = %e,
                    "device self-revoke could not bump a robot's epoch; the device \
                     IS revoked (cloud + re-registration blocked) — retry the revoke to reach \
                     this robot"
                );
                robots_failed.push(robot_b64);
            }
        }
    }
    if robots_updated > 0 || !robots_failed.is_empty() {
        tracing::info!(
            device_id = %device.device_id,
            robots_updated,
            robots_failed = robots_failed.len(),
            "self-revoked device fanned out into owned robots' revocation epochs"
        );
    }

    Ok(Json(RevokeDeviceResponse {
        device_id: device.device_id,
        public_key: URL_SAFE_NO_PAD.encode(device.public_key),
        revoked: device.revoked,
        robots_updated,
        robots_failed,
    }))
}

// ============================================================================
// /v1/robots/{robot_id}/{access,revoke} — the robot's ACL revocation epoch
// ============================================================================

/// Authenticate the caller AND assert it owns `robot_id`, returning both. A
/// non-owner (or unknown robot) is indistinguishably [`AccountdError::NotFound`], so
/// only the owner may inspect or mutate the robot's access list (broader org-scoped
/// access lands with the team page).
fn authenticate_robot_owner(
    state: &AppState,
    headers: &HeaderMap,
    robot_id: &str,
) -> Result<(UserRow, crate::db::RobotRow)> {
    let user = authenticate(state, headers)?;
    let id = parse_key32_b64(robot_id, "robot_id")?;
    let robot = state
        .db
        .get_robot(&id)?
        .filter(|r| r.owner_account_id == user.account_id)
        .ok_or(AccountdError::NotFound("robot"))?;
    Ok((user, robot))
}

/// The robot's current access-list revocation state. The `signed_epoch`
/// is the artifact the robot applies via `TrustStore::apply_epoch`; it is absent only
/// when the robot has never had a revocation (nothing to sync).
#[derive(Serialize)]
struct RobotAccessResponse {
    /// `base64url(RobotId)`.
    robot_id: String,
    /// The current epoch number (0 when the robot has no revocations yet).
    epoch: u64,
    /// The revoked account ids, `base64url`.
    revoked_accounts: Vec<String>,
    /// The revoked device (transport) keys, `base64url`.
    revoked_devices: Vec<String>,
    /// `base64url(postcard(SignedEpoch))` — the robot's sync artifact (absent when
    /// there is no epoch yet).
    signed_epoch: Option<String>,
    /// `base64url(postcard(SignedIntermediateCert))` — the chain link the robot
    /// verifies the epoch's signature against (absent iff `signed_epoch` is).
    intermediate: Option<String>,
}

/// Build the current [`RobotAccessResponse`] for a robot from its latest epoch (the
/// shared read/shape half of `GET .../access` AND `POST .../revoke`). The signed
/// epoch is re-issued from the STORED `issued_at_ns`, so repeated reads return the
/// BYTE-IDENTICAL artifact (and a robot re-applying an already-applied epoch is a
/// monotonic no-op). No revocations yet ⇒ epoch 0, empty sets, no signed epoch.
fn build_access_response(state: &AppState, robot_id_b: &[u8; 32]) -> Result<RobotAccessResponse> {
    match state.db.latest_epoch(robot_id_b)? {
        None => Ok(RobotAccessResponse {
            robot_id: URL_SAFE_NO_PAD.encode(robot_id_b),
            epoch: 0,
            revoked_accounts: Vec::new(),
            revoked_devices: Vec::new(),
            signed_epoch: None,
            intermediate: None,
        }),
        Some(row) => {
            let signed = state.ca.issue_epoch(
                RobotId(*robot_id_b),
                row.epoch,
                row.revoked_accounts
                    .iter()
                    .copied()
                    .map(AccountId)
                    .collect(),
                row.revoked_devices.iter().copied().map(PublicKey).collect(),
                row.issued_at_ns,
            );
            Ok(RobotAccessResponse {
                robot_id: URL_SAFE_NO_PAD.encode(robot_id_b),
                epoch: row.epoch,
                revoked_accounts: row
                    .revoked_accounts
                    .iter()
                    .map(|k| URL_SAFE_NO_PAD.encode(k))
                    .collect(),
                revoked_devices: row
                    .revoked_devices
                    .iter()
                    .map(|k| URL_SAFE_NO_PAD.encode(k))
                    .collect(),
                signed_epoch: Some(codec::encode_b64(&signed)?),
                intermediate: Some(codec::encode_b64(state.ca.intermediate())?),
            })
        }
    }
}

/// Add a revocation target to a robot's set + bump its signed epoch — the shared core
/// of the owner-driven `robot_revoke` AND the device self-revoke's owned-robot
/// fan-out. Returns `true` iff a NEW epoch was minted (idempotent: an already-present
/// target mints nothing and returns `false`). The caller enforces authz + the
/// owner-account guard BEFORE calling this.
///
/// The read-modify-write is ATOMIC under ONE DB lock ([`crate::db::Db::add_revocation`]):
/// a concurrent revoke on the same robot cannot slip an epoch in between the read
/// and the write, so this never spuriously conflicts.
fn add_robot_revocation(
    state: &AppState,
    robot_id_b: &[u8; 32],
    target: RevocationTarget,
    now: u64,
) -> Result<bool> {
    let minted = state.db.add_revocation(robot_id_b, target, now)?;
    if let Some(row) = &minted {
        tracing::info!(
            robot = %URL_SAFE_NO_PAD.encode(robot_id_b),
            epoch = row.epoch,
            revoked_accounts = row.revoked_accounts.len(),
            revoked_devices = row.revoked_devices.len(),
            "bumped the robot's revocation epoch"
        );
    }
    Ok(minted.is_some())
}

/// `GET /v1/robots/{robot_id}/access` — the robot's current revocation state + the
/// signed epoch to sync (owner-only).
async fn robot_access(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Path(robot_id): Path<String>,
) -> Result<Json<RobotAccessResponse>> {
    let (_user, robot) = authenticate_robot_owner(&state, &headers, &robot_id)?;
    Ok(Json(build_access_response(&state, &robot.robot_id)?))
}

#[derive(Deserialize)]
struct RobotRevokeRequest {
    /// Revoke a whole account's access to the robot (`base64url(AccountId)`).
    #[serde(default)]
    account_id: Option<String>,
    /// Revoke ONE device key from the robot (`base64url(PublicKey)`) — the account's
    /// OTHER devices keep their access.
    #[serde(default)]
    device_key: Option<String>,
}

/// `POST /v1/robots/{robot_id}/revoke` — owner-only. Add a target ACCOUNT or DEVICE
/// key to the robot's revocation set, bump the epoch, and return the fresh signed
/// epoch the robot applies. EXACTLY ONE of `account_id` / `device_key` must be
/// present (both / neither is a 400). Re-revoking an already-revoked target is
/// idempotent (a no-op that returns the CURRENT epoch, minting no new one). Only the
/// robot's owner may revoke.
///
/// **Owner self-lockout guard:** revoking the robot's OWN OWNER account
/// is REFUSED with a 400 — it would apply wholesale (`is_allowed` short-circuits) and
/// permanently lock the robot out, recoverable only by a physical chassis-secret
/// factory reset (the epoch sets are grow-only; there is no un-revoke endpoint). The
/// owner always retains access; to hand a robot over, use the ownership-transfer flow
/// (Studio / the web account page), not a self-revoke. Revoking one of the owner's
/// own DEVICE keys is still allowed — that cuts one desk, the account (and its other
/// devices) keep access, and it is recoverable by registering a new device.
async fn robot_revoke(
    State(state): State<Arc<AppState>>,
    headers: HeaderMap,
    Path(robot_id): Path<String>,
    Json(req): Json<RobotRevokeRequest>,
) -> Result<Json<RobotAccessResponse>> {
    let (_user, robot) = authenticate_robot_owner(&state, &headers, &robot_id)?;
    let robot_id_b = robot.robot_id;

    // Exactly one target.
    let target = match (req.account_id.as_deref(), req.device_key.as_deref()) {
        (Some(a), None) => RevocationTarget::Account(parse_key32_b64(a, "account_id")?),
        (None, Some(d)) => RevocationTarget::Device(parse_key32_b64(d, "device_key")?),
        (Some(_), Some(_)) => {
            return Err(AccountdError::BadRequest(
                "provide exactly one of account_id / device_key, not both".into(),
            ))
        }
        (None, None) => {
            return Err(AccountdError::BadRequest(
                "provide exactly one of account_id / device_key".into(),
            ))
        }
    };

    // Owner self-lockout guard: refuse revoking the robot's OWN owner account (a
    // permanent, only-chassis-recoverable lockout). Refuse LOUDLY (never a silent
    // success that does nothing) so the caller learns why.
    if let RevocationTarget::Account(a) = target {
        if a == robot.owner_account_id {
            return Err(AccountdError::BadRequest(
                "cannot revoke the robot's OWNER account — the owner always retains access; \
                 revoking it would permanently lock the robot out (recoverable only by a \
                 physical factory reset). To hand the robot over, use ownership transfer."
                    .into(),
            ));
        }
    }

    // Apply (idempotent) then return the fresh state + signed epoch.
    let now = state.clock.now_ns();
    add_robot_revocation(&state, &robot_id_b, target, now)?;
    Ok(Json(build_access_response(&state, &robot_id_b)?))
}
