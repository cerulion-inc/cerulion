// SPDX-License-Identifier: AGPL-3.0-only
//! # cerulion_accountd — the Cerulion account service
//!
//! The cloud **issuer/CA + account DB + login authority** — the one hosted
//! component the account system adds. It:
//!
//! - runs **OAuth login** (Google / GitHub / email magic-link) and the **RFC 8628
//!   device-authorization grant** (headless / CLI login),
//! - mints opaque **session + refresh tokens** (the WAN-gate credential),
//! - issues **`SignedDeviceCert`s** (binding a device key → an account) against a
//!   rotatable **intermediate** key, itself root-signed M-of-N, and
//! - serves the pinned **`RootSet`** at `GET /.well-known/cerulion-roots`.
//!
//! Every pairing artifact it issues is built from `cerulion_pairing`'s own
//! structures + `.sign()` methods, so it is **byte-compatible** with the shipped
//! robot-side verifier ([`cerulion_pairing::verify::TrustStore`]) — one issuer,
//! one verifier, no second crypto stack (invariant I6).
//!
//! ## Scope
//!
//! Endpoints: `/v1/auth/device/{start,poll}`, `/v1/auth/{refresh,revoke}`,
//! `/v1/auth/magic-link/{start,complete}`, `/v1/auth/oauth/{provider}/{start,callback}`,
//! `/v1/auth/supabase/exchange`, `/v1/me`, `/v1/devices` (POST → device cert, GET → registered devices), and
//! `/.well-known/cerulion-roots`. Grants + epochs are schema-present but
//! endpoint-dormant (A4/A5).
//!
//! ## A2 scope (login-gated ownership at install)
//!
//! `POST /v1/robots` binds the installer's account as a robot's owner and returns
//! an OWNER_FULL owner `SignedGrant` the robot verifies OFFLINE;
//! `GET /v1/robots/{robot_id}` reads a robot's ownership (owner-only). A robot's
//! transport key maps to exactly one owner (one-key-one-robot, like devices).
//!
//! ## A6 scope (revocation + the Team & access page)
//!
//! `GET /v1/robots` lists the caller's OWNED robots; `GET /v1/robots/{id}/access` +
//! `POST /v1/robots/{id}/revoke` read and grow a robot's revocation epoch
//! (owner-only); `POST /v1/devices/{id}/revoke` self-revokes a desk and fans it into
//! every owned robot's epoch. `GET /team` serves the owner-facing **Team & access
//! page**, a hosted web surface Cerulion Studio renders in an
//! in-app webview; see the `team` module for the hosting rationale and the
//! credential-free host handshake.

// Principle 12 (logging; see the Logging convention in `AGENTS.md`): library
// code never prints. It logs through `tracing`. Scoped `not(test)` so unit
// tests keep printing diagnostics, and applied at the crate root rather than
// in `[workspace.lints]` because that table cannot distinguish a lib target
// from a test binary. Pinned by
// `cerulion_cli_engine/tests/library_print_ban_test.rs`.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod api;
mod ca;
mod codec;
mod config;
mod db;
mod device_code;
mod error;
mod hash;
mod oauth;
mod rng;
mod session;
mod supabase;
mod team;

use std::sync::Arc;

pub use api::build_router;
pub use ca::{Ca, CaConfig};
pub use codec::{decode_b64, encode_b64};
pub use config::{
    default_verification_base_uri, Clock, OAuthConfig, ProviderCreds, ServiceConfig, DEFAULT_BIND,
    SEC_NS,
};
pub use db::{Db, EpochRow, RevocationTarget, RobotRow};
pub use device_code::{
    insert_with_collision_retry, poll_outcome, should_record_poll, AttemptOutcome,
    DeviceCodeSnapshot, DeviceCodeState, PollOutcome, RetryGiveUp,
};
pub use error::{AccountdError, Result};
pub use hash::hash_token;
pub use oauth::{
    pkce_challenge, pkce_pair, CapturingEmailSender, EmailSender, GitHubProvider, GoogleProvider,
    Identity, IdentityResolver, LoggingEmailSender, Provider, Providers, UnconfiguredResolver,
};
pub use session::{
    authenticate_session, evaluate_refresh, RefreshDecision, SessionAuth, SessionSnapshot,
};
pub use supabase::{
    JwksSource, ReqwestJwksSource, StaticJwksSource, SupabaseClaims, SupabaseConfig,
    SupabaseVerifier,
};
pub use team::{
    TEAM_HOST_TO_PAGE_FNS, TEAM_PAGE_CSP, TEAM_PAGE_ENDPOINT_EXPRESSIONS, TEAM_PAGE_HTML,
    TEAM_PAGE_TO_HOST_VERBS,
};

/// The shared application state every handler reads.
pub struct AppState {
    /// The certificate authority (issues device certs / grants / epochs).
    pub ca: Ca,
    /// The account database.
    pub db: Db,
    /// Service configuration.
    pub config: ServiceConfig,
    /// The injectable clock (real time in production; pinned in tests).
    pub clock: Clock,
    /// The configured OAuth providers.
    pub providers: Providers,
    /// The live external-identity resolver (loud default; a reqwest-backed
    /// resolver is not implemented).
    pub identity_resolver: Arc<dyn IdentityResolver>,
    /// The magic-link email transport seam.
    pub email: Arc<dyn EmailSender>,
    /// The configured Supabase JWT verifier, if the provider is enabled.
    pub supabase_verifier: Option<Arc<SupabaseVerifier>>,
}

impl AppState {
    /// Assemble application state from its parts.
    pub fn new(
        ca: Ca,
        db: Db,
        config: ServiceConfig,
        clock: Clock,
        identity_resolver: Arc<dyn IdentityResolver>,
        email: Arc<dyn EmailSender>,
    ) -> Self {
        let supabase_verifier = config.supabase.as_ref().map(|supabase| {
            Arc::new(SupabaseVerifier::new(
                supabase.clone(),
                Arc::new(ReqwestJwksSource::new(
                    supabase.jwks_url.clone().unwrap_or_else(|| {
                        format!(
                            "{}/.well-known/jwks.json",
                            supabase.issuer.trim_end_matches('/')
                        )
                    }),
                )),
            ))
        });
        AppState {
            ca,
            db,
            config,
            clock,
            providers: Providers::default(),
            identity_resolver,
            email,
            supabase_verifier,
        }
    }

    /// A dev/test instance: a **dev-provisioned** CA (in-process root ceremony —
    /// NOT a production path) + an in-memory database. Callers inject the email
    /// transport + identity resolver so tests can capture links / stand in for the
    /// external IdP.
    pub fn dev(
        config: ServiceConfig,
        clock: Clock,
        email: Arc<dyn EmailSender>,
        identity_resolver: Arc<dyn IdentityResolver>,
    ) -> Result<Self> {
        let ca = Ca::dev_provision(config.ca.clone(), clock.now_ns())?;
        let db = Db::open_in_memory()?;
        Ok(AppState::new(
            ca,
            db,
            config,
            clock,
            identity_resolver,
            email,
        ))
    }
}

/// Serve the account service on an already-bound listener until the future is
/// dropped / the process exits. The binary adds signal-driven graceful shutdown.
pub async fn serve(listener: tokio::net::TcpListener, state: Arc<AppState>) -> std::io::Result<()> {
    axum::serve(listener, build_router(state)).await
}
