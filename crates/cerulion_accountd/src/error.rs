// SPDX-License-Identifier: AGPL-3.0-only
//! The unified crate error type and its HTTP mapping.
//!
//! Every fallible path returns [`AccountdError`]; the axum [`IntoResponse`] impl
//! maps each variant to an HTTP status + a stable JSON body
//! `{ "error": "<code>", "error_description": "<message>" }`. The device-code
//! polling errors ([`AccountdError::AuthorizationPending`] /
//! [`AccountdError::SlowDown`] / [`AccountdError::ExpiredToken`]) carry the exact
//! RFC 8628 §3.5 error codes so a standard device-flow client understands them.

use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde_json::json;
use thiserror::Error;

/// The single error type for the account service.
#[derive(Debug, Error)]
pub enum AccountdError {
    /// A database operation failed.
    #[error("database error: {0}")]
    Db(#[from] rusqlite::Error),
    /// A pairing-layer operation (root-set build, chain construction) failed.
    #[error("pairing error: {0}")]
    Pairing(#[from] cerulion_pairing::PairingError),
    /// A container (de)serialization failed.
    #[error("serialization error: {0}")]
    Serialization(String),
    /// The platform RNG failed to produce entropy.
    #[error("entropy error: {0}")]
    Random(String),

    /// No valid session was presented (missing/expired/revoked bearer token).
    #[error("unauthorized: {0}")]
    Unauthorized(&'static str),
    /// A referenced resource does not exist.
    #[error("not found: {0}")]
    NotFound(&'static str),
    /// The request was malformed.
    #[error("bad request: {0}")]
    BadRequest(String),
    /// A resource already exists / a uniqueness constraint was violated.
    #[error("conflict: {0}")]
    Conflict(String),
    /// The caller is authenticated but failed to prove possession of the device key
    /// it presented: a signature that does not verify, or a challenge
    /// that is unknown / expired / already spent / issued to another account. Maps to
    /// 403 Forbidden — the session is valid, but the key-ownership proof is not.
    #[error("proof of possession failed: {0}")]
    ProofOfPossessionFailed(String),

    /// An OAuth provider was requested but is not configured (no client id).
    /// A loud refusal, never a silent bypass (no bypass tiers, by design).
    #[error("oauth provider not configured: {0}")]
    ProviderNotConfigured(String),
    /// The external identity resolver is not installed (this crate ships the loud
    /// default; the live Google/GitHub token exchange is not implemented).
    #[error("identity resolver not installed for provider: {0}")]
    IdentityResolverUnavailable(String),

    // -- RFC 8628 device-authorization polling outcomes ----------------------
    /// The user has not yet authorized the device code — keep polling.
    #[error("authorization pending")]
    AuthorizationPending,
    /// Polled faster than the interval — back off and retry.
    #[error("slow down")]
    SlowDown,
    /// The device code has expired; start a new device-authorization request.
    #[error("expired token")]
    ExpiredToken,
    /// The device code was already redeemed for a token pair.
    #[error("device code already redeemed")]
    AlreadyRedeemed,

    /// An internal invariant failed.
    #[error("internal error: {0}")]
    Internal(String),
    /// The daemon configuration is invalid.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
}

impl AccountdError {
    /// The stable machine-readable error code (used in the JSON body).
    fn code(&self) -> &'static str {
        match self {
            AccountdError::Db(_) => "db_error",
            AccountdError::Pairing(_) => "pairing_error",
            AccountdError::Serialization(_) => "serialization_error",
            AccountdError::Random(_) => "entropy_error",
            AccountdError::Unauthorized(_) => "unauthorized",
            AccountdError::NotFound(_) => "not_found",
            AccountdError::BadRequest(_) => "invalid_request",
            AccountdError::Conflict(_) => "conflict",
            AccountdError::ProofOfPossessionFailed(_) => "proof_of_possession_failed",
            AccountdError::ProviderNotConfigured(_) => "provider_not_configured",
            AccountdError::IdentityResolverUnavailable(_) => "identity_resolver_unavailable",
            AccountdError::AuthorizationPending => "authorization_pending",
            AccountdError::SlowDown => "slow_down",
            AccountdError::ExpiredToken => "expired_token",
            AccountdError::AlreadyRedeemed => "access_denied",
            AccountdError::Internal(_) => "internal_error",
            AccountdError::InvalidConfig(_) => "invalid_config",
        }
    }

    /// The HTTP status this error maps to.
    fn status(&self) -> StatusCode {
        match self {
            AccountdError::Unauthorized(_) => StatusCode::UNAUTHORIZED,
            AccountdError::NotFound(_) => StatusCode::NOT_FOUND,
            AccountdError::Conflict(_) => StatusCode::CONFLICT,
            AccountdError::ProofOfPossessionFailed(_) => StatusCode::FORBIDDEN,
            // RFC 8628 §3.5: all device-flow polling errors are HTTP 400 with the
            // code in the body.
            AccountdError::BadRequest(_)
            | AccountdError::AuthorizationPending
            | AccountdError::SlowDown
            | AccountdError::ExpiredToken
            | AccountdError::AlreadyRedeemed => StatusCode::BAD_REQUEST,
            AccountdError::ProviderNotConfigured(_)
            | AccountdError::IdentityResolverUnavailable(_) => StatusCode::NOT_IMPLEMENTED,
            AccountdError::Db(_)
            | AccountdError::Pairing(_)
            | AccountdError::Serialization(_)
            | AccountdError::Random(_)
            | AccountdError::Internal(_)
            | AccountdError::InvalidConfig(_) => StatusCode::INTERNAL_SERVER_ERROR,
        }
    }
}

impl IntoResponse for AccountdError {
    fn into_response(self) -> Response {
        let status = self.status();
        let code = self.code();
        if status.is_server_error() {
            // A 5xx is a real fault. Its detail (raw rusqlite text — table names,
            // constraint names, internal messages) must NEVER reach the client.
            // Log the full detail server-side with a correlation id; the client
            // gets only an opaque message + that id (to quote in a bug report).
            let error_id = crate::rng::opaque_token().unwrap_or_else(|_| "unavailable".to_string());
            tracing::error!(error = %self, %code, %error_id, "account service internal error");
            let body = Json(json!({
                "error": code,
                "error_description": "an internal error occurred",
                "error_id": error_id,
            }));
            return (status, body).into_response();
        }
        // 4xx: the message is client-actionable (missing token, bad request,
        // provider not configured) and carries no server internals.
        let body = Json(json!({
            "error": code,
            "error_description": self.to_string(),
        }));
        (status, body).into_response()
    }
}

/// Crate result alias.
pub type Result<T> = std::result::Result<T, AccountdError>;
