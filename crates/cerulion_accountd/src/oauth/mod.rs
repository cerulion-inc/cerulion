// SPDX-License-Identifier: AGPL-3.0-only
//! OAuth login: the provider abstraction (Google / GitHub) + the magic-link path.
//!
//! Three login providers back the account service: **Google**,
//! **GitHub** (authorization-code + PKCE), and **email magic-link**. A provider
//! with no configured client id is UNCONFIGURED — its start endpoint refuses
//! loudly (never a silent bypass).
//!
//! ## What ships
//!
//! - **magic-link** — fully round-tripped and hermetic: the email *transport* is a
//!   pluggable [`EmailSender`] seam whose dev/test impls log / capture the link
//!   (correct dev behavior, NOT a bypass — the flow is identical, only the delivery
//!   transport differs). This is the acceptance login path.
//! - **Google / GitHub** — the [`Provider`] abstraction: PKCE-S256 authorization
//!   URL construction (pure, unit-tested) + provider→[`Identity`] mapping (pure,
//!   unit-tested), config-gated so an unconfigured provider refuses loudly. The
//!   live network token exchange is the [`IdentityResolver`] seam; this crate ships the
//!   loud default ([`UnconfiguredResolver`]) and the live reqwest-backed resolver
//!   is not implemented here: it belongs with the login UX that consumes it.

mod github;
mod google;
mod magic_link;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use sha2::{Digest, Sha256};

pub use github::GitHubProvider;
pub use google::GoogleProvider;
pub use magic_link::{CapturingEmailSender, EmailSender, LoggingEmailSender};

use crate::config::ProviderCreds;
use crate::error::{AccountdError, Result};
use crate::rng;

/// An external identity resolved from a provider login.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    /// The provider (`google` | `github` | `email`).
    pub provider: String,
    /// The provider-scoped stable subject id.
    pub subject: String,
    /// The user's email, if the provider supplied one.
    pub email: Option<String>,
}

/// An OAuth authorization-code provider (Google / GitHub).
pub trait Provider: Send + Sync {
    /// The provider name (`google` | `github`).
    fn name(&self) -> &'static str;

    /// Build the provider's PKCE-S256 authorization URL. `state` is the anti-CSRF
    /// lookup key; `pkce_challenge` is `base64url(sha256(verifier))`.
    fn authorize_url(&self, creds: &ProviderCreds, state: &str, pkce_challenge: &str) -> String;

    /// Map a provider userinfo JSON payload to an [`Identity`]. Pure — the live
    /// resolver fetches the payload, this decides the mapping.
    fn map_identity(&self, userinfo: &serde_json::Value) -> Result<Identity>;
}

/// The live external-identity resolver: exchanges an authorization code (+ its
/// PKCE verifier) for the resolved [`Identity`]. This is the network seam; the crate
/// ships [`UnconfiguredResolver`] (loud refusal) and the live reqwest-backed
/// resolver is not implemented.
pub trait IdentityResolver: Send + Sync {
    /// Resolve the identity behind an authorization code.
    fn resolve(&self, provider: &str, code: &str, pkce_verifier: &str) -> Result<Identity>;
}

/// The A0 default resolver: refuses loudly (no live token exchange yet, and never
/// a silent bypass).
#[derive(Debug, Default)]
pub struct UnconfiguredResolver;

impl IdentityResolver for UnconfiguredResolver {
    fn resolve(&self, provider: &str, _code: &str, _pkce_verifier: &str) -> Result<Identity> {
        Err(AccountdError::IdentityResolverUnavailable(
            provider.to_string(),
        ))
    }
}

/// The configured OAuth providers.
pub struct Providers {
    google: GoogleProvider,
    github: GitHubProvider,
}

impl Default for Providers {
    fn default() -> Self {
        Providers {
            google: GoogleProvider,
            github: GitHubProvider,
        }
    }
}

impl Providers {
    /// Look up a provider by name (`google` | `github`).
    pub fn by_name(&self, name: &str) -> Option<&dyn Provider> {
        match name {
            "google" => Some(&self.google),
            "github" => Some(&self.github),
            _ => None,
        }
    }
}

/// A PKCE-S256 `(verifier, challenge)` pair. The verifier is a 43-char base64url
/// high-entropy string (a valid RFC 7636 verifier); the challenge is
/// `base64url(sha256(verifier))`.
pub fn pkce_pair() -> Result<(String, String)> {
    let verifier = rng::opaque_token()?;
    let challenge = pkce_challenge(&verifier);
    Ok((verifier, challenge))
}

/// The PKCE-S256 challenge for a verifier.
pub fn pkce_challenge(verifier: &str) -> String {
    URL_SAFE_NO_PAD.encode(Sha256::digest(verifier.as_bytes()))
}
