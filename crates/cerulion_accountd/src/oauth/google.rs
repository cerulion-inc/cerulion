// SPDX-License-Identifier: AGPL-3.0-only
//! The Google OAuth provider (authorization-code + PKCE-S256, OpenID Connect).

use super::{Identity, Provider};
use crate::config::ProviderCreds;
use crate::error::{AccountdError, Result};

/// Google's OAuth 2.0 / OIDC authorization endpoint.
const AUTH_ENDPOINT: &str = "https://accounts.google.com/o/oauth2/v2/auth";

/// The Google login provider.
#[derive(Debug, Default, Clone, Copy)]
pub struct GoogleProvider;

impl Provider for GoogleProvider {
    fn name(&self) -> &'static str {
        "google"
    }

    fn authorize_url(&self, creds: &ProviderCreds, state: &str, pkce_challenge: &str) -> String {
        let mut url = url::Url::parse(AUTH_ENDPOINT).expect("static Google auth endpoint is valid");
        url.query_pairs_mut()
            .append_pair("client_id", &creds.client_id)
            .append_pair("redirect_uri", &creds.redirect_uri)
            .append_pair("response_type", "code")
            .append_pair("scope", "openid email")
            .append_pair("state", state)
            .append_pair("code_challenge", pkce_challenge)
            .append_pair("code_challenge_method", "S256")
            .append_pair("access_type", "offline");
        url.to_string()
    }

    fn map_identity(&self, userinfo: &serde_json::Value) -> Result<Identity> {
        // OIDC userinfo: `sub` is the stable subject; `email` is optional.
        let subject = userinfo
            .get("sub")
            .and_then(|v| v.as_str())
            .ok_or_else(|| AccountdError::BadRequest("google userinfo missing `sub`".into()))?;
        let email = userinfo
            .get("email")
            .and_then(|v| v.as_str())
            .map(str::to_string);
        Ok(Identity {
            provider: "google".to_string(),
            subject: subject.to_string(),
            email,
        })
    }
}
