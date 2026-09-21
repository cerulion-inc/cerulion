// SPDX-License-Identifier: AGPL-3.0-only
//! The GitHub OAuth provider (authorization-code).

use super::{Identity, Provider};
use crate::config::ProviderCreds;
use crate::error::{AccountdError, Result};

/// GitHub's OAuth authorization endpoint.
const AUTH_ENDPOINT: &str = "https://github.com/login/oauth/authorize";

/// The GitHub login provider.
#[derive(Debug, Default, Clone, Copy)]
pub struct GitHubProvider;

impl Provider for GitHubProvider {
    fn name(&self) -> &'static str {
        "github"
    }

    fn authorize_url(&self, creds: &ProviderCreds, state: &str, _pkce_challenge: &str) -> String {
        // GitHub's classic OAuth app flow does not implement PKCE; the `state`
        // parameter is the anti-CSRF binding. (The verifier is still generated +
        // stored uniformly, so the flow shape matches Google's.)
        let mut url = url::Url::parse(AUTH_ENDPOINT).expect("static GitHub auth endpoint is valid");
        url.query_pairs_mut()
            .append_pair("client_id", &creds.client_id)
            .append_pair("redirect_uri", &creds.redirect_uri)
            .append_pair("scope", "read:user user:email")
            .append_pair("state", state);
        url.to_string()
    }

    fn map_identity(&self, userinfo: &serde_json::Value) -> Result<Identity> {
        // GitHub `/user`: `id` is the stable numeric subject; `email` may be null.
        let subject = userinfo
            .get("id")
            .and_then(|v| v.as_i64())
            .map(|n| n.to_string())
            .ok_or_else(|| AccountdError::BadRequest("github user missing numeric `id`".into()))?;
        let email = userinfo
            .get("email")
            .and_then(|v| v.as_str())
            .map(str::to_string);
        Ok(Identity {
            provider: "github".to_string(),
            subject,
            email,
        })
    }
}
