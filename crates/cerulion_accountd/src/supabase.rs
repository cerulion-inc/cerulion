// SPDX-License-Identifier: AGPL-3.0-only
//! Supabase Auth is the team identity; accountd maps it to an `AccountId`.
//!
//! Verification is deliberately isolated behind [`JwksSource`]. Production uses
//! [`ReqwestJwksSource`], while tests can inject a deterministic static source.
//! Cached keys are re-validated against JWKS at least hourly.

use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use jsonwebtoken::jwk::{AlgorithmParameters, EllipticCurve, Jwk, JwkSet, KeyAlgorithm};
use jsonwebtoken::{decode, decode_header, Algorithm, DecodingKey, Validation};
use serde::Deserialize;
use url::Url;

use crate::config::SEC_NS;
use crate::{AccountdError, Result};

const LEEWAY_SECS: u64 = 60;
const REFETCH_INTERVAL_NS: u64 = 60 * SEC_NS;
// A cached key is served without a network round trip for at most this long.
// Afterward, the whole set is refetched before any cached `kid` is trusted.
const JWKS_MAX_AGE_NS: u64 = 60 * 60 * SEC_NS;
const JWKS_FETCH_TIMEOUT: Duration = Duration::from_secs(10);
const INVALID_JWKS_URL: &str =
    "supabase jwks url must use https (http is allowed for loopback only)";

/// Configuration for the Supabase JWT identity provider.
#[derive(Clone)]
pub struct SupabaseConfig {
    /// The exact JWT issuer accepted from Supabase; empty is treated as unset.
    pub issuer: String,
    /// The audience accepted from Supabase.
    pub audience: String,
    /// JWKS endpoint. Defaults to `{issuer}/.well-known/jwks.json`.
    pub jwks_url: Option<String>,
    /// Legacy HS256 signing secret, if enabled; empty is treated as unset.
    pub hs256_secret: Option<String>,
}

impl std::fmt::Debug for SupabaseConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SupabaseConfig")
            .field("issuer", &self.issuer)
            .field("audience", &self.audience)
            .field("jwks_url", &self.jwks_url)
            .field(
                "hs256_secret",
                &self.hs256_secret.as_ref().map(|_| "<redacted>"),
            )
            .finish()
    }
}

impl SupabaseConfig {
    /// Build configuration from environment values, deriving the standard JWKS
    /// endpoint when one is not explicitly supplied.
    pub fn from_env_values(
        issuer: Option<String>,
        audience: Option<String>,
        jwks_url: Option<String>,
        hs256_secret: Option<String>,
    ) -> Result<Option<Self>> {
        let issuer = issuer
            .map(|issuer| issuer.trim().to_string())
            .filter(|issuer| !issuer.is_empty());
        let Some(issuer) = issuer else {
            return Ok(None);
        };
        let audience = audience
            .map(|audience| audience.trim().to_string())
            .filter(|audience| !audience.is_empty())
            .unwrap_or_else(|| "authenticated".to_string());
        let jwks_url = jwks_url
            .map(|jwks_url| jwks_url.trim().to_string())
            .filter(|jwks_url| !jwks_url.is_empty())
            .unwrap_or_else(|| format!("{}/.well-known/jwks.json", issuer.trim_end_matches('/')));
        validate_jwks_url(&jwks_url)?;
        Ok(Some(Self {
            issuer,
            audience,
            jwks_url: Some(jwks_url),
            hs256_secret: hs256_secret
                .map(|secret| secret.trim().to_string())
                .filter(|secret| !secret.is_empty()),
        }))
    }
}

fn validate_jwks_url(url: &str) -> Result<()> {
    let parsed =
        Url::parse(url).map_err(|_| AccountdError::InvalidConfig(INVALID_JWKS_URL.into()))?;
    match parsed.scheme() {
        "https" => Ok(()),
        "http" if is_loopback_http(&parsed) => Ok(()),
        _ => Err(AccountdError::InvalidConfig(INVALID_JWKS_URL.into())),
    }
}

fn is_loopback_http(url: &Url) -> bool {
    if url.scheme() != "http" {
        return false;
    }
    let Some(host) = url.host_str() else {
        return false;
    };
    let host = host
        .strip_prefix('[')
        .and_then(|host| host.strip_suffix(']'))
        .unwrap_or(host);
    host.eq_ignore_ascii_case("localhost")
        || host
            .parse::<std::net::IpAddr>()
            .is_ok_and(|address| address.is_loopback())
}

/// An asynchronous, injectable source of Supabase signing keys.
pub trait JwksSource: Send + Sync {
    /// Fetch the current JWKS.
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>>;
}

/// Production JWKS source backed by reqwest.
#[derive(Clone, Debug)]
pub struct ReqwestJwksSource {
    url: String,
    client: reqwest::Client,
}

impl ReqwestJwksSource {
    /// Create a source for `url`.
    pub fn new(url: impl Into<String>) -> Self {
        let url = url.into();
        let allow_loopback_http = Url::parse(&url).is_ok_and(|parsed| is_loopback_http(&parsed));
        let mut client = reqwest::Client::builder().timeout(JWKS_FETCH_TIMEOUT);
        if !allow_loopback_http {
            client = client.https_only(true);
        }
        Self {
            url,
            client: client
                .build()
                .expect("fixed Supabase JWKS client configuration is valid"),
        }
    }
}

impl JwksSource for ReqwestJwksSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        Box::pin(async move {
            self.client
                .get(&self.url)
                .send()
                .await
                .map_err(|e| AccountdError::Internal(format!("supabase jwks request failed: {e}")))?
                .error_for_status()
                .map_err(|e| {
                    AccountdError::Internal(format!("supabase jwks response failed: {e}"))
                })?
                .json::<JwkSet>()
                .await
                .map_err(|e| AccountdError::Internal(format!("supabase jwks decode failed: {e}")))
        })
    }
}

/// A deterministic JWKS source useful for tests and local bring-up.
#[derive(Clone, Debug)]
pub struct StaticJwksSource(pub JwkSet);

impl JwksSource for StaticJwksSource {
    fn fetch(&self) -> Pin<Box<dyn Future<Output = Result<JwkSet>> + Send + '_>> {
        let jwks = self.0.clone();
        Box::pin(async move { Ok(jwks) })
    }
}

/// A Supabase JWT verifier with a small in-memory JWKS cache.
///
/// Cached keys are re-validated against JWKS at least hourly of process
/// uptime. Fetch attempts and cache freshness both use monotonic time, so
/// wall-clock steps cannot affect key caching or amplify outbound requests.
pub struct SupabaseVerifier {
    /// The accepted issuer, audience, and legacy secret.
    pub config: SupabaseConfig,
    jwks: Arc<dyn JwksSource>,
    /// Monotonic time used to throttle outbound JWKS fetches.
    monotonic: Arc<dyn Fn() -> u64 + Send + Sync>,
    cache: Mutex<JwksCache>,
    fetch_lock: tokio::sync::Mutex<()>,
}

#[derive(Default)]
struct JwksCache {
    keys: Option<JwkSet>,
    /// Monotonic timestamp of the last JWKS fetch attempt. This bounds the
    /// outbound fetch rate and must not be affected by wall-clock steps.
    last_attempt_mono_ns: Option<u64>,
    /// Monotonic timestamp of the last successful JWKS fetch. This controls
    /// cached-key freshness as a duration of process uptime.
    last_success_mono_ns: Option<u64>,
}

impl std::fmt::Debug for SupabaseVerifier {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SupabaseVerifier")
            .field("config", &self.config)
            .finish_non_exhaustive()
    }
}

impl SupabaseVerifier {
    /// Construct a verifier with an injectable JWKS source.
    pub fn new(config: SupabaseConfig, jwks: Arc<dyn JwksSource>) -> Self {
        let start = Instant::now();
        Self::with_monotonic(
            config,
            jwks,
            Arc::new(move || start.elapsed().as_nanos() as u64),
        )
    }

    /// Construct a verifier with an injectable JWKS source and monotonic clock.
    pub fn with_monotonic(
        config: SupabaseConfig,
        jwks: Arc<dyn JwksSource>,
        monotonic: Arc<dyn Fn() -> u64 + Send + Sync>,
    ) -> Self {
        Self {
            config,
            jwks,
            monotonic,
            cache: Mutex::new(JwksCache::default()),
            fetch_lock: tokio::sync::Mutex::new(()),
        }
    }

    /// Verify a token against the configured Supabase issuer and audience.
    ///
    /// Expiration is checked against `now_ns`, not the process wall clock, so
    /// callers retain deterministic time control in tests.
    pub async fn verify(&self, token: &str, now_ns: u64) -> Result<SupabaseClaims> {
        let header = decode_header(token)
            .map_err(|_| AccountdError::Unauthorized("invalid supabase token"))?;
        let (algorithm, key) = match header.alg {
            Algorithm::HS256 => {
                let secret = self
                    .config
                    .hs256_secret
                    .as_deref()
                    .filter(|secret| !secret.is_empty())
                    .ok_or(AccountdError::Unauthorized("invalid supabase token"))?;
                (
                    Algorithm::HS256,
                    DecodingKey::from_secret(secret.as_bytes()),
                )
            }
            Algorithm::ES256 | Algorithm::RS256 => {
                let kid = header
                    .kid
                    .as_deref()
                    .ok_or(AccountdError::Unauthorized("invalid supabase token"))?;
                let jwk = self.key_for(kid).await?;
                let algorithm = jwk_algorithm(&jwk)?;
                if algorithm != header.alg {
                    return Err(AccountdError::Unauthorized("invalid supabase token"));
                }
                let key = DecodingKey::try_from(&jwk)
                    .map_err(|_| AccountdError::Unauthorized("invalid supabase token"))?;
                (algorithm, key)
            }
            _ => return Err(AccountdError::Unauthorized("invalid supabase token")),
        };

        let mut validation = Validation::new(algorithm);
        validation.leeway = LEEWAY_SECS;
        validation.validate_exp = false;
        validation.set_issuer(std::slice::from_ref(&self.config.issuer));
        validation.set_audience(std::slice::from_ref(&self.config.audience));
        validation.set_required_spec_claims(&["exp", "iss", "aud", "sub"]);

        let claims = decode::<SupabaseClaims>(token, &key, &validation)
            .map_err(|_| AccountdError::Unauthorized("invalid supabase token"))?
            .claims;
        let now_secs = now_ns / SEC_NS;
        if claims.exp.saturating_add(LEEWAY_SECS) < now_secs
            || claims.role.as_deref() != Some("authenticated")
        {
            return Err(AccountdError::Unauthorized("invalid supabase token"));
        }
        if claims.is_anonymous {
            return Err(AccountdError::Unauthorized("invalid supabase token"));
        }
        Ok(claims)
    }

    /// A stale set whose revalidation fails is rejected, and the failure is
    /// throttled like any other fetch attempt.
    async fn key_for(&self, kid: &str) -> Result<Jwk> {
        let mono_now_ns = (self.monotonic)();
        if let Some(key) = self.fresh_cached_key(kid, mono_now_ns)? {
            return Ok(key);
        }

        let _fetch_guard = self.fetch_lock.lock().await;
        let mono_now_ns = (self.monotonic)();
        if let Some(key) = self.fresh_cached_key(kid, mono_now_ns)? {
            return Ok(key);
        }

        let refetch_allowed = {
            let cache = self.lock_cache()?;
            cache
                .last_attempt_mono_ns
                .is_none_or(|last| mono_now_ns.saturating_sub(last) >= REFETCH_INTERVAL_NS)
        };
        if !refetch_allowed {
            return Err(AccountdError::Unauthorized("invalid supabase token"));
        }
        self.lock_cache()?.last_attempt_mono_ns = Some(mono_now_ns);
        let fetched = tokio::time::timeout(JWKS_FETCH_TIMEOUT, self.jwks.fetch())
            .await
            .map_err(|_| AccountdError::Unauthorized("invalid supabase token"))?
            .map_err(|_| AccountdError::Unauthorized("invalid supabase token"))?;
        let found = fetched.find(kid).cloned();
        let mut cache = self.lock_cache()?;
        cache.keys = Some(fetched);
        cache.last_success_mono_ns = Some((self.monotonic)());
        found.ok_or(AccountdError::Unauthorized("invalid supabase token"))
    }

    fn lock_cache(&self) -> Result<std::sync::MutexGuard<'_, JwksCache>> {
        self.cache
            .lock()
            .map_err(|_| AccountdError::Internal("supabase jwks cache poisoned".into()))
    }

    /// A cached key for `kid`, only while the set is younger than
    /// `JWKS_MAX_AGE_NS` of monotonic process uptime.
    fn fresh_cached_key(&self, kid: &str, mono_now_ns: u64) -> Result<Option<Jwk>> {
        let cache = self.lock_cache()?;
        let fresh = cache
            .last_success_mono_ns
            .is_some_and(|last| mono_now_ns.saturating_sub(last) < JWKS_MAX_AGE_NS);
        Ok(fresh
            .then(|| cache.keys.as_ref().and_then(|set| set.find(kid).cloned()))
            .flatten())
    }
}

fn jwk_algorithm(jwk: &Jwk) -> Result<Algorithm> {
    match jwk.common.key_algorithm {
        Some(KeyAlgorithm::ES256) => Ok(Algorithm::ES256),
        Some(KeyAlgorithm::RS256) => Ok(Algorithm::RS256),
        Some(KeyAlgorithm::HS256) => Err(AccountdError::Unauthorized("invalid supabase token")),
        None => match &jwk.algorithm {
            AlgorithmParameters::EllipticCurve(params) if params.curve == EllipticCurve::P256 => {
                Ok(Algorithm::ES256)
            }
            AlgorithmParameters::RSA(_) => Ok(Algorithm::RS256),
            _ => Err(AccountdError::Unauthorized("invalid supabase token")),
        },
        _ => Err(AccountdError::Unauthorized("invalid supabase token")),
    }
}

/// Claims accepted from a Supabase access token.
#[derive(Clone, Debug, Deserialize)]
pub struct SupabaseClaims {
    /// Stable Supabase user id.
    pub sub: String,
    /// Optional email attached to the identity.
    pub email: Option<String>,
    /// Unix timestamp in seconds.
    pub exp: u64,
    /// Unix timestamp in seconds.
    pub iat: u64,
    /// Exact issuer.
    pub iss: String,
    /// String or array audience, preserved for callers.
    pub aud: serde_json::Value,
    /// Supabase role, when present.
    pub role: Option<String>,
    /// Whether Supabase issued this token for an anonymous sign-in.
    #[serde(default)]
    pub is_anonymous: bool,
}

#[cfg(test)]
mod tests {
    use std::panic::{catch_unwind, AssertUnwindSafe};
    use std::sync::Arc;

    use jsonwebtoken::jwk::JwkSet;

    use super::{validate_jwks_url, StaticJwksSource, SupabaseConfig, SupabaseVerifier};
    use crate::AccountdError;

    #[test]
    fn derives_default_jwks_url_from_issuer() {
        let config = SupabaseConfig::from_env_values(
            Some("https://example.supabase.co/auth/v1/".to_string()),
            None,
            None,
            None,
        )
        .expect("valid configuration")
        .expect("issuer enables configuration");
        assert_eq!(config.audience, "authenticated");
        assert_eq!(
            config.jwks_url.as_deref(),
            Some("https://example.supabase.co/auth/v1/.well-known/jwks.json")
        );
    }

    #[test]
    fn empty_hs256_secret_is_treated_as_unset() {
        for secret in [Some(String::new()), Some("  \n".to_string())] {
            let config = SupabaseConfig::from_env_values(
                Some("https://example.supabase.co/auth/v1".to_string()),
                None,
                None,
                secret,
            )
            .expect("valid configuration")
            .expect("issuer enables configuration");
            assert_eq!(config.hs256_secret, None);
        }
    }

    #[test]
    fn blank_audience_defaults_to_authenticated() {
        for audience in [Some(String::new()), Some("  \n".to_string())] {
            let config = SupabaseConfig::from_env_values(
                Some("https://example.supabase.co/auth/v1".to_string()),
                audience,
                None,
                None,
            )
            .expect("valid configuration")
            .expect("issuer enables configuration");
            assert_eq!(config.audience, "authenticated");
        }
    }

    #[test]
    fn blank_jwks_url_defaults_to_derived_url() {
        for jwks_url in [Some(String::new()), Some("  \n".to_string())] {
            let config = SupabaseConfig::from_env_values(
                Some("https://example.supabase.co/auth/v1".to_string()),
                None,
                jwks_url,
                None,
            )
            .expect("valid configuration")
            .expect("issuer enables configuration");
            assert_eq!(
                config.jwks_url.as_deref(),
                Some("https://example.supabase.co/auth/v1/.well-known/jwks.json")
            );
        }
    }

    #[test]
    fn trims_surrounding_optional_supabase_values() {
        let config = SupabaseConfig::from_env_values(
            Some("https://example.supabase.co/auth/v1".to_string()),
            Some("  authenticated \n".to_string()),
            Some("  https://keys.example.test/jwks.json \n".to_string()),
            Some("  legacy-secret \n".to_string()),
        )
        .expect("valid configuration")
        .expect("issuer enables configuration");
        assert_eq!(config.audience, "authenticated");
        assert_eq!(
            config.jwks_url.as_deref(),
            Some("https://keys.example.test/jwks.json")
        );
        assert_eq!(config.hs256_secret.as_deref(), Some("legacy-secret"));
    }

    #[test]
    fn empty_issuer_leaves_supabase_unconfigured() {
        for issuer in [Some(String::new()), Some("  ".to_string())] {
            assert!(SupabaseConfig::from_env_values(issuer, None, None, None)
                .expect("empty issuer is valid unconfigured state")
                .is_none());
        }
    }

    #[test]
    fn trims_surrounding_issuer_whitespace() {
        let config = SupabaseConfig::from_env_values(
            Some("  https://x.supabase.co/auth/v1 \n".to_string()),
            None,
            None,
            None,
        )
        .expect("valid configuration")
        .expect("issuer enables configuration");

        assert_eq!(config.issuer, "https://x.supabase.co/auth/v1");
        assert_eq!(
            config.jwks_url.as_deref(),
            Some("https://x.supabase.co/auth/v1/.well-known/jwks.json")
        );
    }

    #[test]
    fn jwks_url_validation_accepts_https_and_loopback_http() {
        for url in [
            "https://example.supabase.co/auth/v1/.well-known/jwks.json",
            "http://localhost/.well-known/jwks.json",
            "http://127.0.0.1:8787/.well-known/jwks.json",
            "http://[::1]/.well-known/jwks.json",
        ] {
            assert!(validate_jwks_url(url).is_ok(), "accepted URL: {url}");
        }
    }

    #[test]
    fn jwks_url_validation_rejects_non_loopback_http() {
        let error =
            validate_jwks_url("http://example.com/keys").expect_err("HTTP must be loopback");
        assert_eq!(
            error.to_string(),
            "invalid configuration: supabase jwks url must use https (http is allowed for loopback only)"
        );
    }

    #[test]
    fn jwks_url_validation_rejects_non_http_schemes_and_unparsable_urls() {
        for url in ["ftp://example.com/keys", "://not a URL"] {
            assert!(validate_jwks_url(url).is_err(), "rejected URL: {url}");
        }
    }

    #[test]
    fn derived_non_loopback_http_jwks_url_is_rejected() {
        let result = SupabaseConfig::from_env_values(
            Some("http://example.com/auth/v1".to_string()),
            None,
            None,
            None,
        );
        assert_eq!(
            result
                .expect_err("derived non-loopback HTTP must be rejected")
                .to_string(),
            "invalid configuration: supabase jwks url must use https (http is allowed for loopback only)"
        );
    }

    #[test]
    fn jwks_url_validation_is_deterministic() {
        let first = validate_jwks_url("http://example.com/keys").map_err(|error| error.to_string());
        let second =
            validate_jwks_url("http://example.com/keys").map_err(|error| error.to_string());
        assert_eq!(first, second);
    }

    #[test]
    fn poisoned_jwks_cache_maps_to_internal() {
        let verifier = SupabaseVerifier::new(
            SupabaseConfig::from_env_values(
                Some("https://example.supabase.co/auth/v1".to_string()),
                None,
                None,
                None,
            )
            .expect("valid configuration")
            .expect("issuer enables configuration"),
            Arc::new(StaticJwksSource(JwkSet { keys: Vec::new() })),
        );

        let poisoned = catch_unwind(AssertUnwindSafe(|| {
            let _guard = verifier.cache.lock().expect("cache starts unpoisoned");
            panic!("poison the cache mutex for the error-mapping test");
        }));
        assert!(poisoned.is_err());

        let result = verifier.lock_cache();
        match result {
            Err(AccountdError::Internal(message)) => {
                assert_eq!(message, "supabase jwks cache poisoned")
            }
            Ok(_) => panic!("poisoned cache unexpectedly locked"),
            Err(error) => panic!("unexpected cache error: {error:?}"),
        };
    }
}
