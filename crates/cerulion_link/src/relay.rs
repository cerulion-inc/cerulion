// SPDX-License-Identifier: MIT OR Apache-2.0
//! The relay-configuration seam.
//!
//! iroh uses relay servers to assist connectivity (hole-punching coordination
//! and, as a fallback, relayed transport) and to power dial-by-key discovery.
//! This seam is DATA-DRIVEN: dev uses n0's public relays, prod points at
//! self-hosted relay URLs supplied by the caller — no prod URLs are hardcoded
//! here.

use std::str::FromStr;

use iroh::endpoint::{presets, Builder, RelayMode};
use iroh::RelayUrl;

use crate::error::BoxError;

/// The environment variable a robot / desk client reads for a self-hosted relay
/// URL override (mirrors the gateway's env-driven config surface). Consumed by
/// [`RelayConfig::resolve_from_env`].
pub const CERULION_RELAY_URL_ENV: &str = "CERULION_RELAY_URL";

/// A relay URL supplied by config / env could not be parsed. A LOUD typed error
/// — [`RelayConfig::resolve`] NEVER silently falls back to a default on a bad
/// URL (a mistyped self-hosted relay must fail closed, not silently route
/// through n0's public relays).
#[derive(Debug, thiserror::Error)]
pub enum RelayParseError {
    /// The supplied relay URL string is not a valid relay URL. The message
    /// interpolates the underlying `RelayUrl` parse reason (`{source}`) — the
    /// most actionable part for an operator with a mistyped relay — so a caller
    /// that renders only `Display` (not the `.source()` chain) still surfaces
    /// WHY the URL was rejected.
    #[error("relay URL '{url}' is not a valid relay URL: {source}")]
    BadUrl {
        /// The offending URL string.
        url: String,
        /// The underlying iroh `RelayUrl` parse error.
        #[source]
        source: BoxError,
    },
}

/// How an endpoint should reach the relay network.
#[derive(Debug, Clone, Default)]
pub enum RelayConfig {
    /// n0's public relays plus n0 discovery — the DEV default. Pure dial-by-key
    /// works with no configuration because n0 discovery resolves an [`EndpointId`] to
    /// its current addresses.
    ///
    /// [`EndpointId`]: iroh::EndpointId
    #[default]
    N0Default,

    /// No relay at all — direct / LAN-only connectivity. Dialing then REQUIRES
    /// an [`EndpointAddr`](iroh::EndpointAddr) carrying the peer's direct
    /// addresses (there is no discovery to resolve a bare key).
    Disabled,

    /// Self-hosted relay servers (PROD). The caller supplies the relay URLs;
    /// this crate hardcodes none.
    Custom(Vec<RelayUrl>),
}

impl RelayConfig {
    /// Apply this relay configuration to a freshly-preset endpoint builder.
    ///
    /// The preset is chosen per variant (`N0` bundles n0 relays + discovery;
    /// `Empty` starts from nothing), then the relay mode is set explicitly for
    /// the non-n0 variants.
    pub(crate) fn into_builder(self) -> Builder {
        match self {
            RelayConfig::N0Default => Builder::new(presets::N0),
            RelayConfig::Disabled => Builder::new(presets::Empty).relay_mode(RelayMode::Disabled),
            RelayConfig::Custom(urls) => {
                Builder::new(presets::Empty).relay_mode(RelayMode::custom(urls))
            }
        }
    }

    /// Resolve the relay posture from a disabled flag + an env-supplied URL + a
    /// config-file-supplied URL. The ONE parse helper shared by the robot
    /// (`cerulion_remoted`) and the desk client — neither duplicates
    /// the precedence logic. PURE (no I/O; the caller passes both URL strings in
    /// — see [`RelayConfig::resolve_from_env`] for the env-reading convenience).
    ///
    /// Precedence:
    /// 1. `disabled` wins outright → [`RelayConfig::Disabled`] (LAN / direct only).
    /// 2. else `env_url` > `config_url` (env overrides config) → the first
    ///    non-empty of the two becomes a [`RelayConfig::Custom`] single-URL relay.
    /// 3. else → [`RelayConfig::N0Default`] (the decision-1 launch
    ///    default: n0's public relays).
    ///
    /// An empty / whitespace-only URL reads as UNSET (an empty env var is not a
    /// URL). A present-but-malformed URL is a LOUD [`RelayParseError`], NEVER a
    /// silent fall-back to the default.
    pub fn resolve(
        disabled: bool,
        env_url: Option<&str>,
        config_url: Option<&str>,
    ) -> Result<RelayConfig, RelayParseError> {
        if disabled {
            return Ok(RelayConfig::Disabled);
        }
        // env > config; an empty/whitespace value reads as unset.
        let chosen = non_empty(env_url).or_else(|| non_empty(config_url));
        match chosen {
            None => Ok(RelayConfig::N0Default),
            Some(u) => {
                let url = RelayUrl::from_str(u).map_err(|e| RelayParseError::BadUrl {
                    url: u.to_string(),
                    source: Box::new(e),
                })?;
                Ok(RelayConfig::Custom(vec![url]))
            }
        }
    }

    /// [`RelayConfig::resolve`] with the env half read from
    /// [`CERULION_RELAY_URL_ENV`] — the convenience for a caller (the desk
    /// client) that has no clap `env`-bound flag already merging it. The robot's
    /// binary reads the env through its clap arg and calls [`RelayConfig::resolve`]
    /// directly (never double-reading the env).
    pub fn resolve_from_env(
        disabled: bool,
        config_url: Option<&str>,
    ) -> Result<RelayConfig, RelayParseError> {
        let env = std::env::var(CERULION_RELAY_URL_ENV).ok();
        Self::resolve(disabled, env.as_deref(), config_url)
    }
}

/// A URL string trimmed of surrounding whitespace, or `None` if it is
/// absent / empty / whitespace-only (an empty env var reads as unset).
fn non_empty(url: Option<&str>) -> Option<&str> {
    url.map(str::trim).filter(|s| !s.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Mutex, MutexGuard, OnceLock};

    const GOOD: &str = "https://relay.example.com";
    const OTHER: &str = "https://other-relay.example.com";

    /// Assert a [`RelayConfig::Custom`] carries EXACTLY one URL that parsed from
    /// `expected` (an oracle on the parsed relay, not a self-compare). Uses
    /// `starts_with` because the `url` crate normalizes an empty path to a
    /// trailing `/`; `expected` hosts are distinct (`relay` vs `other-relay`) so
    /// this still distinguishes env-vs-config precedence.
    fn assert_custom_single(cfg: &RelayConfig, expected: &str) {
        match cfg {
            RelayConfig::Custom(urls) => {
                assert_eq!(urls.len(), 1, "exactly one relay URL");
                assert!(
                    urls[0].to_string().starts_with(expected),
                    "parsed URL '{}' should start with '{expected}'",
                    urls[0]
                );
            }
            other => panic!("expected Custom, got {other:?}"),
        }
    }

    #[test]
    fn resolve_unset_is_n0_default() {
        // No disabled, no env, no config → the decision-1 launch default.
        assert!(matches!(
            RelayConfig::resolve(false, None, None).unwrap(),
            RelayConfig::N0Default
        ));
        // An empty (whitespace-only) env/config reads as unset, NOT a bad URL.
        assert!(matches!(
            RelayConfig::resolve(false, Some("   "), Some("")).unwrap(),
            RelayConfig::N0Default
        ));
    }

    #[test]
    fn resolve_disabled_wins_over_every_url() {
        // disabled wins even with a valid env AND config URL present.
        assert!(matches!(
            RelayConfig::resolve(true, Some(GOOD), Some(OTHER)).unwrap(),
            RelayConfig::Disabled
        ));
    }

    #[test]
    fn resolve_env_url_becomes_a_custom_single_relay() {
        let cfg = RelayConfig::resolve(false, Some(GOOD), None).unwrap();
        assert_custom_single(&cfg, GOOD);
    }

    #[test]
    fn resolve_config_url_is_honored_when_no_env() {
        let cfg = RelayConfig::resolve(false, None, Some(GOOD)).unwrap();
        assert_custom_single(&cfg, GOOD);
    }

    #[test]
    fn resolve_env_wins_over_config() {
        // Both present → env overrides config (the documented precedence).
        let cfg = RelayConfig::resolve(false, Some(GOOD), Some(OTHER)).unwrap();
        assert_custom_single(&cfg, GOOD);
    }

    #[test]
    fn resolve_bad_url_is_a_loud_typed_error_never_a_silent_default() {
        // A present-but-malformed URL fails closed with the offending string in
        // the message — NEVER a silent fall-back to N0Default.
        let err = RelayConfig::resolve(false, Some("::::not a url::::"), None).unwrap_err();
        assert!(matches!(err, RelayParseError::BadUrl { .. }));
        let msg = err.to_string();
        assert!(
            msg.contains("::::not a url::::"),
            "names the bad URL: {msg}"
        );
        // The underlying parse reason is interpolated after the colon (the
        // actionable "why" — M1 regression guard): the message carries MORE than
        // the bare top-level phrase.
        assert!(msg.contains("not a valid relay URL:"), "msg: {msg}");
        assert!(
            msg.len() > "relay URL '::::not a url::::' is not a valid relay URL:".len(),
            "the source parse reason must be appended, got: {msg}"
        );
    }

    // -- the env-reading convenience (process-global env → serialized) --------

    fn env_lock() -> MutexGuard<'static, ()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// RAII: restore `CERULION_RELAY_URL` to its pre-test value on drop.
    struct RelayEnvGuard(Option<String>);
    impl RelayEnvGuard {
        fn set(value: &str) -> Self {
            let prev = std::env::var(CERULION_RELAY_URL_ENV).ok();
            std::env::set_var(CERULION_RELAY_URL_ENV, value);
            RelayEnvGuard(prev)
        }
        fn unset() -> Self {
            let prev = std::env::var(CERULION_RELAY_URL_ENV).ok();
            std::env::remove_var(CERULION_RELAY_URL_ENV);
            RelayEnvGuard(prev)
        }
    }
    impl Drop for RelayEnvGuard {
        fn drop(&mut self) {
            match &self.0 {
                Some(v) => std::env::set_var(CERULION_RELAY_URL_ENV, v),
                None => std::env::remove_var(CERULION_RELAY_URL_ENV),
            }
        }
    }

    #[test]
    fn resolve_from_env_reads_the_relay_url_env_and_config_fallback() {
        let _lock = env_lock();

        // (a) env set → Custom from the env URL (env wins over the config arg).
        {
            let _g = RelayEnvGuard::set(GOOD);
            let cfg = RelayConfig::resolve_from_env(false, Some(OTHER)).unwrap();
            assert_custom_single(&cfg, GOOD);
        }

        // (b) env unset, config supplied → Custom from the config URL.
        {
            let _g = RelayEnvGuard::unset();
            let cfg = RelayConfig::resolve_from_env(false, Some(GOOD)).unwrap();
            assert_custom_single(&cfg, GOOD);
        }

        // (c) env unset, no config → N0Default.
        {
            let _g = RelayEnvGuard::unset();
            assert!(matches!(
                RelayConfig::resolve_from_env(false, None).unwrap(),
                RelayConfig::N0Default
            ));
        }

        // (d) a garbage env URL is still a loud typed error via the env path.
        {
            let _g = RelayEnvGuard::set("::::garbage::::");
            let err = RelayConfig::resolve_from_env(false, None).unwrap_err();
            assert!(matches!(err, RelayParseError::BadUrl { .. }));
        }
    }
}
