// SPDX-License-Identifier: AGPL-3.0-only
//! Service configuration + the injectable clock seam.

use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc,
};
use std::time::{SystemTime, UNIX_EPOCH};

use crate::ca::CaConfig;
use crate::supabase::SupabaseConfig;

/// One second in nanoseconds.
pub const SEC_NS: u64 = 1_000_000_000;

/// The environment variable that sets [`ServiceConfig::refresh_reuse_window_ns`],
/// in whole seconds (`0` disables the window).
pub const REFRESH_REUSE_WINDOW_ENV: &str = "CERULION_ACCOUNTD_REFRESH_REUSE_WINDOW_S";

/// The default refresh-token reuse window: 60 s. It is at least one 30 s request
/// timeout (`cerulion_discovery::robot_state::ACCOUNT_HTTP_TIMEOUT`) plus the netd
/// revocation sync's first 5 s retry delay, so that worker's first retry after a
/// lost reply lands inside it; its later, doubling retries do not. An interactive
/// `cerulion` command does not retry; the next command is its retry.
pub const DEFAULT_REFRESH_REUSE_WINDOW_NS: u64 = 60 * SEC_NS;

/// The largest accepted refresh-token reuse window: 300 s, the longest wait the
/// netd revocation sync puts between two failed attempts. The window is how long
/// an already-rotated refresh token can still obtain the pair that replaced it.
pub const MAX_REFRESH_REUSE_WINDOW_NS: u64 = 300 * SEC_NS;

/// Parse [`REFRESH_REUSE_WINDOW_ENV`]: `None` (unset) is the 60 s default; a value
/// must be a whole number of seconds from 0 to 300. Anything else is an error that
/// names the variable, so a typo never silently falls back to the default.
pub fn parse_refresh_reuse_window(value: Option<&str>) -> Result<u64, String> {
    let Some(raw) = value else {
        return Ok(DEFAULT_REFRESH_REUSE_WINDOW_NS);
    };
    let max_s = MAX_REFRESH_REUSE_WINDOW_NS / SEC_NS;
    let digits = raw.trim();
    let not_seconds = || {
        format!(
            "{REFRESH_REUSE_WINDOW_ENV}={raw:?} is not a whole number of seconds (0 to {max_s})"
        )
    };
    if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err(not_seconds());
    }
    let secs: u64 = digits.parse().map_err(|_| not_seconds())?;
    if secs > max_s {
        return Err(format!(
            "{REFRESH_REUSE_WINDOW_ENV}={secs} is above the {max_s} s maximum"
        ));
    }
    Ok(secs * SEC_NS)
}

/// A monotone-enough wall clock, injectable so tests can pin `now_ns`
/// deterministically. The pure state machines take `now_ns` as a parameter and
/// are tested independently; this seam only feeds the DB-touching flows.
#[derive(Clone, Debug)]
pub enum Clock {
    /// Real wall time (Unix nanoseconds).
    System,
    /// A fixed time (tests).
    Fixed(u64),
    /// A shared deterministic time source (tests).
    Shared(Arc<AtomicU64>),
}

impl Clock {
    /// The current time in Unix nanoseconds.
    pub fn now_ns(&self) -> u64 {
        match self {
            Clock::System => SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            Clock::Fixed(ns) => *ns,
            Clock::Shared(ns) => ns.load(Ordering::SeqCst),
        }
    }
}

/// OAuth client credentials for one provider (Google / GitHub).
#[derive(Clone)]
pub struct ProviderCreds {
    /// The OAuth client id.
    pub client_id: String,
    /// The OAuth client secret (used at the token-exchange seam).
    pub client_secret: String,
    /// The redirect URI registered with the provider.
    pub redirect_uri: String,
}

impl std::fmt::Debug for ProviderCreds {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("ProviderCreds { client_secret: [REDACTED] }")
    }
}

/// OAuth provider configuration. A `None` provider is UNCONFIGURED — its start
/// endpoint refuses loudly (never a silent bypass, by design).
#[derive(Clone, Debug, Default)]
pub struct OAuthConfig {
    /// Google credentials, if configured.
    pub google: Option<ProviderCreds>,
    /// GitHub credentials, if configured.
    pub github: Option<ProviderCreds>,
}

/// The socket this daemon listens on unless `CERULION_ACCOUNTD_BIND` says
/// otherwise, and so the origin its own verification URIs default to.
///
/// Loopback is the whole point: `cerulion_accountd` is the local dev/test issuer,
/// and a literal that can only ever mean "this machine" names no deployment. It is
/// defined ONCE here so the listener and the URIs it prints cannot disagree — they
/// did when each carried its own copy.
pub const DEFAULT_BIND: &str = "127.0.0.1:8787";

/// The origin a daemon listening on `bind` should print in the device codes and
/// magic links it issues, when nothing configures one explicitly.
///
/// Derived rather than constant because the two can disagree: a daemon moved to
/// another port with `CERULION_ACCOUNTD_BIND` kept printing [`DEFAULT_BIND`],
/// so every code it issued named a socket nothing was listening on. A wildcard
/// bind is not an origin anyone can dial, so it resolves to the loopback of the
/// same family — the daemon is a local dev/test issuer, and a URI it prints is
/// for this machine's browser. Anything reachable from elsewhere is a
/// deployment fact this cannot know: set
/// `CERULION_ACCOUNTD_VERIFICATION_BASE_URI`.
pub fn default_verification_base_uri(bind: &str) -> String {
    let bind = bind.trim();
    // Split at the LAST colon: an IPv6 literal is full of them.
    let (host, port) = match bind.rsplit_once(':') {
        Some((h, p)) if !p.is_empty() && p.chars().all(|c| c.is_ascii_digit()) => (h, p),
        // Not a `host:port` at all — hand it back and let the listener fail on
        // the same string, rather than inventing a port it will not bind.
        _ => return format!("http://{bind}"),
    };
    let host = match host {
        "0.0.0.0" | "" => "127.0.0.1",
        "::" | "[::]" => "[::1]",
        other => other,
    };
    format!("http://{host}:{port}")
}

/// The account-service configuration.
#[derive(Clone, Debug)]
pub struct ServiceConfig {
    /// CA / issuance parameters.
    pub ca: CaConfig,
    /// Session-token lifetime (short — minutes).
    pub session_ttl_ns: u64,
    /// Refresh-token lifetime (days).
    pub refresh_ttl_ns: u64,
    /// How long after a rotation the refresh token it replaced is still accepted,
    /// once, to return the pair that rotation issued (`0` disables it). Set by
    /// [`REFRESH_REUSE_WINDOW_ENV`]; the default and bounds are
    /// [`DEFAULT_REFRESH_REUSE_WINDOW_NS`] and [`MAX_REFRESH_REUSE_WINDOW_NS`].
    pub refresh_reuse_window_ns: u64,
    /// Device-code lifetime (RFC 8628 `expires_in`).
    pub device_code_ttl_ns: u64,
    /// Minimum poll interval in seconds (RFC 8628 `interval`; `0` disables the
    /// slow-down gate — used in tests).
    pub device_code_interval_secs: u64,
    /// Magic-link lifetime.
    pub magic_link_ttl_ns: u64,
    /// OAuth authorization-flow (`state` / PKCE) lifetime.
    pub oauth_flow_ttl_ns: u64,
    /// Proof-of-possession challenge lifetime. Short — a device signs
    /// and spends the challenge in one round-trip, so a tight window bounds a stolen
    /// challenge without inconveniencing the legitimate flow.
    pub pop_challenge_ttl_ns: u64,
    /// The public base URI the device-flow verification page + magic links live
    /// under. Defaults to [`DEFAULT_BIND`] — the loopback socket the daemon itself
    /// listens on — because this daemon is a local dev/test issuer; production
    /// sign-in is the hosted service
    /// `cerulion_cli_engine::login_cmd::DEFAULT_ACCOUNT_SERVICE` names. Set
    /// `CERULION_ACCOUNTD_VERIFICATION_BASE_URI` when it is reachable under some
    /// other origin, or the codes it prints point at the wrong host.
    pub verification_base_uri: String,
    /// OAuth provider credentials.
    pub oauth: OAuthConfig,
    /// Supabase Auth configuration. `None` refuses Supabase exchange loudly.
    pub supabase: Option<SupabaseConfig>,
}

impl Default for ServiceConfig {
    fn default() -> Self {
        ServiceConfig {
            ca: CaConfig::default(),
            session_ttl_ns: 15 * 60 * SEC_NS,        // 15 minutes
            refresh_ttl_ns: 30 * 24 * 3600 * SEC_NS, // 30 days
            refresh_reuse_window_ns: DEFAULT_REFRESH_REUSE_WINDOW_NS,
            device_code_ttl_ns: 10 * 60 * SEC_NS, // 10 minutes
            device_code_interval_secs: 5,
            magic_link_ttl_ns: 15 * 60 * SEC_NS,   // 15 minutes
            oauth_flow_ttl_ns: 15 * 60 * SEC_NS,   // 15 minutes
            pop_challenge_ttl_ns: 5 * 60 * SEC_NS, // 5 minutes
            verification_base_uri: format!("http://{DEFAULT_BIND}"),
            oauth: OAuthConfig::default(),
            supabase: None,
        }
    }
}

impl ServiceConfig {
    /// The RFC 8628 device-flow verification URI (`{base}/v1/auth/device`).
    ///
    /// This daemon serves no browser page there: the path holds `POST .../start` and
    /// `POST .../poll` only, and a code is authorized through the magic-link or OAuth
    /// endpoints. The URI names where an approval surface fronting this daemon lives,
    /// which is why `CERULION_ACCOUNTD_VERIFICATION_BASE_URI` overrides it.
    pub fn verification_uri(&self) -> String {
        format!(
            "{}/v1/auth/device",
            self.verification_base_uri.trim_end_matches('/')
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provider_credentials_debug_redacts_direct_and_nested_secrets() {
        let secret = "oauth-secret-oracle-9d61b19d";
        let credentials = ProviderCreds {
            client_id: "public-client-id".into(),
            client_secret: secret.into(),
            redirect_uri: "https://example.com/callback".into(),
        };
        let oauth = OAuthConfig {
            google: Some(credentials.clone()),
            github: Some(credentials.clone()),
        };
        let service = ServiceConfig {
            oauth: oauth.clone(),
            ..ServiceConfig::default()
        };
        assert_eq!(
            format!("{credentials:?}"),
            "ProviderCreds { client_secret: [REDACTED] }"
        );
        for output in [
            format!("{credentials:?}"),
            format!("{credentials:#?}"),
            format!("{oauth:?}"),
            format!("{oauth:#?}"),
            format!("{service:?}"),
            format!("{service:#?}"),
        ] {
            assert!(output.contains("client_secret: [REDACTED]"));
            assert!(!output.contains(secret));
        }
    }

    #[test]
    fn the_refresh_reuse_window_parses_seconds_within_its_bounds() {
        assert_eq!(parse_refresh_reuse_window(None), Ok(60_000_000_000));
        assert_eq!(
            ServiceConfig::default().refresh_reuse_window_ns,
            60_000_000_000
        );
        assert_eq!(parse_refresh_reuse_window(Some("0")), Ok(0));
        assert_eq!(parse_refresh_reuse_window(Some(" 45 ")), Ok(45_000_000_000));
        assert_eq!(parse_refresh_reuse_window(Some("300")), Ok(300_000_000_000));
        for bad in ["301", "-1", "+60", "1.5", "", "60s", "18446744073709551616"] {
            let err = parse_refresh_reuse_window(Some(bad)).unwrap_err();
            assert!(err.contains(REFRESH_REUSE_WINDOW_ENV), "{bad:?}: {err}");
        }
    }

    #[test]
    fn the_default_origin_follows_the_port_the_daemon_was_moved_to() {
        assert_eq!(
            default_verification_base_uri("127.0.0.1:9999"),
            "http://127.0.0.1:9999"
        );
        assert_eq!(
            default_verification_base_uri(DEFAULT_BIND),
            format!("http://{DEFAULT_BIND}")
        );
    }

    /// `:0` asks the KERNEL for a port, so the requested address never names the
    /// one served: a URI built from it says `:0`, which nothing answers. The
    /// listener's own address is the only statement of where the daemon is, which
    /// is why `cerulion-accountd` binds before it builds this config.
    #[test]
    fn an_ephemeral_bind_is_only_dialable_through_the_listeners_own_address() {
        assert_eq!(
            default_verification_base_uri("127.0.0.1:0"),
            "http://127.0.0.1:0",
            "the requested address cannot know the port; this is the unusable URI \
             the daemon must never print"
        );

        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        assert_ne!(addr.port(), 0, "a bound socket has a real port");
        assert_eq!(
            default_verification_base_uri(&addr.to_string()),
            format!("http://127.0.0.1:{}", addr.port())
        );
    }

    #[test]
    fn a_wildcard_bind_becomes_the_loopback_of_its_own_family() {
        assert_eq!(
            default_verification_base_uri("0.0.0.0:8787"),
            "http://127.0.0.1:8787"
        );
        assert_eq!(
            default_verification_base_uri("[::]:8787"),
            "http://[::1]:8787"
        );
        assert_eq!(
            default_verification_base_uri(":8787"),
            "http://127.0.0.1:8787"
        );
    }

    #[test]
    fn a_named_host_is_kept_verbatim() {
        assert_eq!(
            default_verification_base_uri("192.168.1.4:8080"),
            "http://192.168.1.4:8080"
        );
        assert_eq!(
            default_verification_base_uri("[2001:db8::1]:8080"),
            "http://[2001:db8::1]:8080"
        );
    }

    #[test]
    fn a_bind_that_is_not_host_port_is_handed_back_rather_than_repaired() {
        // The listener fails on the same string; inventing a port here would
        // print a URI for a socket nothing ever bound.
        assert_eq!(default_verification_base_uri("nonsense"), "http://nonsense");
        assert_eq!(
            default_verification_base_uri("127.0.0.1:http"),
            "http://127.0.0.1:http"
        );
    }

    #[test]
    fn the_verification_uri_is_the_base_plus_the_rfc_8628_path() {
        let mut config = ServiceConfig {
            verification_base_uri: "http://127.0.0.1:9000/".into(),
            ..ServiceConfig::default()
        };
        assert_eq!(
            config.verification_uri(),
            "http://127.0.0.1:9000/v1/auth/device"
        );
        config.verification_base_uri = default_verification_base_uri("0.0.0.0:9000");
        assert_eq!(
            config.verification_uri(),
            "http://127.0.0.1:9000/v1/auth/device"
        );
    }
}
