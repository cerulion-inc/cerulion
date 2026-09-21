// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-vizd` network configuration — the desk-default zenoh config the
//! daemon initializes its transport with (remote arm).
//!
//! The daemon is network-CONFIGURED by default (scouting ON — multicast + gossip)
//! but its zenoh session stays LAZY: `cerulion_core` opens no session until the
//! FIRST `register_ingress_topic` (a remote attach). So a purely-LOCAL daemon —
//! one that never sees a `--robot` attach — opens NO zenoh session at all,
//! preserving the "local-only by default" spirit while unlocking the remote
//! arm the moment a controller asks for a robot's topic. On a typical LAN scouting
//! finds the robot's gateway with ZERO config; the [`CONNECT_ENV`]/[`LISTEN_ENV`]
//! locators are the boot-time escape hatch for a robot scouting can't reach (a
//! different subnet, multicast disabled). [`NETWORK_ENV`]`=off` forces a strictly
//! local daemon (never any zenoh) for a locked-down network.
//!
//! The `cerulion viz` verb threads its `--connect`/`--listen` flags into these env
//! vars when it SPAWNS the daemon, so the locators are baked into the spawned
//! daemon's config. A daemon already running keeps its own boot-time locators (a
//! shared long-lived daemon's config is fixed at start — scouting covers the LAN
//! regardless); the verb says so.

use cerulion_core::transport::network::NetworkConfig;

/// Comma/whitespace-separated zenoh locators to CONNECT to (e.g.
/// `tcp/192.168.123.99:7683`). Threaded from `cerulion viz --connect` at spawn.
pub const CONNECT_ENV: &str = "CERULION_VIZD_CONNECT";

/// Comma/whitespace-separated zenoh locators to LISTEN on (e.g.
/// `tcp/0.0.0.0:7447`). Threaded from `cerulion viz --listen` at spawn.
pub const LISTEN_ENV: &str = "CERULION_VIZD_LISTEN";

/// Kill-switch: `CERULION_VIZD_NETWORK=off` forces a strictly LOCAL-ONLY daemon
/// (`network: None` → cerulion_core opens no zenoh session ever, and a remote
/// attach surfaces the explicit "not network-configured" residual). Any other value
/// (or unset) keeps the network-configured, scouting-on desk default. Mirrors the
/// `graph run` `CERULION_NETWORK=off` convention but scoped to the daemon so it
/// never surprises a graph run sharing the machine.
pub const NETWORK_ENV: &str = "CERULION_VIZD_NETWORK";

/// The kill-switch value (`off`) that disables the daemon's network plane.
pub const NETWORK_OFF: &str = "off";

/// Parse a `CONNECT`/`LISTEN` env value into a locator list: split on commas AND
/// ASCII whitespace, trim each, drop empties. Pure — oracle-tested. So
/// `"tcp/a:1, tcp/b:2"` and `"tcp/a:1 tcp/b:2"` both yield two locators, and an
/// empty / whitespace-only value yields none.
pub fn parse_locators(raw: &str) -> Vec<String> {
    raw.split([',', ' ', '\t', '\n', '\r'])
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .collect()
}

/// Build the daemon's desk-default [`NetworkConfig`] from explicit locator lists:
/// scouting ON (multicast + gossip — the automagic LAN-discovery default), the
/// given connect/listen locators folded in, everything else at
/// [`NetworkConfig::default`] (peer mode, `robot_identity: None` — an ingress-only
/// bridge announces nothing). The zenoh session this configures stays LAZY (opened
/// only on the first ingress register), so a purely-local daemon still opens
/// nothing. Pure — oracle-tested.
pub fn desk_network_config(connect: Vec<String>, listen: Vec<String>) -> NetworkConfig {
    NetworkConfig {
        multicast_scouting: true,
        gossip_scouting: true,
        connect_endpoints: connect,
        listen_endpoints: listen,
        ..NetworkConfig::default()
    }
}

/// How the [`NETWORK_ENV`] value classifies the daemon's network plane. Pure —
/// separated from env reading so the byte-exact kill-switch decision (and the
/// foreseeable-mistake guard) is oracle-testable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkSwitch {
    /// Networking ON (env unset or empty) — the scouting-on desk default.
    On,
    /// Networking OFF — the value is EXACTLY [`NETWORK_OFF`] (the kill switch).
    Off,
    /// Networking ON, but the value is a non-empty non-`off` string (e.g. `OFF`,
    /// `false`, `0`, `disabled`) — a foreseeable "I meant to turn it off" mistake.
    /// Networking stays ON and the caller warns LOUDLY (only exactly `off` disables).
    OnMistyped,
}

/// Classify a [`NETWORK_ENV`] value (byte-exact): only exactly [`NETWORK_OFF`]
/// disables; unset/empty is [`NetworkSwitch::On`]; any other non-empty value is
/// [`NetworkSwitch::OnMistyped`] (ON + a loud warn). Pure — oracle-tested.
pub fn classify_network_switch(value: Option<&str>) -> NetworkSwitch {
    match value {
        Some(v) if v == NETWORK_OFF => NetworkSwitch::Off,
        Some(v) if !v.is_empty() => NetworkSwitch::OnMistyped,
        _ => NetworkSwitch::On, // None or empty
    }
}

/// Resolve the daemon's network config from the process environment: `None` when
/// [`NETWORK_ENV`] is EXACTLY [`NETWORK_OFF`] (LOCAL-ONLY kill switch — a loud info
/// breadcrumb, then no zenoh ever), else `Some(scouting-on desk default)` with the
/// [`CONNECT_ENV`]/[`LISTEN_ENV`] locators folded in. A non-empty non-`off` value
/// (a foreseeable "off"-typo like `OFF`/`false`/`0`) keeps networking ON and warns
/// LOUDLY — never a silent "I thought I disabled it". A configured-but-unused
/// manager opens no session (the lazy-session invariant), so the common case
/// (never a remote attach) is byte-identical to a local-only daemon at runtime.
pub fn network_config_from_env() -> Option<NetworkConfig> {
    let raw = std::env::var(NETWORK_ENV).ok();
    match classify_network_switch(raw.as_deref()) {
        NetworkSwitch::Off => {
            tracing::info!(
                "cerulion-vizd: {NETWORK_ENV}={NETWORK_OFF} — LOCAL-ONLY (no zenoh session ever; \
                 remote `--robot` attach will surface the not-network-configured residual)"
            );
            return None;
        }
        NetworkSwitch::OnMistyped => {
            tracing::warn!(
                value = raw.as_deref().unwrap_or(""),
                "cerulion-vizd: {NETWORK_ENV} is set but NOT exactly \"off\" — the network stays \
                 ON (only the exact value \"off\" is the local-only kill switch). Set \
                 {NETWORK_ENV}=off to disable networking."
            );
        }
        NetworkSwitch::On => {}
    }
    let connect = std::env::var(CONNECT_ENV)
        .ok()
        .map(|s| parse_locators(&s))
        .unwrap_or_default();
    let listen = std::env::var(LISTEN_ENV)
        .ok()
        .map(|s| parse_locators(&s))
        .unwrap_or_default();
    if !connect.is_empty() || !listen.is_empty() {
        tracing::info!(
            ?connect,
            ?listen,
            "cerulion-vizd: network locators from {CONNECT_ENV}/{LISTEN_ENV} — scouting stays ON"
        );
    }
    Some(desk_network_config(connect, listen))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_locators_splits_on_commas_and_whitespace_and_drops_empties() {
        assert_eq!(parse_locators(""), Vec::<String>::new());
        assert_eq!(parse_locators("   "), Vec::<String>::new());
        assert_eq!(parse_locators(",  , "), Vec::<String>::new());
        assert_eq!(
            parse_locators("tcp/192.168.123.99:7683"),
            vec!["tcp/192.168.123.99:7683".to_string()]
        );
        // Comma-separated with surrounding spaces.
        assert_eq!(
            parse_locators("tcp/a:1, tcp/b:2 ,tcp/c:3"),
            vec![
                "tcp/a:1".to_string(),
                "tcp/b:2".to_string(),
                "tcp/c:3".to_string()
            ]
        );
        // Whitespace-separated (no commas) also works.
        assert_eq!(
            parse_locators("tcp/a:1\ttcp/b:2\ntcp/c:3"),
            vec![
                "tcp/a:1".to_string(),
                "tcp/b:2".to_string(),
                "tcp/c:3".to_string()
            ]
        );
    }

    #[test]
    fn desk_network_config_is_scouting_on_peer_with_the_given_locators() {
        let cfg = desk_network_config(
            vec!["tcp/a:1".to_string()],
            vec!["tcp/0.0.0.0:7447".to_string()],
        );
        assert!(
            cfg.multicast_scouting,
            "multicast scouting is ON by default"
        );
        assert!(cfg.gossip_scouting, "gossip scouting is ON by default");
        assert_eq!(cfg.connect_endpoints, vec!["tcp/a:1".to_string()]);
        assert_eq!(cfg.listen_endpoints, vec!["tcp/0.0.0.0:7447".to_string()]);
        // The default posture: peer mode, no announce identity (ingress-only).
        assert_eq!(cfg.mode, cerulion_core::transport::network::ZenohMode::Peer);
        assert!(
            cfg.robot_identity.is_none(),
            "an ingress-only desk announces nothing (robot_identity None)"
        );
        // An empty-locator desk default is still scouting-on peer (the pure-LAN
        // automagic path — no explicit locators needed).
        let bare = desk_network_config(vec![], vec![]);
        assert!(bare.multicast_scouting && bare.gossip_scouting);
        assert!(bare.connect_endpoints.is_empty() && bare.listen_endpoints.is_empty());
    }

    #[test]
    fn classify_network_switch_only_exact_off_disables() {
        // Only the byte-exact "off" is the kill switch.
        assert_eq!(classify_network_switch(Some("off")), NetworkSwitch::Off);
        // Unset / empty → ON.
        assert_eq!(classify_network_switch(None), NetworkSwitch::On);
        assert_eq!(classify_network_switch(Some("")), NetworkSwitch::On);
        // Foreseeable "I meant off" typos stay ON (loud) — NOT silently disabled.
        for mistyped in ["OFF", "Off", "false", "0", "no", "disabled", "off "] {
            assert_eq!(
                classify_network_switch(Some(mistyped)),
                NetworkSwitch::OnMistyped,
                "{mistyped:?} is not exactly \"off\" — network stays ON (loud)"
            );
        }
    }

    /// RAII env guard: restores the prior value (or removes) on drop, panic-safe.
    struct EnvVarGuard {
        key: &'static str,
        prev: Option<String>,
    }
    impl EnvVarGuard {
        fn set(key: &'static str, value: &str) -> Self {
            let prev = std::env::var(key).ok();
            std::env::set_var(key, value);
            Self { key, prev }
        }
        fn unset(key: &'static str) -> Self {
            let prev = std::env::var(key).ok();
            std::env::remove_var(key);
            Self { key, prev }
        }
    }
    impl Drop for EnvVarGuard {
        fn drop(&mut self) {
            match &self.prev {
                Some(v) => std::env::set_var(self.key, v),
                None => std::env::remove_var(self.key),
            }
        }
    }

    #[test]
    fn network_config_from_env_reads_the_kill_switch_and_locators() {
        // The env-READING path itself. Folded into ONE body
        // (with RAII guards) — no sibling test reads CERULION_VIZD_* env, so a
        // single mutate+restore body is race-free even under parallel libtest.

        // (a) exactly "off" → LOCAL-ONLY (None).
        {
            let _g = EnvVarGuard::set(NETWORK_ENV, NETWORK_OFF);
            assert!(
                network_config_from_env().is_none(),
                "CERULION_VIZD_NETWORK=off yields a network-less (None) config"
            );
        }

        // (b) a mistyped "off" (e.g. "OFF") keeps networking ON (Some).
        {
            let _g = EnvVarGuard::set(NETWORK_ENV, "OFF");
            let cfg = network_config_from_env().expect("a mistyped off keeps networking ON");
            assert!(
                cfg.multicast_scouting,
                "scouting stays on for a mistyped off"
            );
        }

        // (c) unset NETWORK + set CONNECT/LISTEN → the locators fold into the config.
        {
            let _gn = EnvVarGuard::unset(NETWORK_ENV);
            let _gc = EnvVarGuard::set(CONNECT_ENV, "tcp/10.0.0.1:7683,tcp/10.0.0.2:7447");
            let _gl = EnvVarGuard::set(LISTEN_ENV, "tcp/0.0.0.0:7447");
            let cfg = network_config_from_env().expect("default is network-configured");
            assert_eq!(
                cfg.connect_endpoints,
                vec![
                    "tcp/10.0.0.1:7683".to_string(),
                    "tcp/10.0.0.2:7447".to_string()
                ],
                "CONNECT env folds into connect_endpoints"
            );
            assert_eq!(
                cfg.listen_endpoints,
                vec!["tcp/0.0.0.0:7447".to_string()],
                "LISTEN env folds into listen_endpoints"
            );
            assert!(
                cfg.multicast_scouting && cfg.gossip_scouting,
                "scouting stays ON"
            );
        }

        // (d) fully unset → scouting-on default with NO locators (the pure-LAN path).
        {
            let _gn = EnvVarGuard::unset(NETWORK_ENV);
            let _gc = EnvVarGuard::unset(CONNECT_ENV);
            let _gl = EnvVarGuard::unset(LISTEN_ENV);
            let cfg = network_config_from_env().expect("unset default is network-configured");
            assert!(cfg.connect_endpoints.is_empty() && cfg.listen_endpoints.is_empty());
            assert!(cfg.multicast_scouting && cfg.gossip_scouting);
        }
    }
}
