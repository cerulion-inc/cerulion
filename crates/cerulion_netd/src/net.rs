// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-netd` network configuration — the desk-default
//! zenoh config netd initializes its ONE session with.
//!
//! netd is network-CONFIGURED by default (scouting ON — multicast + gossip) but
//! its zenoh session stays LAZY: `cerulion_core` opens no session until the FIRST
//! `register_ingress_topic` (the first remote-topic demand). So a netd spawned
//! but never demanded opens NO zenoh session at all — and then self-exits on idle
//! (`daemon`), leaving nothing behind. On a typical LAN scouting finds the robot's
//! gateway with ZERO config; the [`CONNECT_ENV`]/[`LISTEN_ENV`] locators are the
//! boot-time escape hatch for a robot scouting can't reach (a different subnet,
//! multicast disabled). [`NETWORK_ENV`]`=off` forces a strictly local-only netd
//! (never any zenoh) for a locked-down network — a remote demand then surfaces the
//! explicit "not network-configured" residual.
//!
//! netd is the ONE gateway per computer. An ingress-only desk would need NO robot identity
//! (`robot_identity: None`) — it consumes, it does not produce.
//! But EGRESS converges into netd too: a desk graph pushes its produced-topic
//! plan into the shared session, so netd DOES need an announce identity. Under the
//! rule "hostname IS robot identity", [`network_config_from_env`]
//! stamps this machine's identity (the hostname, via
//! [`robot_identity_from_env`](cerulion_core::graph::robot_identity_from_env)) into
//! the shared `NetworkConfig` at init.
//!
//! **The design tension:** `robot_identity` is immutable at
//! `TransportManager::init` and the session is SHARED (Principle #8), so a truly
//! "minted only on the first egress plan" identity is infeasible without making the
//! config mutable post-init. So
//! the identity is CARRIED from init. This is byte-identical for a pure-ingress desk
//! at the ANNOUNCE level — nothing announces until [`crate::egress::GatewayEgressPlane`]
//! boots the embedded gateway on the first egress plan (the announce-level
//! activation IS lazy). The ONE pre-egress effect of carrying the identity is the
//! demand-GET self-demand filter (`network.rs`), which skips a REMOTE
//! robot whose announce identity string equals this desk's hostname — a pathological
//! same-hostname collision (two machines with one hostname on a LAN is itself
//! broken), and arguably the more-correct behavior. Every other ingress-path
//! primitive (`register_ingress_topic` / provenance / keepalive / `ingress_stats`)
//! is identity-independent.

use cerulion_core::transport::network::NetworkConfig;

/// Comma/whitespace-separated zenoh locators to CONNECT to (e.g.
/// `tcp/192.168.123.99:7683`). Threaded from a consumer's `--connect` at spawn.
pub const CONNECT_ENV: &str = "CERULION_NETD_CONNECT";

/// Comma/whitespace-separated zenoh locators to LISTEN on (e.g.
/// `tcp/0.0.0.0:7447`). Threaded from a consumer's `--listen` at spawn.
pub const LISTEN_ENV: &str = "CERULION_NETD_LISTEN";

/// Kill-switch: `CERULION_NETD_NETWORK=off` forces a strictly LOCAL-ONLY netd
/// (`network: None` → cerulion_core opens no zenoh session ever, and a remote
/// demand surfaces the explicit "not network-configured" residual). Any other value
/// (or unset) keeps the network-configured, scouting-on desk default. Mirrors the
/// `graph run` `CERULION_NETWORK=off` convention but scoped to the daemon so it
/// never surprises a graph run sharing the machine.
pub const NETWORK_ENV: &str = "CERULION_NETD_NETWORK";

/// The kill-switch value (`off`) that disables netd's network plane.
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

/// Build netd's desk-default [`NetworkConfig`] from explicit locator lists:
/// scouting ON (multicast + gossip — the automagic LAN-discovery default), the
/// given connect/listen locators folded in, everything else at
/// [`NetworkConfig::default`] (peer mode, `robot_identity: None` — an ingress-only
/// desk announces nothing). The zenoh session this configures stays LAZY (opened
/// only on the first ingress register), so a never-demanded netd still opens
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

/// Whether this daemon should boot the embedded egress gateway AT
/// DAEMON START — the STANDING gateway of a machine that SERVES the network.
///
/// The gate is the LISTEN endpoint (`cfg.listen_endpoints`, i.e. [`LISTEN_ENV`])
/// — deliberately the SAME gate as the mDNS beacon, never robot identity:
/// identity is stamped on every machine, so only a listen locator distinguishes
/// a robot from a desk. `None` (the [`NETWORK_ENV`]`=off` kill switch) wins —
/// a LOCAL-ONLY daemon boots nothing whatever [`LISTEN_ENV`] says, because there
/// is no session to open. Pure — oracle-tested.
///
/// A `true` here also makes the daemon a STANDING one (idle self-exit disabled —
/// see `NetdConfig::idle_self_exit`): the gateway + beacon are the machine's
/// network presence, and an idle self-exit would withdraw them with nothing on a
/// pure-ROS robot to respawn the daemon (netd is spawned by LOCAL consumers,
/// which such a robot has none of).
pub fn standing_gateway_requested(cfg: Option<&NetworkConfig>) -> bool {
    cfg.is_some_and(|c| !c.listen_endpoints.is_empty())
}

/// How the [`NETWORK_ENV`] value classifies netd's network plane. Pure —
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

/// Resolve netd's network config from the process environment: `None` when
/// [`NETWORK_ENV`] is EXACTLY [`NETWORK_OFF`] (LOCAL-ONLY kill switch — a loud info
/// breadcrumb, then no zenoh ever), else `Some(scouting-on desk default)` with the
/// [`CONNECT_ENV`]/[`LISTEN_ENV`] locators folded in. A non-empty non-`off` value
/// (a foreseeable "off"-typo like `OFF`/`false`/`0`) keeps networking ON and warns
/// LOUDLY — never a silent "I thought I disabled it". A configured-but-unused
/// manager opens no session (the lazy-session invariant), so a never-demanded netd
/// is byte-identical to a local-only one at runtime.
pub fn network_config_from_env() -> Option<NetworkConfig> {
    let raw = std::env::var(NETWORK_ENV).ok();
    match classify_network_switch(raw.as_deref()) {
        NetworkSwitch::Off => {
            tracing::info!(
                "cerulion-netd: {NETWORK_ENV}={NETWORK_OFF} — LOCAL-ONLY (no zenoh session ever; \
                 a remote demand will surface the not-network-configured residual)"
            );
            return None;
        }
        NetworkSwitch::OnMistyped => {
            tracing::warn!(
                value = raw.as_deref().unwrap_or(""),
                "cerulion-netd: {NETWORK_ENV} is set but NOT exactly \"off\" — the network stays \
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
            "cerulion-netd: network locators from {CONNECT_ENV}/{LISTEN_ENV} — scouting stays ON"
        );
    }
    let mut cfg = desk_network_config(connect, listen);
    // Stamp THIS machine's identity (hostname, `CERULION_ROBOT_IDENTITY`
    // override honored) so the egress plane's embedded gateway can announce the
    // desk's produced topics. Immutable-at-init + shared-session ⇒ carried from init;
    // a pure-ingress desk announces nothing until the gateway boots (see the module
    // docs for the design tension). ONE identity resolver across every plane
    // (cerulion_core::graph::robot_identity_from_env — the same one the LAN gateway +
    // the remote wire plane use), so the desk never self-attributes under two names.
    cfg.robot_identity = Some(cerulion_core::graph::robot_identity_from_env());
    Some(cfg)
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
        // The default posture: peer mode, no announce identity (ingress-only —
        // netd is the desk consumer, not a data source).
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

    /// The standing-gateway gate is the LISTEN endpoint: set ⇒
    /// boot at start; unset ⇒ desk (lazy boot only); LOCAL-ONLY (`None`) wins
    /// even with a listen locator configured somewhere. Hand oracle.
    #[test]
    fn standing_gateway_requested_gates_on_listen_and_local_only_wins() {
        // A robot: a listen locator ⇒ standing gateway at start.
        let robot = desk_network_config(vec![], vec!["tcp/0.0.0.0:7683".to_string()]);
        assert!(standing_gateway_requested(Some(&robot)));
        // A desk: no listen locator ⇒ NO start boot (the original lazy path),
        // even with connect locators (dialing out is not serving).
        let desk = desk_network_config(vec!["tcp/10.0.0.9:7683".to_string()], vec![]);
        assert!(!standing_gateway_requested(Some(&desk)));
        // LOCAL-ONLY (the NETWORK_ENV=off kill switch resolves the config to
        // None): nothing boots, whatever LISTEN said.
        assert!(!standing_gateway_requested(None));
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
        // The env-READING path. Folded into ONE body (with RAII guards) so the
        // mutate+restore windows never interleave with each other.
        //
        // It ALSO takes the CRATE-WIDE env lock. The argument
        // "no sibling test reads CERULION_NETD_* env, so a single mutate+restore
        // body is race-free even under parallel libtest" reasons about VAR NAMES, and
        // that is not the hazard: `wan.rs` mutates 30 env vars in this same test binary,
        // and concurrent `setenv`/`getenv` from different threads races on the environ
        // block whatever the keys are. (This body also touches
        // `CERULION_ROBOT_IDENTITY`, which is not a `CERULION_NETD_*` var at all.)
        let _lock = crate::test_env::env_lock();

        // (a) exactly "off" → LOCAL-ONLY (None).
        {
            let _g = EnvVarGuard::set(NETWORK_ENV, NETWORK_OFF);
            assert!(
                network_config_from_env().is_none(),
                "CERULION_NETD_NETWORK=off yields a network-less (None) config"
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
            // The production env config carries this machine's identity
            // (hostname) so the egress plane's gateway can announce — UNLIKE the bare
            // `desk_network_config` builder, which stays identity-free.
            assert!(
                cfg.robot_identity.is_some(),
                "network_config_from_env stamps the machine identity (egress needs it)"
            );
        }

        // (e) an explicit CERULION_ROBOT_IDENTITY override is honored verbatim.
        {
            let _gn = EnvVarGuard::unset(NETWORK_ENV);
            let _gi =
                EnvVarGuard::set(cerulion_core::graph::ROBOT_IDENTITY_ENV, "desk-override-42");
            let cfg = network_config_from_env().expect("configured");
            assert_eq!(cfg.robot_identity.as_deref(), Some("desk-override-42"));
        }
    }
}
