// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion-netd` network configuration — the desk-default
//! zenoh config netd initializes its ONE session with.
//!
//! netd is network-CONFIGURED by default (gossip scouting on, zenoh multicast
//! scouting OFF) and every network-configured netd LISTENS: [`LISTEN_ENV`] when
//! set, else a synthesized `tcp/0.0.0.0:<port>` on the first free port in
//! [`SYNTH_LISTEN_BASE_PORT`]`..`[`SYNTH_LISTEN_BASE_PORT`]` + `[`SYNTH_LISTEN_PORT_SPAN`]
//! ([`ensure_listen_endpoint`]). `main.rs` opens the session at daemon
//! start and raises the `_cerulion._tcp` beacon on the bound port, so a peer
//! that boots later finds this machine in its own boot browse and dials it.
//! The synthesized range starts above 7683, the port a robot's
//! [`LISTEN_ENV`] or a graph's `network:` block conventionally binds.
//!
//! Peers that are already up come from the boot-time fold in
//! [`crate::discovery_fold`]: one bounded `_cerulion._tcp` mDNS browse plus the
//! `~/.cerulion/peers.json` cache, both TCP-probed, folded into the connect
//! endpoints before the session exists. With multicast scouting off the session
//! sends no UDP 7446 scout packets.
//! [`cerulion_core::transport::network::MULTICAST_SCOUTING_ENV`]`=on` turns
//! multicast scouting back on for a network where mDNS does not cross. The
//! [`CONNECT_ENV`]/[`LISTEN_ENV`] locators are the boot-time escape hatch for a
//! robot neither rung can reach (a different subnet). [`NETWORK_ENV`]`=off`
//! forces a strictly local-only netd (never any zenoh) for a locked-down network —
//! a remote demand then surfaces the explicit "not network-configured" residual.
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
/// (or unset) keeps the network-configured desk default. Mirrors the
/// `graph run` `CERULION_NETWORK=off` convention but scoped to the daemon so it
/// never surprises a graph run sharing the machine.
pub const NETWORK_ENV: &str = "CERULION_NETD_NETWORK";

/// The kill-switch value (`off`) that disables netd's network plane.
pub const NETWORK_OFF: &str = "off";

/// First port tried for the listen endpoint netd synthesizes when [`LISTEN_ENV`]
/// is unset.
pub const SYNTH_LISTEN_BASE_PORT: u16 = 7684;

/// How many consecutive ports, from [`SYNTH_LISTEN_BASE_PORT`], are tried.
pub const SYNTH_LISTEN_PORT_SPAN: u16 = 16;

/// The first port in `base..base+span` for which `is_free` holds. Pure.
pub fn first_free_port(base: u16, span: u16, is_free: impl Fn(u16) -> bool) -> Option<u16> {
    (0..span)
        .filter_map(|i| base.checked_add(i))
        .find(|p| is_free(*p))
}

/// Whether a TCP listener can bind `0.0.0.0:port` right now (the bind is
/// released before returning).
fn tcp_port_free(port: u16) -> bool {
    std::net::TcpListener::bind(("0.0.0.0", port)).is_ok()
}

/// The listen endpoint netd uses when [`LISTEN_ENV`] is unset:
/// `tcp/0.0.0.0:<port>` on the first port in [`SYNTH_LISTEN_BASE_PORT`]`..` +
/// [`SYNTH_LISTEN_PORT_SPAN`] that binds, or `None` when none does.
pub fn synthesized_listen_endpoint() -> Option<String> {
    first_free_port(
        SYNTH_LISTEN_BASE_PORT,
        SYNTH_LISTEN_PORT_SPAN,
        tcp_port_free,
    )
    .map(|p| format!("tcp/0.0.0.0:{p}"))
}

/// Give `cfg` a listen endpoint when it has none: push
/// [`synthesized_listen_endpoint`], or log a `warn!` and leave it listener-less
/// when no port in the range binds. `main.rs` calls it after the boot-time fold,
/// immediately before `TransportManager::init`, so the probe and zenoh's bind are
/// as close together as the start sequence allows.
pub fn ensure_listen_endpoint(cfg: &mut NetworkConfig) {
    if !cfg.listen_endpoints.is_empty() {
        return;
    }
    match synthesized_listen_endpoint() {
        Some(ep) => {
            tracing::info!(
                listen = %ep,
                "cerulion-netd: {LISTEN_ENV} unset; listening on a synthesized endpoint so a \
                 peer that boots later can find and dial this machine"
            );
            cfg.listen_endpoints.push(ep);
        }
        None => tracing::warn!(
            base = SYNTH_LISTEN_BASE_PORT,
            span = SYNTH_LISTEN_PORT_SPAN,
            "cerulion-netd: {LISTEN_ENV} unset and no port in the synthesized range is free; \
             this netd does not listen, so a peer that boots later cannot dial it"
        ),
    }
}

/// The [`LISTEN_ENV`] locators, parsed; empty when unset.
pub fn listen_env_locators() -> Vec<String> {
    std::env::var(LISTEN_ENV)
        .ok()
        .map(|s| parse_locators(&s))
        .unwrap_or_default()
}

/// Absolute path of the PEM trust anchors a `quic/` [`CONNECT_ENV`] locator's
/// peer certificate must chain to (zenoh `transport/link/tls/root_ca_certificate`).
pub const TLS_ROOT_CA_ENV: &str = "CERULION_NETD_TLS_ROOT_CA";

/// Absolute path of the PEM certificate a `quic/` [`LISTEN_ENV`] locator
/// presents (zenoh `transport/link/tls/listen_certificate`).
pub const TLS_CERTIFICATE_ENV: &str = "CERULION_NETD_TLS_CERTIFICATE";

/// Absolute path of the PEM private key for [`TLS_CERTIFICATE_ENV`] (zenoh
/// `transport/link/tls/listen_private_key`).
pub const TLS_PRIVATE_KEY_ENV: &str = "CERULION_NETD_TLS_PRIVATE_KEY";

/// Build netd's `quic/` certificate settings from the three env values: `None`
/// when all are unset or empty, else the set ones.
/// A wrong path surfaces when the session opens, naming the path. Pure,
/// oracle-tested.
pub fn tls_from_values(
    root_ca: Option<&str>,
    certificate: Option<&str>,
    private_key: Option<&str>,
) -> Option<cerulion_core::NetworkTls> {
    let path = |v: Option<&str>| {
        v.map(str::trim)
            .filter(|v| !v.is_empty())
            .map(std::path::PathBuf::from)
    };
    let tls = cerulion_core::NetworkTls {
        root_ca: path(root_ca),
        certificate: path(certificate),
        private_key: path(private_key),
    };
    (tls.root_ca.is_some() || tls.certificate.is_some() || tls.private_key.is_some()).then_some(tls)
}

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
/// gossip scouting on, multicast scouting OFF, the given connect/listen locators
/// folded in, everything else at [`NetworkConfig::default`] (peer mode,
/// `robot_identity: None` — an ingress-only desk announces nothing). Gossip
/// travels over links the session already has; it opens no UDP socket.
/// [`network_config_from_env`] sets `multicast_scouting` from
/// [`cerulion_core::transport::network::MULTICAST_SCOUTING_ENV`], and
/// [`ensure_listen_endpoint`] adds a listener when none is given. Pure —
/// oracle-tested.
pub fn desk_network_config(connect: Vec<String>, listen: Vec<String>) -> NetworkConfig {
    NetworkConfig {
        multicast_scouting: false,
        gossip_scouting: true,
        connect_endpoints: connect,
        listen_endpoints: listen,
        ..NetworkConfig::default()
    }
}

/// Whether this daemon is a STANDING one: it boots the embedded egress
/// gateway at daemon start and runs with idle self-exit disabled. Pure.
///
/// The gate is an EXPLICIT [`LISTEN_ENV`] (`explicit_listen`, the parsed env
/// value), not the config's listen endpoints: every network-configured netd
/// listens (a synthesized endpoint when the env is unset), and only the env
/// marks a machine that serves the network with nothing to respawn its daemon
/// (a pure-ROS robot). `None` (the [`NETWORK_ENV`]`=off` kill switch) wins.
pub fn standing_gateway_requested(cfg: Option<&NetworkConfig>, explicit_listen: &[String]) -> bool {
    cfg.is_some() && !explicit_listen.is_empty()
}

/// How the [`NETWORK_ENV`] value classifies netd's network plane. Pure —
/// separated from env reading so the byte-exact kill-switch decision (and the
/// foreseeable-mistake guard) is oracle-testable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetworkSwitch {
    /// Networking ON (env unset or empty) — the desk default.
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
/// breadcrumb, then no zenoh ever), else `Some(desk default)` with the
/// [`CONNECT_ENV`]/[`LISTEN_ENV`] locators folded in and `multicast_scouting` read
/// from [`cerulion_core::transport::network::MULTICAST_SCOUTING_ENV`]. A non-empty non-`off` value
/// (a foreseeable "off"-typo like `OFF`/`false`/`0`) keeps networking ON and warns
/// LOUDLY — never a silent "I thought I disabled it". The listen endpoint is
/// added afterwards by [`ensure_listen_endpoint`].
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
    let listen = listen_env_locators();
    if !connect.is_empty() || !listen.is_empty() {
        tracing::info!(
            ?connect,
            ?listen,
            "cerulion-netd: network locators from {CONNECT_ENV}/{LISTEN_ENV}"
        );
    }
    let mut cfg = desk_network_config(connect, listen);
    cfg.multicast_scouting = cerulion_core::transport::network::multicast_scouting_from_env();
    let env = |k: &str| std::env::var(k).ok();
    cfg.tls = tls_from_values(
        env(TLS_ROOT_CA_ENV).as_deref(),
        env(TLS_CERTIFICATE_ENV).as_deref(),
        env(TLS_PRIVATE_KEY_ENV).as_deref(),
    );
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
    use cerulion_core::transport::network::MULTICAST_SCOUTING_ENV;

    #[test]
    fn tls_from_values_is_none_when_unset_and_keeps_only_set_paths() {
        assert_eq!(tls_from_values(None, None, None), None);
        assert_eq!(tls_from_values(Some(""), Some("  "), None), None);
        let tls = tls_from_values(Some("/etc/c/ca.pem"), None, None).expect("root_ca set");
        assert_eq!(
            tls,
            cerulion_core::NetworkTls {
                root_ca: Some("/etc/c/ca.pem".into()),
                certificate: None,
                private_key: None,
            }
        );
        let tls = tls_from_values(None, Some("/c/me.pem"), Some("/c/me.key")).unwrap();
        assert_eq!(tls.certificate, Some("/c/me.pem".into()));
        assert_eq!(tls.private_key, Some("/c/me.key".into()));
        assert_eq!(tls.root_ca, None);
    }

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
    fn desk_network_config_is_gossip_only_peer_with_the_given_locators() {
        let cfg = desk_network_config(
            vec!["tcp/a:1".to_string()],
            vec!["tcp/0.0.0.0:7447".to_string()],
        );
        assert!(
            !cfg.multicast_scouting,
            "multicast scouting is OFF by default (no UDP 7446 scouts)"
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
        // An empty-locator desk default: gossip on, multicast off.
        let bare = desk_network_config(vec![], vec![]);
        assert!(!bare.multicast_scouting && bare.gossip_scouting);
        assert!(bare.connect_endpoints.is_empty() && bare.listen_endpoints.is_empty());
    }

    /// The standing-gateway gate is the LISTEN env: set ⇒ standing; unset ⇒ not
    /// standing even with a synthesized listener; LOCAL-ONLY (`None`) wins.
    /// Hand oracle.
    #[test]
    fn standing_gateway_requested_gates_on_the_listen_env_and_local_only_wins() {
        let env_listen = vec!["tcp/0.0.0.0:7683".to_string()];
        // A robot: LISTEN env set ⇒ standing.
        let robot = desk_network_config(vec![], env_listen.clone());
        assert!(standing_gateway_requested(Some(&robot), &env_listen));
        // A desk with a SYNTHESIZED listen endpoint (env unset) is not standing,
        // even though its config listens.
        let desk = desk_network_config(vec![], vec!["tcp/0.0.0.0:7684".to_string()]);
        assert!(!standing_gateway_requested(Some(&desk), &[]));
        // LOCAL-ONLY (None) wins whatever LISTEN said.
        assert!(!standing_gateway_requested(None, &env_listen));
    }

    /// The synthesized-port picker returns the first free port in range, skips
    /// taken ones, and gives up at the span's end. Hand oracle.
    #[test]
    fn first_free_port_scans_the_span_in_order() {
        assert_eq!(first_free_port(7684, 16, |_| true), Some(7684));
        assert_eq!(first_free_port(7684, 16, |p| p >= 7687), Some(7687));
        assert_eq!(first_free_port(7684, 16, |p| p == 7699), Some(7699));
        assert_eq!(first_free_port(7684, 16, |p| p == 7700), None);
        assert_eq!(first_free_port(7684, 0, |_| true), None);
        // No overflow past u16::MAX.
        assert_eq!(first_free_port(u16::MAX, 4, |_| true), Some(u16::MAX));
        assert_eq!(first_free_port(u16::MAX, 4, |p| p != u16::MAX), None);
    }

    /// `ensure_listen_endpoint` keeps a configured listener and synthesizes one
    /// only for a listener-less config.
    #[test]
    fn ensure_listen_endpoint_keeps_a_configured_listener() {
        let mut robot = desk_network_config(vec![], vec!["tcp/0.0.0.0:7683".to_string()]);
        ensure_listen_endpoint(&mut robot);
        assert_eq!(robot.listen_endpoints, vec!["tcp/0.0.0.0:7683".to_string()]);
        let mut desk = desk_network_config(vec![], vec![]);
        ensure_listen_endpoint(&mut desk);
        assert_eq!(desk.listen_endpoints.len(), 1);
    }

    /// The synthesized endpoint binds all IPv4 addresses on a port inside the
    /// range, above the 7683 a robot's LISTEN env or a graph block binds.
    #[test]
    fn synthesized_listen_endpoint_is_in_range_on_all_ipv4_addresses() {
        let ep = synthesized_listen_endpoint().expect("a free port in the range");
        let port: u16 = ep
            .strip_prefix("tcp/0.0.0.0:")
            .expect("tcp/0.0.0.0: form")
            .parse()
            .expect("numeric port");
        assert!(
            (SYNTH_LISTEN_BASE_PORT..SYNTH_LISTEN_BASE_PORT + SYNTH_LISTEN_PORT_SPAN)
                .contains(&port)
        );
        assert_ne!(port, 7683);
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

        let _gm = EnvVarGuard::unset(MULTICAST_SCOUTING_ENV);

        // (b) a mistyped "off" (e.g. "OFF") keeps networking ON (Some).
        {
            let _g = EnvVarGuard::set(NETWORK_ENV, "OFF");
            let cfg = network_config_from_env().expect("a mistyped off keeps networking ON");
            assert!(
                cfg.gossip_scouting && !cfg.multicast_scouting,
                "a mistyped off keeps the networked default (gossip on, multicast off)"
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
                cfg.gossip_scouting && !cfg.multicast_scouting,
                "locators leave the scouting posture unchanged"
            );
        }

        // (d) fully unset → no locators (the listen endpoint is synthesized
        // later, by `ensure_listen_endpoint`), gossip on, multicast off.
        {
            let _gn = EnvVarGuard::unset(NETWORK_ENV);
            let _gc = EnvVarGuard::unset(CONNECT_ENV);
            let _gl = EnvVarGuard::unset(LISTEN_ENV);
            let mut cfg = network_config_from_env().expect("unset default is network-configured");
            assert!(cfg.connect_endpoints.is_empty() && cfg.listen_endpoints.is_empty());
            ensure_listen_endpoint(&mut cfg);
            assert_eq!(cfg.listen_endpoints.len(), 1, "{:?}", cfg.listen_endpoints);
            assert!(cfg.listen_endpoints[0].starts_with("tcp/0.0.0.0:"));
            assert!(!cfg.multicast_scouting && cfg.gossip_scouting);
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

        // (f) the multicast switch: `on` enables it, anything else leaves it off.
        {
            let _gn = EnvVarGuard::unset(NETWORK_ENV);
            {
                let _g = EnvVarGuard::set(MULTICAST_SCOUTING_ENV, "on");
                let cfg = network_config_from_env().expect("configured");
                assert!(cfg.multicast_scouting && cfg.gossip_scouting);
            }
            for value in ["off", "", "true", "1"] {
                let _g = EnvVarGuard::set(MULTICAST_SCOUTING_ENV, value);
                let cfg = network_config_from_env().expect("configured");
                assert!(
                    !cfg.multicast_scouting,
                    "{MULTICAST_SCOUTING_ENV}={value:?} leaves multicast scouting off"
                );
            }
        }
    }
}
