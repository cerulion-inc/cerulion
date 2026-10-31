// SPDX-License-Identifier: AGPL-3.0-only
//! The `graph run` warning for egress on `tcp/` from a Linux host that resets
//! a TCP connection's congestion window after idle.
//!
//! Mechanism: with `net.ipv4.tcp_slow_start_after_idle` nonzero (the Linux
//! default), the congestion window of a TCP connection that sends nothing for
//! longer than one retransmission timeout decays toward its initial size,
//! halving once per timeout elapsed (`tcp_cwnd_restart` in the kernel's
//! `net/ipv4/tcp_output.c`). A large message sent at a low rate then needs
//! several more round trips each time, so the cost grows with the path's
//! round-trip time. The window belongs to the SENDING side, so the check runs
//! only once this run has egress on a live route. The `quic/` carrier's
//! congestion controller (quinn) has no idle reset.
//!
//! `warn_if_tcp_egress_idle_resets` is called from the egress setup in
//! `graph_cmd` once a route carries this run's egress:
//!
//! - the per-run gateway child, which a `network:` block (or an unreachable
//!   netd) starts. Its locators are this run's own, so the line names the
//!   `tcp/` ones and is a WARN.
//! - the shared `cerulion-netd` session, which every networked run on the
//!   machine registers with whether or not any remote peer subscribes. The
//!   line is an INFO there, so an ordinary run with no remote subscriber is not
//!   flagged. netd dials discovered peers as `tcp/`; a netd started with only
//!   `quic/` locators is the case this check cannot see.
//!
//! The sysctl is per network namespace and is read in this process's
//! namespace, which the per-run gateway child inherits; netd shares it when
//! this run spawned netd or when both run in the host namespace.

/// The sysctl file this check reads.
pub const SLOW_START_AFTER_IDLE_SYSCTL: &str = "/proc/sys/net/ipv4/tcp_slow_start_after_idle";

/// The warning text. `pub` so tests pin the line by identity.
pub const TCP_SLOW_START_AFTER_IDLE_WARNING: &str = "this run sends over tcp/ from a host where net.ipv4.tcp_slow_start_after_idle is nonzero: the congestion window of a TCP connection that sat idle for longer than one retransmission timeout decays toward its initial size, so each large message sent at a low rate pays a few extra round trips, a cost that grows with the path's round-trip time. Measured 2026-10-26 over a 31 ms wide-area path with a 301 KB message once per second: tcp/ 164.8 ms p50, quic/ 79.3 ms p50 (at 30 Hz both were about 63 ms). Fix: set `sysctl -w net.ipv4.tcp_slow_start_after_idle=0` on the sending host and keep it in a file under /etc/sysctl.d/, or move both ends to quic/ (a network: block with quic/ locators, their tls: files and an egress: list; see docs/networking.md, Carriers and per-topic reliability).";

/// The route that carries this run's egress.
#[derive(Debug, Clone, Copy)]
pub(crate) enum EgressRoute<'a> {
    /// The per-run gateway child, on this run's own network config.
    Child(&'a cerulion_core::NetworkConfig),
    /// The shared `cerulion-netd` session.
    Netd,
}

/// The `tcp/` locators among `net`'s listen and connect endpoints, in that
/// order.
pub(crate) fn tcp_locators(net: &cerulion_core::NetworkConfig) -> Vec<String> {
    net.listen_endpoints
        .iter()
        .chain(net.connect_endpoints.iter())
        .filter(|l| l.trim_start().starts_with("tcp/"))
        .cloned()
        .collect()
}

/// Whether the sysctl text turns the idle reset on. The kernel treats any
/// nonzero integer as on. `None` for text that is not an integer.
pub(crate) fn idle_reset_enabled(sysctl_text: &str) -> Option<bool> {
    sysctl_text.trim().parse::<i64>().ok().map(|v| v != 0)
}

/// Whether `plan` sends anything off this machine: it announces at least one
/// topic and its egress policy is not an empty allow-list. An ingress-only
/// gateway only receives, and the idle reset belongs to the sending side.
pub(crate) fn plan_has_egress(plan: &cerulion_core::GatewayPlan) -> bool {
    let policy_allows = match &plan.egress_policy {
        cerulion_core::GatewayEgressPolicy::AllowAll => true,
        cerulion_core::GatewayEgressPolicy::AllowList(topics) => !topics.is_empty(),
    };
    policy_allows && !plan.announce.is_empty()
}

/// What a warning reports, when one is due.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum IdleWarning {
    /// The per-run gateway child listens or dials on these `tcp/` locators.
    Child(Vec<String>),
    /// The egress rides the shared `cerulion-netd` session.
    Netd,
}

/// The decision, with the host read injected. `sysctl_text` is `None` where
/// the file does not exist or cannot be read (every non-Linux host), which
/// never warns; so does a value of 0 or text that is not an integer, a plan
/// with no egress, and a child route with no `tcp/` locator.
pub(crate) fn idle_warning(
    route: EgressRoute<'_>,
    plan: &cerulion_core::GatewayPlan,
    sysctl_text: Option<&str>,
) -> Option<IdleWarning> {
    if !plan_has_egress(plan) || idle_reset_enabled(sysctl_text?) != Some(true) {
        return None;
    }
    match route {
        EgressRoute::Child(net) => {
            let locators = tcp_locators(net);
            (!locators.is_empty()).then_some(IdleWarning::Child(locators))
        }
        EgressRoute::Netd => Some(IdleWarning::Netd),
    }
}

/// Read [`SLOW_START_AFTER_IDLE_SYSCTL`]. `None` when it cannot be read; on a
/// non-Linux host the file does not exist.
fn read_sysctl() -> Option<String> {
    if !cfg!(target_os = "linux") {
        return None;
    }
    match std::fs::read_to_string(SLOW_START_AFTER_IDLE_SYSCTL) {
        Ok(text) => Some(text),
        Err(err) => {
            tracing::debug!(
                path = SLOW_START_AFTER_IDLE_SYSCTL,
                error = %err,
                "could not read the tcp idle-reset sysctl; skipping the tcp/ idle check"
            );
            None
        }
    }
}

/// Log [`TCP_SLOW_START_AFTER_IDLE_WARNING`] once for this run's egress
/// route when it is due: WARN on the per-run gateway route, INFO on the
/// netd route (see the module docs). Returns whether it logged.
pub(crate) fn warn_if_tcp_egress_idle_resets(
    graph_name: &str,
    route: EgressRoute<'_>,
    plan: &cerulion_core::GatewayPlan,
) -> bool {
    let text = read_sysctl();
    let value = text
        .as_deref()
        .map(str::trim)
        .unwrap_or_default()
        .to_string();
    match idle_warning(route, plan, text.as_deref()) {
        Some(IdleWarning::Child(locators)) => {
            tracing::warn!(
                graph = %graph_name,
                route = "per-run gateway",
                tcp_locators = %locators.join(","),
                sysctl = SLOW_START_AFTER_IDLE_SYSCTL,
                value = %value,
                "{TCP_SLOW_START_AFTER_IDLE_WARNING}"
            );
            true
        }
        Some(IdleWarning::Netd) => {
            tracing::info!(
                graph = %graph_name,
                route = "cerulion-netd",
                sysctl = SLOW_START_AFTER_IDLE_SYSCTL,
                value = %value,
                "{TCP_SLOW_START_AFTER_IDLE_WARNING}"
            );
            true
        }
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan(
        policy: cerulion_core::GatewayEgressPolicy,
        announce: &[&str],
    ) -> cerulion_core::GatewayPlan {
        cerulion_core::GatewayPlan {
            egress_policy: policy,
            announce: announce.iter().map(|s| s.to_string()).collect(),
            ingress: Vec::new(),
        }
    }

    /// A plan that sends one topic.
    fn egress() -> cerulion_core::GatewayPlan {
        plan(cerulion_core::GatewayEgressPolicy::AllowAll, &["/r/cmd"])
    }

    fn net(listen: &[&str], connect: &[&str]) -> cerulion_core::NetworkConfig {
        cerulion_core::NetworkConfig {
            listen_endpoints: listen.iter().map(|s| s.to_string()).collect(),
            connect_endpoints: connect.iter().map(|s| s.to_string()).collect(),
            ..cerulion_core::NetworkConfig::default()
        }
    }

    #[test]
    fn fires_for_a_child_tcp_listener_when_the_sysctl_is_on() {
        let n = net(&["tcp/[::]:7683"], &[]);
        assert_eq!(
            idle_warning(EgressRoute::Child(&n), &egress(), Some("1\n")),
            Some(IdleWarning::Child(vec!["tcp/[::]:7683".to_string()]))
        );
    }

    #[test]
    fn lists_only_the_tcp_locators_listen_first() {
        let n = net(
            &["udp/0.0.0.0:7447?rel=1;mixed_rel=1", "tcp/0.0.0.0:7448"],
            &["quic/10.0.0.2:7447", "tcp/10.0.0.3:7447"],
        );
        assert_eq!(
            idle_warning(EgressRoute::Child(&n), &egress(), Some("1")),
            Some(IdleWarning::Child(vec![
                "tcp/0.0.0.0:7448".to_string(),
                "tcp/10.0.0.3:7447".to_string()
            ]))
        );
    }

    #[test]
    fn fires_for_the_netd_route_when_the_sysctl_is_on() {
        assert_eq!(
            idle_warning(EgressRoute::Netd, &egress(), Some("1")),
            Some(IdleWarning::Netd)
        );
    }

    #[test]
    fn any_nonzero_value_is_on_as_the_kernel_reads_it() {
        let n = net(&["tcp/0.0.0.0:7447"], &[]);
        assert!(idle_warning(EgressRoute::Child(&n), &egress(), Some("2\n")).is_some());
        assert!(idle_warning(EgressRoute::Child(&n), &egress(), Some("-1")).is_some());
    }

    #[test]
    fn does_not_fire_when_the_sysctl_is_off() {
        let n = net(&["tcp/[::]:7683"], &[]);
        assert_eq!(
            idle_warning(EgressRoute::Child(&n), &egress(), Some("0\n")),
            None
        );
        assert_eq!(idle_warning(EgressRoute::Netd, &egress(), Some("0")), None);
    }

    #[test]
    fn does_not_fire_for_a_child_without_a_tcp_locator() {
        let n = net(
            &["quic/0.0.0.0:7447?mixed_rel=1"],
            &["udp/10.0.0.2:7447?rel=1;mixed_rel=1"],
        );
        assert_eq!(
            idle_warning(EgressRoute::Child(&n), &egress(), Some("1")),
            None
        );
    }

    #[test]
    fn does_not_fire_when_the_sysctl_is_unreadable_or_not_an_integer() {
        let n = net(&["tcp/[::]:7683"], &[]);
        for text in [None, Some("yes"), Some("")] {
            assert_eq!(idle_warning(EgressRoute::Child(&n), &egress(), text), None);
            assert_eq!(idle_warning(EgressRoute::Netd, &egress(), text), None);
        }
    }

    #[test]
    fn does_not_fire_for_a_plan_with_no_egress() {
        use cerulion_core::GatewayEgressPolicy::{AllowAll, AllowList};
        let n = net(&["tcp/0.0.0.0:7447"], &[]);
        let no_egress = [
            plan(AllowList(Vec::new()), &[]),
            plan(AllowAll, &[]),
            plan(AllowList(vec!["/r/cmd".into()]), &[]),
            plan(AllowList(Vec::new()), &["/r/cmd"]),
        ];
        for p in &no_egress {
            assert!(!plan_has_egress(p), "{p:?}");
            assert_eq!(idle_warning(EgressRoute::Child(&n), p, Some("1")), None);
            assert_eq!(idle_warning(EgressRoute::Netd, p, Some("1")), None);
        }
        assert!(plan_has_egress(&plan(
            AllowList(vec!["/r/cmd".into()]),
            &["/r/cmd"]
        )));
        assert!(plan_has_egress(&egress()));
    }

    #[test]
    fn the_warning_names_both_fixes_and_the_sysctl() {
        for needle in [
            "net.ipv4.tcp_slow_start_after_idle is nonzero",
            "sysctl -w net.ipv4.tcp_slow_start_after_idle=0",
            "/etc/sysctl.d/",
            "quic/ locators, their tls: files and an egress: list",
            "round-trip time",
        ] {
            assert!(
                TCP_SLOW_START_AFTER_IDLE_WARNING.contains(needle),
                "warning lacks {needle:?}"
            );
        }
    }
}
