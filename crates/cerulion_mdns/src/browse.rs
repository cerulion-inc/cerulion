// SPDX-License-Identifier: AGPL-3.0-only
//! The browse half: a bounded `_cerulion._tcp` browse that maps each resolved
//! instance to `Mdns`-rung [`DiscoveredPeer`]s.
//!
//! Two callers: the `cerulion` CLI's discovery ladder (`topic list`) and
//! `cerulion-netd`'s boot-time peer fold. Each call starts its own
//! `ServiceDaemon`, browses for at most `budget` and shuts the daemon down
//! before it returns, so the multicast queries it sends stop with the call.

use std::net::IpAddr;
use std::time::{Duration, Instant};

use cerulion_discovery::ladder::{DiscoveredPeer, DiscoveryRung};
use mdns_sd::{ServiceDaemon, ServiceEvent};

use crate::CERULION_SERVICE_TYPE;

/// Browse `_cerulion._tcp` for up to `budget`, returning every resolved
/// gateway as an `Mdns`-rung [`DiscoveredPeer`]. A daemon-start or browse
/// failure logs a `warn!` and returns an empty vec; it never returns an error
/// and never panics. The daemon is shut down before returning.
pub fn browse_rung(budget: Duration) -> Vec<DiscoveredPeer> {
    browse_rung_report(budget).peers
}

/// What a [`browse_rung_report`] pass learned, plus whether its window
/// elapsed cleanly.
pub struct BrowseRungReport {
    /// The gateways resolved during the window: possibly all of them, possibly
    /// none, possibly a partial set when the window did not elapse cleanly.
    pub peers: Vec<DiscoveredPeer>,
    /// `false` when the browse could not start or its daemon exited
    /// mid-window, so `peers` may be partial: presence in `peers` is real,
    /// absence proves nothing.
    pub window_elapsed_cleanly: bool,
}

impl BrowseRungReport {
    fn partial(peers: Vec<DiscoveredPeer>) -> Self {
        Self {
            peers,
            window_elapsed_cleanly: false,
        }
    }
}

/// [`browse_rung`] that also reports whether the browse window elapsed
/// cleanly. A caller asserting that a record was withdrawn needs clean
/// windows only.
pub fn browse_rung_report(budget: Duration) -> BrowseRungReport {
    let daemon = match ServiceDaemon::new() {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "mDNS: cannot start the ServiceDaemon to browse `_cerulion._tcp`; robot discovery continues on the other rungs (cache, hostname)"
            );
            return BrowseRungReport::partial(Vec::new());
        }
    };
    let receiver = match daemon.browse(CERULION_SERVICE_TYPE) {
        Ok(rx) => rx,
        Err(e) => {
            tracing::warn!(
                error = %e,
                "mDNS: cannot browse `_cerulion._tcp`; robot discovery continues on the other rungs (cache, hostname)"
            );
            let _ = daemon.shutdown();
            return BrowseRungReport::partial(Vec::new());
        }
    };

    // `recv_timeout` returns Err on both a window timeout and a sender
    // disconnect; `is_disconnected` tells them apart.
    let report = drain_browse_window(
        budget,
        Instant::now,
        |remaining| match receiver.recv_timeout(remaining) {
            Ok(ServiceEvent::ServiceResolved(resolved)) => {
                let addrs: Vec<IpAddr> =
                    resolved.addresses.iter().map(|a| a.to_ip_addr()).collect();
                let robot_txt = resolved.get_property_val_str("robot");
                BrowseEvent::Resolved(peer_from_resolved(
                    robot_txt,
                    &resolved.fullname,
                    &addrs,
                    resolved.port,
                ))
            }
            Ok(_) => BrowseEvent::Other,
            Err(_) => BrowseEvent::NoEvent,
        },
        || receiver.is_disconnected(),
    );
    let _ = daemon.shutdown();
    report
}

/// One receive of a browse window.
enum BrowseEvent {
    /// A resolved `_cerulion._tcp` record, mapped to its peers.
    Resolved(Vec<DiscoveredPeer>),
    /// Any other browse event.
    Other,
    /// The receive timed out or the daemon's sender disconnected.
    NoEvent,
}

/// Collect a browse window of `budget` from `recv` (handed the time left
/// in the window) and report whether it elapsed cleanly. A zero budget
/// observes nothing, so it is partial. Every exit, including the deadline,
/// checks `is_disconnected`: a daemon that exited while events were still
/// queued leaves a partial view, and a partial view proves no absence.
fn drain_browse_window(
    budget: Duration,
    now: impl Fn() -> Instant,
    mut recv: impl FnMut(Duration) -> BrowseEvent,
    is_disconnected: impl Fn() -> bool,
) -> BrowseRungReport {
    if budget.is_zero() {
        return BrowseRungReport::partial(Vec::new());
    }
    let mut peers: Vec<DiscoveredPeer> = Vec::new();
    let deadline = now() + budget;
    loop {
        let current = now();
        if current >= deadline {
            break;
        }
        match recv(deadline - current) {
            BrowseEvent::Resolved(resolved) => peers.extend(resolved),
            BrowseEvent::Other => {}
            BrowseEvent::NoEvent => break,
        }
    }
    // A disconnect means the ServiceDaemon's background thread exited before
    // the caller shut it down, so the result may be partial.
    if is_disconnected() {
        tracing::warn!(
            "mDNS: the `_cerulion._tcp` browse daemon died mid-browse; robot discovery results may be PARTIAL; if an expected robot is missing, reach it directly with `--connect tcp/<host>:7683`"
        );
        return BrowseRungReport::partial(peers);
    }
    BrowseRungReport {
        peers,
        window_elapsed_cleanly: true,
    }
}

/// Recover the instance name from a resolved fullname by stripping the
/// `.<service-type>` suffix (`myrobot._cerulion._tcp.local.` → `myrobot`).
/// A fullname without that suffix is returned unchanged.
pub fn instance_from_fullname(fullname: &str) -> String {
    let suffix = format!(".{CERULION_SERVICE_TYPE}");
    fullname
        .strip_suffix(&suffix)
        .unwrap_or(fullname)
        .to_string()
}

/// Map one resolved mDNS record to zero or more [`DiscoveredPeer`]s. Pure.
///
/// - `robot` is the TXT `robot=` value, or the fullname's instance name when
///   the record has none.
/// - If the record carries any IPv4 address, one peer is emitted per IPv4
///   address (`tcp/<ip>:<port>`); only an IPv6-only record yields IPv6 peers
///   (`tcp/[<ip>]:<port>`). No addresses yields no peers.
fn peer_from_resolved(
    robot_txt: Option<&str>,
    fullname: &str,
    addrs: &[IpAddr],
    port: u16,
) -> Vec<DiscoveredPeer> {
    let robot = robot_txt
        .map(|s| s.to_string())
        .unwrap_or_else(|| instance_from_fullname(fullname));
    let v4: Vec<&IpAddr> = addrs.iter().filter(|a| a.is_ipv4()).collect();
    let chosen: Vec<&IpAddr> = if v4.is_empty() {
        addrs.iter().filter(|a| a.is_ipv6()).collect()
    } else {
        v4
    };
    chosen
        .into_iter()
        .map(|ip| {
            let locator = match ip {
                IpAddr::V4(v4) => format!("tcp/{v4}:{port}"),
                IpAddr::V6(v6) => format!("tcp/[{v6}]:{port}"),
            };
            DiscoveredPeer {
                robot: robot.clone(),
                locator,
                rung: DiscoveryRung::Mdns,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, Ipv6Addr};

    fn peer(robot: &str) -> DiscoveredPeer {
        DiscoveredPeer {
            robot: robot.to_string(),
            locator: "tcp/10.0.0.9:7683".to_string(),
            rung: DiscoveryRung::Mdns,
        }
    }

    /// A zero budget observes nothing: the report is partial and nothing is
    /// received.
    #[test]
    fn a_zero_budget_browse_is_partial() {
        let start = Instant::now();
        let report = drain_browse_window(
            Duration::ZERO,
            || start,
            |_| panic!("a zero window receives nothing"),
            || false,
        );
        assert!(report.peers.is_empty());
        assert!(!report.window_elapsed_cleanly);
    }

    /// A resolved event spans the deadline and the daemon disconnected
    /// meanwhile: the loop leaves at the deadline, and the report is still
    /// partial (it keeps the peer it saw).
    #[test]
    fn a_disconnect_seen_at_the_deadline_marks_the_browse_partial() {
        let start = Instant::now();
        let clock = std::cell::Cell::new(start);
        let report = drain_browse_window(
            Duration::from_secs(1),
            || clock.get(),
            |remaining| {
                clock.set(clock.get() + remaining);
                BrowseEvent::Resolved(vec![peer("go2")])
            },
            || true,
        );
        assert_eq!(report.peers, vec![peer("go2")]);
        assert!(!report.window_elapsed_cleanly);
    }

    /// The same window with the daemon alive is clean.
    #[test]
    fn a_window_that_reaches_its_deadline_with_a_live_daemon_is_clean() {
        let start = Instant::now();
        let clock = std::cell::Cell::new(start);
        let report = drain_browse_window(
            Duration::from_secs(1),
            || clock.get(),
            |remaining| {
                clock.set(clock.get() + remaining);
                BrowseEvent::Other
            },
            || false,
        );
        assert!(report.peers.is_empty());
        assert!(report.window_elapsed_cleanly);
    }

    #[test]
    fn instance_from_fullname_strips_service_suffix() {
        assert_eq!(
            instance_from_fullname("myrobot._cerulion._tcp.local."),
            "myrobot"
        );
        // A fullname WITHOUT the suffix is returned verbatim (defensive).
        assert_eq!(instance_from_fullname("bare-name"), "bare-name");
    }

    #[test]
    fn peer_from_resolved_uses_txt_robot_value() {
        let addrs = [IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7))];
        let peers = peer_from_resolved(
            Some("robotA"),
            "ignored._cerulion._tcp.local.",
            &addrs,
            7683,
        );
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[0].robot, "robotA");
        assert_eq!(peers[0].locator, "tcp/192.168.1.7:7683");
        assert_eq!(peers[0].rung, DiscoveryRung::Mdns);
    }

    #[test]
    fn peer_from_resolved_falls_back_to_instance_name_when_no_txt() {
        let addrs = [IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1))];
        let peers = peer_from_resolved(None, "myrobot._cerulion._tcp.local.", &addrs, 7683);
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[0].robot, "myrobot");
        assert_eq!(peers[0].locator, "tcp/10.0.0.1:7683");
    }

    #[test]
    fn peer_from_resolved_prefers_ipv4_over_ipv6() {
        // A dual-stack record: v4 wins, the v6 sibling is dropped → ONE peer.
        let addrs = [
            IpAddr::V6(Ipv6Addr::new(0xfe80, 0, 0, 0, 0, 0, 0, 1)),
            IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7)),
        ];
        let peers = peer_from_resolved(Some("dual"), "dual._cerulion._tcp.local.", &addrs, 7683);
        assert_eq!(peers.len(), 1, "v4 preference collapses the v6 sibling");
        assert_eq!(peers[0].locator, "tcp/192.168.1.7:7683");
    }

    #[test]
    fn peer_from_resolved_uses_ipv6_bracket_form_when_v6_only() {
        let addrs = [IpAddr::V6(Ipv6Addr::new(0xfe80, 0, 0, 0, 0, 0, 0, 1))];
        let peers = peer_from_resolved(Some("v6"), "v6._cerulion._tcp.local.", &addrs, 7683);
        assert_eq!(peers.len(), 1);
        assert_eq!(peers[0].locator, "tcp/[fe80::1]:7683");
        assert_eq!(peers[0].rung, DiscoveryRung::Mdns);
    }

    #[test]
    fn peer_from_resolved_emits_one_peer_per_v4_address() {
        let addrs = [
            IpAddr::V4(Ipv4Addr::new(192, 168, 1, 7)),
            IpAddr::V4(Ipv4Addr::new(192, 168, 1, 8)),
        ];
        let peers = peer_from_resolved(Some("multi"), "multi._cerulion._tcp.local.", &addrs, 7683);
        // Input Vec order is preserved by the map → deterministic oracle.
        assert_eq!(peers.len(), 2);
        assert_eq!(peers[0].locator, "tcp/192.168.1.7:7683");
        assert_eq!(peers[1].locator, "tcp/192.168.1.8:7683");
        assert!(peers.iter().all(|p| p.robot == "multi"));
    }

    #[test]
    fn peer_from_resolved_empty_addrs_yields_no_peers() {
        let peers = peer_from_resolved(Some("x"), "x._cerulion._tcp.local.", &[], 7683);
        assert!(peers.is_empty());
    }
}
