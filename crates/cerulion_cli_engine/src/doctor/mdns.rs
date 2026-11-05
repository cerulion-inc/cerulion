// SPDX-License-Identifier: AGPL-3.0-only
//! `net.mdns`: whether multicast DNS works on this machine's LAN.
//!
//! Browses `_cerulion._tcp` for [`crate::discovery_ladder::MDNS_RUNG_BUDGET`]
//! with [`crate::mdns_discovery::browse_rung_report`], the browse `topic list`
//! makes, which starts its own mDNS responder for the window, sends multicast
//! queries and shuts the responder down before it returns. Seeing no peer is
//! not a fault (there may be none), and a network that drops multicast, or a
//! host whose mDNS socket could not bind, looks the same as one with no peer.
//! A browse that cannot start or whose responder stops mid-window is the
//! fault this check warns about.

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const MDNS: Check = Check {
    id: "net.mdns",
    why: "Cerulion finds robots on the LAN by name with multicast DNS (`_cerulion._tcp`); \
          where multicast is blocked, `topic list` and `connect <name>` find nothing without \
          an explicit peer.",
    network: true,
    run,
};

/// The decision over the browse: the robots that answered and whether the
/// window ran cleanly.
pub(super) fn decide(robots: &[String], window_elapsed_cleanly: bool) -> Outcome {
    let count = Reading::Int(u64::try_from(robots.len()).unwrap_or(u64::MAX));
    let seen = if robots.is_empty() {
        "no Cerulion machine answered".to_string()
    } else {
        format!("{} answered: {}", robots.len(), robots.join(", "))
    };
    if window_elapsed_cleanly {
        Outcome::new(
            Status::Info,
            format!(
                "an mDNS browse of _cerulion._tcp ran its window; {seen} (a network that drops \
                 multicast shows none too)"
            ),
        )
        .reading("robots", count)
    } else {
        Outcome::new(
            Status::Warn,
            format!(
                "the mDNS browse of _cerulion._tcp could not start, or its responder stopped \
                 mid-window; {seen}"
            ),
        )
        .with_fix(
            "run `cerulion -v doctor --check net.mdns` to see the responder's error, or name \
             peers in ~/.cerulion/config.toml (`peers = [\"robot.local\"]`)",
        )
        .reading("robots", count)
    }
}

fn run(_ctx: &Context) -> Outcome {
    let report =
        crate::mdns_discovery::browse_rung_report(crate::discovery_ladder::MDNS_RUNG_BUDGET);
    let mut robots: Vec<String> = report.peers.into_iter().map(|p| p.robot).collect();
    robots.sort();
    robots.dedup();
    decide(&robots, report.window_elapsed_cleanly)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_window_is_info_with_or_without_peers() {
        let none = decide(&[], true);
        assert_eq!(none.status, Status::Info);
        assert_eq!(
            none.summary,
            "an mDNS browse of _cerulion._tcp ran its window; no Cerulion machine answered (a \
             network that drops multicast shows none too)"
        );
        let two = decide(&["go2".to_string(), "lab".to_string()], true);
        assert_eq!(
            two.summary,
            "an mDNS browse of _cerulion._tcp ran its window; 2 answered: go2, lab (a network that \
             drops multicast shows none too)"
        );
        assert_eq!(two.readings.get("robots"), Some(&Reading::Int(2)));
    }

    #[test]
    fn a_window_that_did_not_run_warns_with_the_fix() {
        let o = decide(&[], false);
        assert_eq!(o.status, Status::Warn);
        assert!(o.fix.unwrap().contains("peers = [\"robot.local\"]"));
    }
}
