// SPDX-License-Identifier: AGPL-3.0-only
//! `net.tcp-slow-start`: whether this Linux host resets an idle TCP
//! connection's congestion window.
//!
//! Reads [`crate::tcp_idle_warning::SLOW_START_AFTER_IDLE_SYSCTL`] in this
//! process's network namespace and classifies it with
//! [`crate::tcp_idle_warning::idle_reset_enabled`], the rule the `graph run`
//! warning uses: any nonzero integer is on. The fix is the one that warning
//! prints. The sysctl matters only to `tcp/` links, the default carrier.

// The decision compiles and is tested on every host; only a Linux build calls
// it from `run`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use super::{Check, Context, Outcome, Reading, Status};
use crate::tcp_idle_warning::{
    idle_reset_enabled, SLOW_START_AFTER_IDLE_SYSCTL, TCP_SLOW_START_AFTER_IDLE_WARNING,
};

pub(super) const SLOW_START: Check = Check {
    id: "net.tcp-slow-start",
    why: "with net.ipv4.tcp_slow_start_after_idle nonzero (the Linux default) a TCP \
          connection idle for longer than one retransmission timeout lets its congestion window \
          decay toward its initial size, so a large message sent at a low rate over tcp/, the default carrier, pays \
          extra round trips; quic/ has no idle reset.",
    network: false,
    run,
};

/// The fix the `graph run` warning prints, after its `Fix: `.
fn fix_text() -> &'static str {
    TCP_SLOW_START_AFTER_IDLE_WARNING
        .split_once("Fix: ")
        .map_or(TCP_SLOW_START_AFTER_IDLE_WARNING, |(_, fix)| fix)
}

/// The decision over the sysctl as read: its text, or the read error.
pub(super) fn decide(sysctl: Result<&str, String>) -> Outcome {
    let path = SLOW_START_AFTER_IDLE_SYSCTL;
    let text = match sysctl {
        Ok(text) => text,
        Err(err) => {
            return Outcome::new(Status::Warn, format!("could not read {path}: {err}"))
                .with_fix("run `sysctl net.ipv4.tcp_slow_start_after_idle` to see the value");
        }
    };
    let value = Reading::Text(text.trim().to_string());
    match idle_reset_enabled(text) {
        Some(true) => Outcome::new(
            Status::Warn,
            format!(
                "net.ipv4.tcp_slow_start_after_idle is {}: idle tcp/ links lose their \
                 congestion window",
                text.trim()
            ),
        )
        .with_fix(fix_text())
        .reading("value", value),
        Some(false) => Outcome::new(
            Status::Pass,
            "net.ipv4.tcp_slow_start_after_idle is 0: idle tcp/ links keep their congestion \
             window",
        )
        .reading("value", value),
        None => Outcome::new(
            Status::Warn,
            format!("{path} holds `{}`, which is not an integer", text.trim()),
        )
        .with_fix("run `sysctl net.ipv4.tcp_slow_start_after_idle` to see the value")
        .reading("value", value),
    }
}

#[cfg(target_os = "linux")]
fn run(_ctx: &Context) -> Outcome {
    let read = std::fs::read_to_string(SLOW_START_AFTER_IDLE_SYSCTL).map_err(|e| e.to_string());
    decide(read.as_deref().map_err(Clone::clone))
}

#[cfg(not(target_os = "linux"))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("net.ipv4.tcp_slow_start_after_idle is a Linux sysctl")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_on_warns_with_the_graph_run_fix() {
        let o = decide(Ok("1\n"));
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "net.ipv4.tcp_slow_start_after_idle is 1: idle tcp/ links lose their congestion \
             window"
        );
        let fix = o.fix.unwrap();
        assert!(
            fix.starts_with("set `sysctl -w net.ipv4.tcp_slow_start_after_idle=0`"),
            "{fix}"
        );
        assert!(fix.contains("/etc/sysctl.d/"), "{fix}");
        assert_eq!(
            o.readings.get("value"),
            Some(&Reading::Text("1".to_string()))
        );
    }

    #[test]
    fn any_nonzero_value_is_on_and_zero_passes() {
        assert_eq!(decide(Ok("2")).status, Status::Warn);
        let off = decide(Ok("0\n"));
        assert_eq!(off.status, Status::Pass);
        assert_eq!(off.fix, None);
    }

    #[test]
    fn unreadable_or_non_integer_text_warns() {
        assert_eq!(
            decide(Err("permission denied".to_string())).status,
            Status::Warn
        );
        let junk = decide(Ok("yes"));
        assert_eq!(junk.status, Status::Warn);
        assert!(junk
            .summary
            .ends_with("holds `yes`, which is not an integer"));
    }
}
