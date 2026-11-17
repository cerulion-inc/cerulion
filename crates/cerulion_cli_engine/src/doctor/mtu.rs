// SPDX-License-Identifier: AGPL-3.0-only
//! `net.mtu`: whether each IP network interface can carry a QUIC datagram.
//!
//! QUIC requires every UDP datagram that carries its handshake to have a
//! payload of at least 1200 bytes (RFC 9000, section 14.1), so an interface
//! whose MTU is below 1200 + 8 (UDP) + 20 (IPv4) = 1228 bytes cannot carry
//! QUIC. Linux turns IPv6 off on an interface whose MTU is below 1280, so a
//! smaller interface carries only IPv4 and the IPv4 figure is the one bound.
//!
//! On Linux the check reads `type`, `flags`, `operstate` and `mtu` under
//! `/sys/class/net/<interface>/` and judges every interface whose type
//! carries IP (Ethernet, PPP, raw-IP modems, IP tunnels, InfiniBand and the type-less
//! `none` of WireGuard and TUN), whose `IFF_UP` flag is set and
//! `IFF_LOOPBACK` is not, and whose `operstate` is `up` or `unknown`
//! (WireGuard and TUN report `unknown`). A CAN bus and other non-IP links are
//! not judged. The path MTU between two machines can be lower than either
//! end's interface MTU; this check reads only this host's side.

// The decision and the reader compile and are tested on every host; only a
// Linux build calls them from `run`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use std::path::Path;

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const MTU: Check = Check {
    id: "net.mtu",
    why: "the quic/ and udp/ carriers need room for a 1200-byte UDP payload in one packet, an \
          interface MTU of at least 1228 bytes; tcp/ does not.",
    network: false,
    run,
};

/// The smallest MTU that carries a QUIC handshake datagram over IPv4.
pub(super) const QUIC_MIN_MTU: u64 = 1200 + 8 + 20;

/// The `ARPHRD_*` link types (`linux/if_arp.h`) that carry IP: Ethernet,
/// InfiniBand, PPP, raw IP (LTE and 5G modems), IPIP, IPv6-in-IPv6, SIT, GRE,
/// IPv6 GRE, and `NONE` (WireGuard, TUN).
const IP_LINK_TYPES: [u64; 10] = [1, 32, 512, 519, 768, 769, 776, 778, 823, 65534];

/// One interface as read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Interface {
    pub(super) name: String,
    pub(super) mtu: u64,
}

/// The decision over the IP interfaces that are up.
pub(super) fn decide(interfaces: &[Interface]) -> Outcome {
    if interfaces.is_empty() {
        return Outcome::new(
            Status::Info,
            "no IP network interface other than loopback is up",
        );
    }
    let listing: Vec<String> = interfaces
        .iter()
        .map(|i| format!("{} {}", i.name, i.mtu))
        .collect();
    let small: Vec<&Interface> = interfaces.iter().filter(|i| i.mtu < QUIC_MIN_MTU).collect();
    let base = |status, summary: String| {
        Outcome::new(status, summary)
            .reading("interfaces", Reading::Text(listing.join(",")))
            .reading(
                "smallest_mtu",
                Reading::Int(interfaces.iter().map(|i| i.mtu).min().unwrap_or(0)),
            )
    };
    if small.is_empty() {
        return base(
            Status::Pass,
            format!(
                "every IP interface that is up carries a QUIC datagram: {}",
                listing.join(", ")
            ),
        );
    }
    let described: Vec<String> = small
        .iter()
        .map(|i| format!("{} (MTU {})", i.name, i.mtu))
        .collect();
    let commands: Vec<String> = small
        .iter()
        .map(|i| format!("`sudo ip link set dev {} mtu {QUIC_MIN_MTU}`", i.name))
        .collect();
    base(
        Status::Warn,
        format!(
            "below the {QUIC_MIN_MTU} bytes a QUIC datagram needs: {}",
            described.join(", ")
        ),
    )
    .with_fix(format!(
        "raise the MTU ({}, or the tunnel's own MTU setting), or use a tcp/ locator for links \
         over {}",
        commands.join(", "),
        if small.len() == 1 { "it" } else { "them" }
    ))
}

/// The IP interfaces under `sys_class_net` that are up and not a loopback,
/// by name.
pub(super) fn read_interfaces(sys_class_net: &Path) -> Vec<Interface> {
    /// `IFF_UP` and `IFF_LOOPBACK` in `flags` (`linux/if.h`).
    const IFF_UP: u64 = 0x1;
    const IFF_LOOPBACK: u64 = 0x8;
    let Ok(entries) = std::fs::read_dir(sys_class_net) else {
        return Vec::new();
    };
    let mut out: Vec<Interface> = entries
        .filter_map(Result::ok)
        .filter_map(|e| {
            let dir = e.path();
            let read = |f: &str| std::fs::read_to_string(dir.join(f)).ok();
            let link_type: u64 = read("type")?.trim().parse().ok()?;
            let flags = read("flags")
                .and_then(|s| u64::from_str_radix(s.trim().trim_start_matches("0x"), 16).ok())?;
            let mtu: u64 = read("mtu")?.trim().parse().ok()?;
            let live = read("operstate").is_some_and(|s| matches!(s.trim(), "up" | "unknown"));
            (IP_LINK_TYPES.contains(&link_type)
                && flags & IFF_UP != 0
                && flags & IFF_LOOPBACK == 0
                && live)
                .then(|| Interface {
                    name: e.file_name().to_string_lossy().into_owned(),
                    mtu,
                })
        })
        .collect();
    out.sort_by(|a, b| a.name.cmp(&b.name));
    out
}

#[cfg(target_os = "linux")]
fn run(_ctx: &Context) -> Outcome {
    decide(&read_interfaces(Path::new("/sys/class/net")))
}

#[cfg(not(target_os = "linux"))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("this check reads the Linux /sys/class/net interface files")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iface(name: &str, mtu: u64) -> Interface {
        Interface {
            name: name.to_string(),
            mtu,
        }
    }

    #[test]
    fn the_threshold_is_the_quic_minimum_plus_udp_and_ipv4_headers() {
        assert_eq!(QUIC_MIN_MTU, 1228);
    }

    #[test]
    fn passes_at_the_threshold() {
        let o = decide(&[iface("eth0", 1500), iface("wg1", 1280), iface("w", 1228)]);
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "every IP interface that is up carries a QUIC datagram: eth0 1500, wg1 1280, \
             w 1228"
        );
        assert_eq!(o.readings.get("smallest_mtu"), Some(&Reading::Int(1228)));
    }

    #[test]
    fn warns_one_byte_under_the_threshold_and_names_every_small_interface() {
        let o = decide(&[iface("eth0", 1500), iface("v4", 1227), iface("ppp0", 576)]);
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "below the 1228 bytes a QUIC datagram needs: v4 (MTU 1227), ppp0 (MTU 576)"
        );
        assert_eq!(
            o.fix.as_deref(),
            Some(
                "raise the MTU (`sudo ip link set dev v4 mtu 1228`, `sudo ip link set dev ppp0 \
                 mtu 1228`, or the tunnel's own MTU setting), or use a tcp/ locator for links \
                 over them"
            )
        );
    }

    #[test]
    fn no_interface_up_is_info() {
        assert_eq!(decide(&[]).status, Status::Info);
    }

    #[test]
    fn read_interfaces_keeps_live_ip_interfaces_only() {
        let root = tempfile::tempdir().unwrap();
        for (name, link_type, flags, mtu, oper) in [
            ("lo", "772", "0x9", "65536", "unknown"),
            ("eth0", "1", "0x1003", "1500", "up"),
            ("eth1", "1", "0x1002", "1500", "down"),
            ("eth2", "1", "0x1003", "1500", "lowerlayerdown"),
            ("wg0", "65534", "0x91", "1420", "unknown"),
            ("tun0", "65534", "0x1091", "1200", "down"),
            ("can0", "280", "0xc1", "16", "up"),
        ] {
            let d = root.path().join(name);
            std::fs::create_dir(&d).unwrap();
            for (file, value) in [
                ("type", link_type),
                ("flags", flags),
                ("mtu", mtu),
                ("operstate", oper),
            ] {
                std::fs::write(d.join(file), format!("{value}\n")).unwrap();
            }
        }
        assert_eq!(
            read_interfaces(root.path()),
            vec![iface("eth0", 1500), iface("wg0", 1420)]
        );
    }
}
