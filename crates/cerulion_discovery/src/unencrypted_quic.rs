// SPDX-License-Identifier: AGPL-3.0-only
//! The `CERULION_UNENCRYPTED_QUIC=on` opt-in: listen on and dial zenoh's
//! unencrypted QUIC carrier (`udp/<host>:<port>?rel=1`) first, with TCP as the
//! fallback.
//!
//! zenoh's `udp/...?rel=1` carrier is QUIC streams with encryption and peer
//! authentication switched off: a frame on it is as readable on the wire as one
//! on today's default `tcp/` carrier. What it changes is congestion control. A
//! Linux TCP connection that sits idle longer than one retransmission timeout
//! restarts from its initial window (`net.ipv4.tcp_slow_start_after_idle`), so a
//! large message sent about once a second pays several extra round trips; the
//! QUIC stack zenoh uses has no idle reset.
//!
//! This module holds the pure pieces every crate that acts on the opt-in shares:
//! the switch and the locator rewrites. The dialing side (netd's boot-time fold
//! and its QUIC probe) lives in `cerulion_netd`.

use std::collections::HashSet;

/// The opt-in environment variable. Exactly [`UNENCRYPTED_QUIC_ON`] turns it on.
pub const UNENCRYPTED_QUIC_ENV: &str = "CERULION_UNENCRYPTED_QUIC";

/// The one value that turns [`UNENCRYPTED_QUIC_ENV`] on.
pub const UNENCRYPTED_QUIC_ON: &str = "on";

/// The `_cerulion._tcp` TXT key a gateway sets to the port of its
/// `udp/...?rel=1` listener when the opt-in is on.
pub const UDP_REL_TXT_KEY: &str = "udp_rel";

/// Whether the opt-in is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnencryptedQuic {
    /// Today's behaviour: `tcp/` only.
    Off,
    /// Listen on `udp/...?rel=1` beside every `tcp/` listener, advertise it, and
    /// dial it first.
    On,
}

/// How a [`UNENCRYPTED_QUIC_ENV`] value reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnencryptedQuicValue {
    /// Unset or empty.
    Unset,
    /// Exactly [`UNENCRYPTED_QUIC_ON`].
    On,
    /// Any other non-empty value (`ON`, `1`, `true`, `yes`): the opt-in stays
    /// off and the reader warns, naming the value.
    Unrecognised,
}

impl UnencryptedQuicValue {
    /// The switch this value selects.
    pub fn switch(self) -> UnencryptedQuic {
        match self {
            Self::On => UnencryptedQuic::On,
            Self::Unset | Self::Unrecognised => UnencryptedQuic::Off,
        }
    }
}

/// Classify a [`UNENCRYPTED_QUIC_ENV`] value, byte for byte. Pure.
pub fn classify_unencrypted_quic(value: Option<&str>) -> UnencryptedQuicValue {
    match value {
        None | Some("") => UnencryptedQuicValue::Unset,
        Some(UNENCRYPTED_QUIC_ON) => UnencryptedQuicValue::On,
        Some(_) => UnencryptedQuicValue::Unrecognised,
    }
}

/// Read [`UNENCRYPTED_QUIC_ENV`] from the process environment. An unrecognised
/// value logs one `warn!` naming it and reads as [`UnencryptedQuic::Off`].
pub fn unencrypted_quic_from_env() -> UnencryptedQuic {
    let raw = std::env::var(UNENCRYPTED_QUIC_ENV).ok();
    let value = classify_unencrypted_quic(raw.as_deref());
    if value == UnencryptedQuicValue::Unrecognised {
        tracing::warn!(
            value = raw.as_deref().unwrap_or(""),
            "{UNENCRYPTED_QUIC_ENV} is set but not to \"{UNENCRYPTED_QUIC_ON}\", so it is OFF: \
             this machine listens on and dials tcp/ only. Set \
             {UNENCRYPTED_QUIC_ENV}={UNENCRYPTED_QUIC_ON} to listen on and dial unencrypted QUIC \
             first."
        );
    }
    value.switch()
}

/// [`unencrypted_quic_from_env`] without the warning, for a second reader in the
/// same process (the first one already warned).
pub fn unencrypted_quic_from_env_quiet() -> UnencryptedQuic {
    classify_unencrypted_quic(std::env::var(UNENCRYPTED_QUIC_ENV).ok().as_deref()).switch()
}

/// `tcp/<host>:<port>` to `udp/<host>:<port>?rel=1`. `None` for any other
/// locator, including a `tcp/` one with metadata or config (`?...`, `#...`),
/// whose meaning on the other carrier is not defined. Pure.
pub fn udp_rel_twin_of_tcp(locator: &str) -> Option<String> {
    let address = locator.strip_prefix("tcp/")?;
    (!address.is_empty() && !address.contains(['?', '#'])).then(|| format!("udp/{address}?rel=1"))
}

/// `udp/<host>:<port>?rel=1` (exactly that metadata, no config) to
/// `tcp/<host>:<port>`. `None` for any other locator. Pure.
pub fn tcp_twin_of_udp_rel(locator: &str) -> Option<String> {
    let address = locator.strip_prefix("udp/")?.strip_suffix("?rel=1")?;
    (!address.is_empty() && !address.contains(['?', '#'])).then(|| format!("tcp/{address}"))
}

/// The listen set with `udp/<host>:<port>?rel=1` added after each
/// `tcp/<host>:<port>` that has no such entry yet, in order. Pure.
pub fn with_udp_rel_listeners(listen: &[String]) -> Vec<String> {
    let present: HashSet<&str> = listen.iter().map(String::as_str).collect();
    let mut added: HashSet<String> = HashSet::new();
    let mut out = Vec::with_capacity(listen.len() * 2);
    for locator in listen {
        out.push(locator.clone());
        if let Some(twin) = udp_rel_twin_of_tcp(locator) {
            if !present.contains(twin.as_str()) && added.insert(twin.clone()) {
                out.push(twin);
            }
        }
    }
    out
}

/// One discovered gateway address that also advertised a `udp/...?rel=1`
/// listener: the `tcp/` locator the discovery rung produced and the
/// unencrypted QUIC locator for the same address.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UdpRelTwin {
    /// The discovered `tcp/<ip>:<port>` locator.
    pub tcp: String,
    /// `udp/<ip>:<udp_rel port>?rel=1` for the same address.
    pub udp_rel: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_the_exact_value_on_turns_it_on() {
        assert_eq!(classify_unencrypted_quic(None), UnencryptedQuicValue::Unset);
        assert_eq!(
            classify_unencrypted_quic(Some("")),
            UnencryptedQuicValue::Unset
        );
        assert_eq!(
            classify_unencrypted_quic(Some("on")),
            UnencryptedQuicValue::On
        );
        for v in ["ON", "On", "1", "true", "yes", " on", "on "] {
            assert_eq!(
                classify_unencrypted_quic(Some(v)),
                UnencryptedQuicValue::Unrecognised,
                "{v:?}"
            );
        }
        assert_eq!(UnencryptedQuicValue::On.switch(), UnencryptedQuic::On);
        assert_eq!(UnencryptedQuicValue::Unset.switch(), UnencryptedQuic::Off);
        assert_eq!(
            UnencryptedQuicValue::Unrecognised.switch(),
            UnencryptedQuic::Off
        );
    }

    /// Both env readers, through the real process environment. Serialised: no
    /// other test in this crate touches the variable.
    #[test]
    fn both_env_readers_turn_on_only_for_the_exact_value() {
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        let _g = LOCK.lock().unwrap_or_else(|p| p.into_inner());
        let saved = std::env::var_os(UNENCRYPTED_QUIC_ENV);
        let read = |v: Option<&str>| {
            match v {
                Some(v) => std::env::set_var(UNENCRYPTED_QUIC_ENV, v),
                None => std::env::remove_var(UNENCRYPTED_QUIC_ENV),
            }
            (
                unencrypted_quic_from_env(),
                unencrypted_quic_from_env_quiet(),
            )
        };
        let on = (UnencryptedQuic::On, UnencryptedQuic::On);
        let off = (UnencryptedQuic::Off, UnencryptedQuic::Off);
        let got = [
            read(Some("on")),
            read(Some("ON")),
            read(Some("1")),
            read(None),
        ];
        match saved {
            Some(v) => std::env::set_var(UNENCRYPTED_QUIC_ENV, v),
            None => std::env::remove_var(UNENCRYPTED_QUIC_ENV),
        }
        assert_eq!(got, [on, off, off, off]);
    }

    #[test]
    fn twins_map_the_address_and_nothing_else() {
        assert_eq!(
            udp_rel_twin_of_tcp("tcp/10.0.0.5:7683").as_deref(),
            Some("udp/10.0.0.5:7683?rel=1")
        );
        assert_eq!(
            udp_rel_twin_of_tcp("tcp/[::]:7683").as_deref(),
            Some("udp/[::]:7683?rel=1")
        );
        assert_eq!(udp_rel_twin_of_tcp("tcp/h:1?x=1"), None);
        assert_eq!(udp_rel_twin_of_tcp("tcp/h:1#iface=eth0"), None);
        assert_eq!(udp_rel_twin_of_tcp("tcp/"), None);
        assert_eq!(udp_rel_twin_of_tcp("quic/h:1"), None);
        assert_eq!(udp_rel_twin_of_tcp("udp/h:1?rel=1"), None);

        assert_eq!(
            tcp_twin_of_udp_rel("udp/10.0.0.5:7683?rel=1").as_deref(),
            Some("tcp/10.0.0.5:7683")
        );
        assert_eq!(tcp_twin_of_udp_rel("udp/h:1?rel=1;mixed_rel=1"), None);
        assert_eq!(tcp_twin_of_udp_rel("udp/h:1"), None);
        assert_eq!(tcp_twin_of_udp_rel("udp/?rel=1"), None);
        assert_eq!(tcp_twin_of_udp_rel("tcp/h:1"), None);
    }

    #[test]
    fn listen_twins_follow_each_tcp_entry_once() {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        assert_eq!(
            with_udp_rel_listeners(&s(&["tcp/0.0.0.0:7683"])),
            s(&["tcp/0.0.0.0:7683", "udp/0.0.0.0:7683?rel=1"])
        );
        // Already listed: not added again; non-tcp entries pass through.
        assert_eq!(
            with_udp_rel_listeners(&s(&[
                "udp/0.0.0.0:7683?rel=1",
                "tcp/0.0.0.0:7683",
                "quic/0.0.0.0:7700",
                "tcp/0.0.0.0:7683",
            ])),
            s(&[
                "udp/0.0.0.0:7683?rel=1",
                "tcp/0.0.0.0:7683",
                "quic/0.0.0.0:7700",
                "tcp/0.0.0.0:7683",
            ])
        );
        assert!(with_udp_rel_listeners(&[]).is_empty());
    }
}
