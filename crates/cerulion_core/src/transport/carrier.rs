// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-host carrier a zenoh locator names, the certificate files the
//! `quic/` carrier reads, and the per-topic egress reliability a gateway puts
//! with.
//!
//! The carrier is the locator's first segment, so one locator string is the
//! whole choice:
//!
//! | Locator | Carrier |
//! |---|---|
//! | `tcp/<host>:<port>` | TCP, the default |
//! | `quic/<host>:<port>` | QUIC over TLS 1.3; needs [`NetworkTls`] files |
//! | `quic/<host>:<port>?mixed_rel=1` | the same, plus a datagram link on the same connection |
//! | `udp/<host>:<port>?rel=1;mixed_rel=1` | QUIC with NO encryption and NO peer authentication |
//!
//! `?mixed_rel=1` asks zenoh 1.10 for one QUIC connection carrying two links: a
//! stream link (reliable, one stream per priority) and a datagram link (best
//! effort). A topic declared `reliability: best_effort` is put with zenoh's
//! best-effort marker, and zenoh's link selection sends it on the datagram link
//! when the connection has one. Without a mixed-reliability link every topic
//! rides the reliable carrier, which is why graph-load validation refuses a
//! best-effort topic on a graph that declares none.
//!
//! The `udp/...?rel=1` spelling leaves confidentiality and peer authentication
//! entirely to the network underneath (a WireGuard overlay).
//! Anywhere else, use `quic/`. A `quic/` connector checks the listener's
//! certificate; the listener does not ask for one back, so who may pull a topic
//! stays the demand authorizer's decision.

use std::path::PathBuf;

use serde::{Deserialize, Serialize};

/// Certificate files for the `quic/` carrier, mapped onto zenoh's
/// `transport/link/tls/*` keys (zenoh reads its QUIC certificates from the TLS
/// section). Every path must be absolute; the files are read when the zenoh
/// session opens, and a missing or unreadable file is a session-open error that
/// names the path.
///
/// | Field | zenoh key | Needed by |
/// |---|---|---|
/// | `root_ca` | `root_ca_certificate` | a `quic/` CONNECT locator: PEM trust anchors added to the ones zenoh always trusts (a self-signed peer certificate is its own anchor when it is not marked `CA:TRUE`) |
/// | `certificate` | `listen_certificate` | a `quic/` LISTEN locator: the PEM certificate this machine presents |
/// | `private_key` | `listen_private_key` | a `quic/` LISTEN locator: the PEM private key for `certificate` |
///
/// zenoh's QUIC client trusts the public Web PKI root set as well as
/// `root_ca` (zenoh-link-commons 1.10.1 `quic/utils.rs`, "mixed user-generated
/// CA and webPKI CA"), so a listener holding a publicly issued certificate for
/// the name or IP in the locator is also accepted. The name check
/// (`verify_name_on_connect`) is always on: with it off, any publicly issued
/// certificate for any name would pass.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NetworkTls {
    /// PEM file of trust anchors for the peers this machine connects to.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub root_ca: Option<PathBuf>,
    /// PEM certificate this machine presents on a `quic/` listen locator.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub certificate: Option<PathBuf>,
    /// PEM private key for [`Self::certificate`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub private_key: Option<PathBuf>,
}

impl NetworkTls {
    /// The `(zenoh config key, path)` pairs this block sets, in a fixed order.
    pub(crate) fn file_keys(&self) -> impl Iterator<Item = (&'static str, &PathBuf)> {
        [
            (
                "transport/link/tls/root_ca_certificate",
                self.root_ca.as_ref(),
            ),
            (
                "transport/link/tls/listen_certificate",
                self.certificate.as_ref(),
            ),
            (
                "transport/link/tls/listen_private_key",
                self.private_key.as_ref(),
            ),
        ]
        .into_iter()
        .filter_map(|(k, p)| p.map(|p| (k, p)))
    }
}

/// How a topic's frames are put on the network.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum EgressReliability {
    /// Every frame is delivered in order, retransmitted on loss. The default.
    #[default]
    Reliable,
    /// Frames are put with zenoh's best-effort marker: on a mixed-reliability
    /// QUIC connection they ride QUIC datagrams, so a lost frame is gone and a
    /// later frame is not held behind it. zenoh sizes datagrams once, at link
    /// setup; a path MTU that later shrinks below that size makes a datagram
    /// send fail, which closes the datagram link and the stream link with it.
    BestEffort,
}

/// One topic's egress put policy. Only non-default entries are carried.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EgressQos {
    /// Canonical absolute topic name.
    pub topic: String,
    /// How the topic's frames are put.
    pub reliability: EgressReliability,
}

/// A QUIC endpoint's `mixed_rel` metadata value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MixedRel {
    /// Absent or `0`: one stream link, no datagram link.
    Off,
    /// `1`: the connection carries a datagram link beside the stream link, or
    /// does not come up (zenoh offers only the mixed-reliability ALPN ids).
    On,
    /// `auto`: a datagram link only when the peer also offers mixed
    /// reliability; otherwise the connection comes up with streams only.
    Auto,
}

/// The carrier a zenoh locator selects, as far as cerulion distinguishes them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Carrier {
    /// `tcp/`.
    Tcp,
    /// `quic/`: TLS 1.3 QUIC.
    Quic {
        /// The endpoint's `mixed_rel` value.
        mixed_rel: MixedRel,
    },
    /// `udp/...?rel=1`: QUIC with encryption and peer authentication off.
    UnsecureQuic {
        /// The endpoint's `mixed_rel` value.
        mixed_rel: MixedRel,
    },
    /// Plain `udp/` (no `rel=1`): raw UDP, best effort only.
    Udp,
}

impl Carrier {
    /// True when a connection over this locator is certain to carry a
    /// datagram link (`mixed_rel=1`). `mixed_rel=auto` is false: it falls back
    /// to streams only when the peer does not offer mixed reliability.
    pub fn has_datagram_link(self) -> bool {
        matches!(
            self,
            Carrier::Quic {
                mixed_rel: MixedRel::On
            } | Carrier::UnsecureQuic {
                mixed_rel: MixedRel::On
            }
        )
    }
}

/// Classify a zenoh locator string by its carrier.
///
/// The grammar read here is zenoh's endpoint syntax: `<scheme>/<address>`, then
/// optional `?key=value;key=value` metadata, then optional `#` config. Only the
/// scheme and the `rel` / `mixed_rel` metadata keys are interpreted; every other
/// byte reaches zenoh unchanged.
///
/// # Errors
///
/// A human-readable reason when the scheme is not one this build compiles
/// (`tcp`, `quic`, `udp`), the address is empty, or `rel` / `mixed_rel` carry a
/// value zenoh would refuse at session open.
pub fn classify_locator(locator: &str) -> Result<Carrier, String> {
    let Some((scheme, rest)) = locator.split_once('/') else {
        return Err(format!(
            "locator '{locator}' has no carrier: write it as '<carrier>/<host>:<port>', \
             e.g. 'tcp/192.168.1.10:7447' or 'quic/192.168.1.10:7447'"
        ));
    };
    let before_config = rest.split_once('#').map_or(rest, |(a, _)| a);
    let (address, metadata) = before_config.split_once('?').unwrap_or((before_config, ""));
    if address.is_empty() {
        return Err(format!(
            "locator '{locator}' has no address after '{scheme}/'"
        ));
    }
    let mut rel = None;
    let mut mixed_rel = None;
    for pair in metadata.split(';').filter(|p| !p.is_empty()) {
        let (key, value) = pair.split_once('=').unwrap_or((pair, ""));
        match key {
            "rel" => rel = Some(value),
            "mixed_rel" => mixed_rel = Some(value),
            _ => {}
        }
    }
    let mixed = match mixed_rel {
        None | Some("0") => MixedRel::Off,
        Some("1") => MixedRel::On,
        Some("auto") => MixedRel::Auto,
        Some(other) => {
            return Err(format!(
                "locator '{locator}' sets mixed_rel={other}; zenoh accepts 0, 1 or auto"
            ))
        }
    };
    match scheme {
        "tcp" => {
            if mixed != MixedRel::Off {
                return Err(format!(
                    "locator '{locator}' sets mixed_rel on a tcp/ carrier; mixed reliability \
                     exists only on quic/ and udp/...?rel=1"
                ));
            }
            Ok(Carrier::Tcp)
        }
        "quic" => Ok(Carrier::Quic { mixed_rel: mixed }),
        "udp" => match rel {
            Some("1") => Ok(Carrier::UnsecureQuic { mixed_rel: mixed }),
            None | Some("0") if mixed == MixedRel::Off => Ok(Carrier::Udp),
            None | Some("0") => Err(format!(
                "locator '{locator}' sets mixed_rel on plain udp/; add rel=1 \
                 ('udp/<host>:<port>?rel=1;mixed_rel=1') for QUIC over UDP"
            )),
            Some(other) => Err(format!(
                "locator '{locator}' sets rel={other}; zenoh accepts rel=0 or rel=1"
            )),
        },
        "tls" => Err(format!(
            "locator '{locator}' names the tls/ carrier, which this build does not compile; \
             use quic/ for an encrypted link or tcp/"
        )),
        other => Err(format!(
            "locator '{locator}' names the '{other}' carrier, which this build does not \
             compile; the carriers are tcp/, quic/ and udp/"
        )),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_reads_scheme_and_reliability_metadata() {
        assert_eq!(classify_locator("tcp/0.0.0.0:7447"), Ok(Carrier::Tcp));
        assert_eq!(
            classify_locator("quic/10.0.0.2:7447"),
            Ok(Carrier::Quic {
                mixed_rel: MixedRel::Off
            })
        );
        assert_eq!(
            classify_locator("quic/10.0.0.2:7447?mixed_rel=1"),
            Ok(Carrier::Quic {
                mixed_rel: MixedRel::On
            })
        );
        assert_eq!(
            classify_locator("quic/10.0.0.2:7447?mixed_rel=auto#initial_mtu=1200"),
            Ok(Carrier::Quic {
                mixed_rel: MixedRel::Auto
            })
        );
        assert_eq!(
            classify_locator("udp/10.0.0.3:7684?rel=1;mixed_rel=1"),
            Ok(Carrier::UnsecureQuic {
                mixed_rel: MixedRel::On
            })
        );
        assert_eq!(
            classify_locator("udp/10.0.0.3:7684?rel=1"),
            Ok(Carrier::UnsecureQuic {
                mixed_rel: MixedRel::Off
            })
        );
        assert_eq!(classify_locator("udp/10.0.0.3:7684"), Ok(Carrier::Udp));
        // An explicit `mixed_rel=0` is what zenoh ignores, so it is accepted
        // on every carrier.
        assert_eq!(classify_locator("tcp/h:1?mixed_rel=0"), Ok(Carrier::Tcp));
        assert_eq!(classify_locator("udp/h:1?mixed_rel=0"), Ok(Carrier::Udp));
        // A `#` config section is not metadata: a `mixed_rel` there is not read.
        assert_eq!(
            classify_locator("quic/h:1#mixed_rel=1"),
            Ok(Carrier::Quic {
                mixed_rel: MixedRel::Off
            })
        );
    }

    #[test]
    fn only_mixed_reliability_quic_has_a_datagram_link() {
        for (loc, want) in [
            ("tcp/h:1", false),
            ("quic/h:1", false),
            ("quic/h:1?mixed_rel=0", false),
            ("quic/h:1?mixed_rel=1", true),
            ("quic/h:1?mixed_rel=auto", false),
            ("udp/h:1?rel=1;mixed_rel=auto", false),
            ("udp/h:1?rel=1;mixed_rel=1", true),
            ("udp/h:1?rel=1", false),
            ("udp/h:1", false),
        ] {
            assert_eq!(
                classify_locator(loc).unwrap().has_datagram_link(),
                want,
                "{loc}"
            );
        }
    }

    #[test]
    fn classify_refuses_what_zenoh_would_refuse_or_this_build_lacks() {
        for (loc, needle) in [
            ("192.168.1.10:7447", "no carrier"),
            ("quic/", "no address"),
            ("quic/h:1?mixed_rel=yes", "mixed_rel=yes"),
            ("tcp/h:1?mixed_rel=1", "tcp/ carrier"),
            ("udp/h:1?mixed_rel=1", "plain udp/"),
            ("udp/h:1?rel=2", "rel=2"),
            ("tls/h:1", "tls/ carrier"),
            ("ws/h:1", "'ws' carrier"),
        ] {
            let err = classify_locator(loc).expect_err(loc);
            assert!(err.contains(needle), "{loc}: {err}");
        }
    }

    #[test]
    fn tls_file_keys_map_onto_zenoh_tls_section_in_order() {
        let tls = NetworkTls {
            root_ca: Some("/c/ca.pem".into()),
            certificate: None,
            private_key: Some("/c/key.pem".into()),
        };
        let keys: Vec<_> = tls.file_keys().map(|(k, p)| (k, p.clone())).collect();
        assert_eq!(
            keys,
            vec![
                (
                    "transport/link/tls/root_ca_certificate",
                    PathBuf::from("/c/ca.pem")
                ),
                (
                    "transport/link/tls/listen_private_key",
                    PathBuf::from("/c/key.pem")
                ),
            ]
        );
    }

    #[test]
    fn tls_block_parses_and_refuses_unknown_keys() {
        let tls: NetworkTls = serde_yaml::from_str("root_ca: /c/ca.pem\n").unwrap();
        assert_eq!(tls.root_ca, Some(PathBuf::from("/c/ca.pem")));
        let err = serde_yaml::from_str::<NetworkTls>("root_cert: /c/ca.pem\n").unwrap_err();
        assert!(err.to_string().contains("root_cert"), "{err}");
        // There is no switch for the name check: a key for it is refused.
        let err = serde_yaml::from_str::<NetworkTls>("verify_name: false\n").unwrap_err();
        assert!(err.to_string().contains("verify_name"), "{err}");
    }

    #[test]
    fn reliability_spells_snake_case() {
        assert_eq!(
            serde_yaml::from_str::<EgressReliability>("best_effort").unwrap(),
            EgressReliability::BestEffort
        );
        assert_eq!(
            serde_yaml::from_str::<EgressReliability>("reliable").unwrap(),
            EgressReliability::Reliable
        );
        assert!(serde_yaml::from_str::<EgressReliability>("besteffort").is_err());
    }
}
