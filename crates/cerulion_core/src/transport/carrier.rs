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
//! | `udp/<host>:<port>`, `udp/...?rel=0`, `quic/...?rel=0` | datagram only: refused by [`check_carrier_settings`] |
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

/// The names a user wrote each carrier setting under, quoted in the messages of
/// [`check_carrier_settings`] and [`check_tls_files`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CarrierSettingNames {
    /// The listen locator list (`network listen`, `CERULION_NETD_LISTEN`).
    pub listen: &'static str,
    /// The connect locator list (`network connect`, `CERULION_NETD_CONNECT`).
    pub connect: &'static str,
    /// [`NetworkTls::root_ca`].
    pub root_ca: &'static str,
    /// [`NetworkTls::certificate`].
    pub certificate: &'static str,
    /// [`NetworkTls::private_key`].
    pub private_key: &'static str,
}

impl CarrierSettingNames {
    /// The graph YAML `network:` block's spellings.
    pub const GRAPH: Self = Self {
        listen: "network listen",
        connect: "network connect",
        root_ca: "network tls.root_ca",
        certificate: "network tls.certificate",
        private_key: "network tls.private_key",
    };
}

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
        self.files(&CarrierSettingNames::GRAPH)
            .map(|(_, key, path)| (key, path))
    }

    /// The `(user-facing name, zenoh config key, path)` triples this block
    /// sets, in the order of [`Self::file_keys`].
    fn files<'a>(
        &'a self,
        names: &CarrierSettingNames,
    ) -> impl Iterator<Item = (&'static str, &'static str, &'a PathBuf)> {
        [
            (
                names.root_ca,
                "transport/link/tls/root_ca_certificate",
                self.root_ca.as_ref(),
            ),
            (
                names.certificate,
                "transport/link/tls/listen_certificate",
                self.certificate.as_ref(),
            ),
            (
                names.private_key,
                "transport/link/tls/listen_private_key",
                self.private_key.as_ref(),
            ),
        ]
        .into_iter()
        .filter_map(|(n, k, p)| p.map(|p| (n, k, p)))
    }
}

/// Read every file `tls` names and refuse what zenoh would reject later with a
/// less direct error: a relative path, an unreadable file, a file with no PEM
/// block (`-----BEGIN `), and a listener certificate marked `CA:TRUE`, which
/// the connecting side's webpki rejects as an end entity
/// (`CaUsedAsEndEntity`). Each message starts with the setting's name from
/// `names` and the path. Whether the PEM is the right certificate or key is
/// zenoh's check, at handshake.
///
/// # Errors
///
/// The first refusal, as a human-readable message.
// hot-path-alloc-ok-fn: session-open and daemon-start validation of certificate files, never frame delivery.
pub fn check_tls_files(tls: &NetworkTls, names: &CarrierSettingNames) -> Result<(), String> {
    for (name, key, path) in tls.files(names) {
        let refuse = |why: String| format!("{name} '{}' {why}", path.display());
        if !path.is_absolute() {
            return Err(refuse(
                "is not an absolute path; tls paths are read by the gateway process, whose \
                 working directory is not the one the setting was written in"
                    .to_string(),
            ));
        }
        let bytes = std::fs::read(path).map_err(|e| refuse(format!("cannot be read: {e}")))?;
        if !bytes.windows(11).any(|w| w == b"-----BEGIN ") {
            return Err(refuse("holds no PEM block (-----BEGIN ...)".to_string()));
        }
        if key == "transport/link/tls/listen_certificate" {
            refuse_ca_certificate(&bytes).map_err(refuse)?;
        }
        if path.to_str().is_none() {
            return Err(refuse("is not valid UTF-8".to_string()));
        }
    }
    Ok(())
}

/// A `quic/` listener's certificate (the first CERTIFICATE block of the file)
/// must not be marked `CA:TRUE`: the connecting side validates it with webpki,
/// which rejects a CA certificate presented as the end entity
/// (`CaUsedAsEndEntity`), and that failure surfaces only as a WARN in the
/// listener's log while the connector retries. `openssl req -x509` writes
/// `CA:TRUE` unless told otherwise, so the error names the flag that fixes it.
// hot-path-alloc-ok-fn: certificate parsing at session open or daemon start, never frame delivery.
fn refuse_ca_certificate(pem: &[u8]) -> Result<(), String> {
    let block = x509_parser::pem::Pem::iter_from_buffer(pem)
        .filter_map(Result::ok)
        .find(|p| p.label == "CERTIFICATE")
        .ok_or_else(|| "holds no CERTIFICATE PEM block".to_string())?;
    let cert = block
        .parse_x509()
        .map_err(|e| format!("holds a CERTIFICATE block that does not parse: {e}"))?;
    let is_ca = cert
        .basic_constraints()
        .map_err(|e| format!("has an unreadable basicConstraints extension: {e}"))?
        .is_some_and(|bc| bc.value.ca);
    if is_ca {
        return Err(
            "is marked CA:TRUE, and a quic/ peer refuses a CA certificate as the \
             listener's own certificate (CaUsedAsEndEntity); mint it as an end entity, \
             e.g. add `-addext basicConstraints=critical,CA:FALSE` to `openssl req -x509`"
                .to_string(),
        );
    }
    Ok(())
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

/// One topic's egress put policy. A graph's `network:` block carries only its
/// best-effort topics; a topic with no entry is put reliable.
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
    /// `quic/...?rel=0`: zenoh's QUIC datagram link alone, with no stream link,
    /// so best effort only.
    QuicDatagram,
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

    /// True when the link has no reliable channel at all: zenoh sends every
    /// message on it once, with no retransmission, whatever the message's
    /// reliability. That covers each reliable topic and zenoh's own
    /// declarations, liveliness tokens and queries, which ride the same link.
    pub fn is_datagram_only(self) -> bool {
        matches!(self, Carrier::Udp | Carrier::QuicDatagram)
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
// hot-path-alloc-ok-fn: graph-load validation of the network block, never frame delivery.
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
        "quic" => match rel {
            None | Some("1") => Ok(Carrier::Quic { mixed_rel: mixed }),
            Some("0") => Ok(Carrier::QuicDatagram),
            Some(other) => Err(format!(
                "locator '{locator}' sets rel={other}; zenoh accepts rel=0 or rel=1"
            )),
        },
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

/// The settings-level carrier rules shared by graph-load validation and
/// `cerulion-netd` start-up. `names` supplies the setting names each message
/// quotes; `best_effort_topic` is one egress topic declared best effort, when
/// there is one.
///
/// 1. Every locator classifies under [`classify_locator`].
/// 2. No locator is datagram only ([`Carrier::is_datagram_only`]).
/// 3. A `tls` block sets at least one file; `root_ca` needs a `quic/` connect
///    locator, and `certificate` / `private_key` a `quic/` listen locator (zenoh
///    asks a connecting peer for no certificate). The certificate and key are
///    set together, and every path is absolute.
/// 4. A `quic/` listen locator needs `certificate` and `private_key`; a `quic/`
///    connect locator needs `root_ca`.
/// 5. A best-effort topic needs a locator with `mixed_rel=1`.
///
/// # Errors
///
/// The first refusal, as a human-readable message naming the setting and the
/// fix.
// hot-path-alloc-ok-fn: graph-load and daemon-start validation of locators, never frame delivery.
pub fn check_carrier_settings(
    listen: &[String],
    connect: &[String],
    tls: Option<&NetworkTls>,
    best_effort_topic: Option<&str>,
    names: &CarrierSettingNames,
) -> Result<(), String> {
    let mut quic_listen = false;
    let mut quic_connect = false;
    let mut datagram_link = false;
    for (list, locators, is_listen) in [
        (names.listen, listen, true),
        (names.connect, connect, false),
    ] {
        for loc in locators {
            let carrier = classify_locator(loc).map_err(|reason| format!("{list}: {reason}"))?;
            if carrier.is_datagram_only() {
                return Err(format!(
                    "{list}: locator '{loc}' is a datagram-only link (plain udp/, or rel=0): \
                     zenoh sends every message on it once with no retransmission, so a lost \
                     packet loses a reliable topic's frame or one of zenoh's own declarations; \
                     use 'udp/<host>:<port>?rel=1;mixed_rel=1' (QUIC over UDP: reliable \
                     streams, plus datagrams for best_effort topics), \
                     'quic/<host>:<port>?mixed_rel=1', or tcp/"
                ));
            }
            if matches!(carrier, Carrier::Quic { .. }) {
                if is_listen {
                    quic_listen = true;
                } else {
                    quic_connect = true;
                }
            }
            datagram_link |= carrier.has_datagram_link();
        }
    }

    if let Some(tls) = tls {
        if *tls == NetworkTls::default() {
            return Err(format!(
                "the tls block sets none of {} / {} / {}, so nothing reads it; remove it, or set \
                 the files a quic/ locator needs",
                names.root_ca, names.certificate, names.private_key
            ));
        }
        if tls.root_ca.is_some() && !quic_connect {
            return Err(format!(
                "{} is set but no {} locator is quic/, so it would change nothing (a quic/ \
                 listener asks a connecting peer for no certificate); remove it or add a \
                 quic/ connect locator",
                names.root_ca, names.connect
            ));
        }
        if (tls.certificate.is_some() || tls.private_key.is_some()) && !quic_listen {
            return Err(format!(
                "{} / {} are set but no {} locator is quic/, so they would change nothing \
                 (only a quic/ listener presents them); remove them or add a quic/ listen \
                 locator",
                names.certificate, names.private_key, names.listen
            ));
        }
        for (name, _, p) in tls.files(names) {
            if !p.is_absolute() {
                return Err(format!(
                    "{name} '{}' is not an absolute path; tls paths are read by the gateway \
                     process, whose working directory is not the one the setting was written in",
                    p.display()
                ));
            }
        }
        if tls.certificate.is_some() != tls.private_key.is_some() {
            return Err(format!(
                "one of {} / {} is set without the other; a quic/ listener presents the \
                 certificate and signs with its key, so set both",
                names.certificate, names.private_key
            ));
        }
    }
    if quic_listen && tls.is_none_or(|t| t.certificate.is_none()) {
        return Err(format!(
            "{} has a quic/ locator but no {} / {}; a quic/ listener must present a \
             certificate (or use tcp/, or udp/<ip>:<port>?rel=1;mixed_rel=1 on a WireGuard \
             overlay)",
            names.listen, names.certificate, names.private_key
        ));
    }
    if quic_connect && tls.is_none_or(|t| t.root_ca.is_none()) {
        return Err(format!(
            "{} has a quic/ locator but no {}; set it to the PEM certificate(s) the peer's \
             certificate chains to (a self-signed peer certificate is its own root)",
            names.connect, names.root_ca
        ));
    }

    if let Some(topic) = best_effort_topic {
        if !datagram_link {
            return Err(format!(
                "egress topic '{topic}' is reliability: best_effort, but no {} or {} \
                 locator is certain to have a datagram link (mixed_rel=auto falls back to \
                 streams when the peer does not offer it), so it would ride the reliable link; \
                 add mixed_rel=1 to a quic/ locator ('quic/<host>:<port>?mixed_rel=1') or use \
                 'udp/<host>:<port>?rel=1;mixed_rel=1'",
                names.listen, names.connect
            ));
        }
    }
    Ok(())
}

/// The locators in `listen` and `connect` that carry no datagram link, so a
/// best-effort topic sent to a peer over one of them rides its reliable link.
/// Unclassifiable locators are skipped ([`check_carrier_settings`] refuses
/// them).
// hot-path-alloc-ok-fn: graph-load note, never frame delivery.
pub fn stream_only_locators<'a>(listen: &'a [String], connect: &'a [String]) -> Vec<&'a str> {
    listen
        .iter()
        .chain(connect)
        .filter(|l| classify_locator(l).is_ok_and(|c| !c.has_datagram_link()))
        .map(String::as_str)
        .collect()
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
        // On quic/, rel=0 selects zenoh's datagram-only QUIC link and rel=1
        // is the default stream link.
        assert_eq!(
            classify_locator("quic/10.0.0.2:7447?rel=0"),
            Ok(Carrier::QuicDatagram)
        );
        assert_eq!(
            classify_locator("quic/10.0.0.2:7447?rel=1;mixed_rel=1"),
            Ok(Carrier::Quic {
                mixed_rel: MixedRel::On
            })
        );
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
            ("quic/h:1?rel=yes", "rel=yes"),
            ("tls/h:1", "tls/ carrier"),
            ("ws/h:1", "'ws' carrier"),
        ] {
            let err = classify_locator(loc).expect_err(loc);
            assert!(err.contains(needle), "{loc}: {err}");
        }
    }

    #[test]
    fn only_plain_udp_and_quic_rel0_are_datagram_only() {
        for (loc, want) in [
            ("tcp/h:1", false),
            ("quic/h:1", false),
            ("quic/h:1?rel=1", false),
            ("quic/h:1?mixed_rel=1", false),
            ("quic/h:1?rel=0", true),
            ("udp/h:1?rel=1;mixed_rel=1", false),
            ("udp/h:1?rel=1", false),
            ("udp/h:1", true),
            ("udp/h:1?rel=0", true),
        ] {
            assert_eq!(
                classify_locator(loc).unwrap().is_datagram_only(),
                want,
                "{loc}"
            );
        }
    }

    const NAMES: CarrierSettingNames = CarrierSettingNames {
        listen: "LISTEN",
        connect: "CONNECT",
        root_ca: "ROOT_CA",
        certificate: "CERT",
        private_key: "KEY",
    };

    fn locs(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| s.to_string()).collect()
    }

    /// Every message quotes the caller's names, not the graph spelling.
    #[test]
    fn carrier_settings_refusals_quote_the_callers_names() {
        let root_ca = NetworkTls {
            root_ca: Some("/c/ca.pem".into()),
            ..NetworkTls::default()
        };
        let listener = NetworkTls {
            certificate: Some("/c/a.pem".into()),
            private_key: Some("/c/a.key".into()),
            ..NetworkTls::default()
        };
        for (listen, connect, tls, needles) in [
            (
                locs(&["udp/0.0.0.0:7684"]),
                locs(&[]),
                None,
                &["LISTEN: locator 'udp/0.0.0.0:7684' is a datagram-only link"][..],
            ),
            (
                locs(&[]),
                locs(&["quic/10.0.0.2:7447"]),
                None,
                &["CONNECT has a quic/ locator but no ROOT_CA"][..],
            ),
            (
                locs(&["quic/0.0.0.0:7447"]),
                locs(&[]),
                None,
                &["LISTEN has a quic/ locator but no CERT / KEY"][..],
            ),
            (
                locs(&["tcp/0.0.0.0:7447"]),
                locs(&[]),
                Some(root_ca.clone()),
                &["ROOT_CA is set but no CONNECT locator is quic/"][..],
            ),
            (
                locs(&[]),
                locs(&["quic/10.0.0.2:7447"]),
                Some(NetworkTls {
                    root_ca: Some("/c/ca.pem".into()),
                    ..listener.clone()
                }),
                &["CERT / KEY are set but no LISTEN locator is quic/"][..],
            ),
            (
                locs(&["quic/0.0.0.0:7447"]),
                locs(&[]),
                Some(NetworkTls {
                    private_key: None,
                    ..listener.clone()
                }),
                &["one of CERT / KEY is set without the other"][..],
            ),
            (
                locs(&[]),
                locs(&["quic/10.0.0.2:7447"]),
                Some(NetworkTls {
                    root_ca: Some("certs/ca.pem".into()),
                    ..NetworkTls::default()
                }),
                &["ROOT_CA 'certs/ca.pem' is not an absolute path"][..],
            ),
        ] {
            let err = check_carrier_settings(&listen, &connect, tls.as_ref(), None, &NAMES)
                .expect_err(&format!("{listen:?} {connect:?} {tls:?}"));
            for needle in needles {
                assert!(err.contains(needle), "want '{needle}' in: {err}");
            }
        }
        // The accepted shapes, as a control on the same names.
        check_carrier_settings(
            &locs(&["quic/0.0.0.0:7447?mixed_rel=1"]),
            &locs(&["tcp/10.0.0.3:7447"]),
            Some(&listener),
            Some("/r/scan"),
            &NAMES,
        )
        .unwrap();
        check_carrier_settings(
            &locs(&[]),
            &locs(&["quic/10.0.0.2:7447"]),
            Some(&root_ca),
            None,
            &NAMES,
        )
        .unwrap();
        check_carrier_settings(&locs(&["tcp/0.0.0.0:7447"]), &[], None, None, &NAMES).unwrap();
    }

    #[test]
    fn stream_only_locators_lists_the_locators_with_no_datagram_link() {
        assert_eq!(
            stream_only_locators(
                &locs(&["quic/0.0.0.0:7447?mixed_rel=1", "tcp/0.0.0.0:7448"]),
                &locs(&["quic/10.0.0.2:7447", "udp/10.0.0.3:7684?rel=1;mixed_rel=1"]),
            ),
            vec!["tcp/0.0.0.0:7448", "quic/10.0.0.2:7447"]
        );
    }

    /// File checks name the caller's setting and refuse a relative path, a
    /// missing file and a file with no PEM block.
    #[test]
    fn tls_file_checks_name_the_setting_and_the_path() {
        let rel = NetworkTls {
            root_ca: Some("ca.pem".into()),
            ..NetworkTls::default()
        };
        let err = check_tls_files(&rel, &NAMES).unwrap_err();
        assert!(
            err.starts_with("ROOT_CA 'ca.pem' is not an absolute path"),
            "{err}"
        );
        let missing = NetworkTls {
            certificate: Some("/nonexistent/cerulion-carrier-test/a.pem".into()),
            ..NetworkTls::default()
        };
        let err = check_tls_files(&missing, &NAMES).unwrap_err();
        assert!(
            err.starts_with("CERT '/nonexistent/cerulion-carrier-test/a.pem' cannot be read"),
            "{err}"
        );
        // `Cargo.toml` exists at a known absolute path and holds no PEM block.
        let not_pem = NetworkTls {
            private_key: Some(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml")),
            ..NetworkTls::default()
        };
        let err = check_tls_files(&not_pem, &NAMES).unwrap_err();
        assert!(err.starts_with("KEY '"), "{err}");
        assert!(err.contains("holds no PEM block"), "{err}");
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
