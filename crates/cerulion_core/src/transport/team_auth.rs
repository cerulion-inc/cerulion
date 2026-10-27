// SPDX-License-Identifier: AGPL-3.0-only
//! Mutually authenticated team sessions: the zenoh configuration a machine
//! with team credentials opens its network session with.
//!
//! A team machine holds its machine key (one ed25519 seed per machine,
//! `cerulion_pairing::identity`), a leaf certificate over that key issued by
//! the team issuing CA, and the team's trust anchors ([`TeamAnchors`]): the
//! issuing CA of the current generation, the CAs of retired generations, and
//! the CA the next rotation will use (`cerulion_pairing::team`). The leaf
//! common name `<role>.g<N>.<team>.cerulion.invalid` names the role.
//!
//! [`TeamAuth::new`] checks those with the `cerulion_pairing` profile checkers.
//! One machine opens two sessions from them, on separate listeners:
//!
//! - the **data session** ([`NetworkAuth::Team`]) trusts the current issuing
//!   CA and the next generation's CA, and carries every topic, query and
//!   liveliness key under the role policy ([`access_control`]);
//! - the **bundle session** ([`NetworkAuth::TeamBundleLane`]) trusts the
//!   retired generations' CAs and the next generation's CA, and its policy
//!   ([`bundle_lane_access_control`]) admits only the bundle exchange on
//!   `cerulion_auth/bundle`. A leaf a retired CA signed, whatever name it
//!   carries, fails the data session's handshake and reaches no
//!   `cerulion_w2`, `cerulion_lv`, `cerulion_ann` or `cerulion_q` key.
//!
//! Each session sets, on its zenoh config:
//!
//! | zenoh key | value |
//! |---|---|
//! | `transport/link/tls/enable_mtls` | `true`: both ends present and verify a certificate |
//! | `transport/link/tls/{listen,connect}_certificate_base64` | the leaf, PEM |
//! | `transport/link/tls/{listen,connect}_private_key_base64` | the machine key, PKCS#8 PEM built in memory |
//! | `transport/link/tls/root_ca_certificate_base64` | the session's anchors (see above), PEM |
//! | `transport/link/tls/verify_name_on_connect` | `false`: the dialed host name or IP is not compared with the certificate |
//! | `transport/link/tls/close_link_on_expiration` | `true`: a link closes when the peer leaf's notAfter passes |
//! | `transport/link/protocols` | `["quic", "tls"]`: zenoh refuses any other link, including a gossip-learned one |
//! | `scouting/{gossip,multicast}/autoconnect` | empty for routers and peers: a scouted or gossip-learned locator is never dialed; routing through linked peers is unchanged |
//! | `access_control` | the session's policy, default deny, keyed on the peer leaf common name |
//!
//! The `*_base64` values are zenoh `SecretValue`s: zenoh does not serialize
//! them and redacts them in `Debug`. Nothing here writes key material to disk,
//! the environment or a log line; [`TeamAuth`]'s `Debug` prints the common
//! name and the machine key fingerprint.
//!
//! zenoh's `quic/` and `tls/` links read the same `transport/link/tls`
//! section, so both links carry the same identity. A team session refuses
//! every other listen or connect locator, and any locator with a `#` endpoint
//! config, which zenoh would merge over that section ([`check_team_locator`]).
//!
//! ## Who a peer is
//!
//! A listener verifies a connecting peer against the session's anchors only.
//! A dialer verifies the listener against the session's anchors and also
//! against the public Web PKI roots, which zenoh's TLS and QUIC clients always
//! load, with no name check. A listener whose certificate chains to the Web
//! PKI therefore links, but its common name is not a team name, so it matches
//! no subject and the default deny drops every message the policy filters in
//! both directions.
//!
//! zenoh matches a subject's `cert_common_names` against the peer leaf's
//! common name by exact string equality, and does not compare the name with
//! the issuing CA. In the data session either of its two anchors can therefore
//! issue a leaf that names any role: the next generation's CA key is held by
//! the issuer like the current one. A retired CA is never a data-session
//! anchor.
//!
//! ## Access policy
//!
//! The policy lists the five role names of each admitted generation. Rules
//! for a peer whose leaf names a role of the current generation (ingress is
//! what the peer may send, egress what this machine may send to it):
//!
//! | Role | Data (`cerulion_w2/**`) | Control (`cerulion_lv`, `cerulion_ann`, `cerulion_q`) | `cerulion_auth/**` |
//! |---|---|---|---|
//! | viewer, ci | receives puts; declares subscribers | declares demand tokens, queries, reads liveliness and receives tokens | bundle exchange |
//! | operator | viewer, plus put and delete | viewer, plus announce tokens and answering queries | bundle exchange |
//! | owner, robot | operator | every control message | everything |
//!
//! In zenoh peer mode a writer forwards a put only to subscribers it has seen
//! declared, and a query reaches only queryables the asker has seen, so a
//! writer's peer also gets egress `declare_subscriber`, and a reader egress
//! `declare_queryable`. A liveliness query is an interest message and its
//! answers are liveliness tokens in the other flow.
//!
//! In the data session a leaf of the next generation gets the bundle exchange
//! only. In the bundle session every admitted name (retired and next
//! generations) gets the bundle exchange only: it may query
//! `cerulion_auth/bundle`, answer this machine's bundle query, and nothing
//! else. A holder of a retired leaf, or of a retired CA key, can therefore
//! answer this machine's bundle query; a consumer of the reply checks it with
//! `cerulion_pairing::team::verify_team_trust_bundle` against the pinned root
//! set before using it. No consumer is in this module.

use base64::Engine as _;
use cerulion_pairing::identity::MachineKey;
use cerulion_pairing::team::{
    self, check_team_ca, verify_leaf_issued_by, LeafFacts, RetiredGeneration, TeamRole, TeamShort,
};
use cerulion_pairing::PairingError;
use serde_json::{json, Value};
use zeroize::Zeroizing;

use super::carrier::{classify_locator, Carrier};
use super::cerulion_q::CERULION_Q_PREFIX;
use super::discovery::{announce_prefix, liveliness_prefix};
use super::network::DATA_KEY_NAMESPACE;
use crate::error::{TransportError, TransportResult};

/// The key namespace of team trust traffic (the bundle query).
pub const TEAM_AUTH_PREFIX: &str = "cerulion_auth";

/// The key a node queries for a newer team trust bundle.
pub const TEAM_BUNDLE_KEY: &str = "cerulion_auth/bundle";

/// How a network session authenticates its links.
#[derive(Clone, Debug)]
pub enum NetworkAuth {
    /// The data session: mutual TLS with team credentials on `quic/` and
    /// `tls/` links, anchored on the current and next issuing CAs.
    Team(TeamAuth),
    /// The bundle session: the same identity, anchored on the retired and
    /// next issuing CAs, admitting only the bundle exchange.
    TeamBundleLane(TeamAuth),
}

/// Why team credentials cannot open a session.
#[derive(Debug, thiserror::Error)]
pub enum TeamAuthError {
    /// The leaf or a CA certificate fails the team profile or its signature.
    #[error("team {what} certificate is not usable: {source}")]
    Certificate {
        /// Which certificate: `leaf`, `issuing CA`, `retired CA g<N>` or `next CA`.
        what: String,
        /// The profile or signature failure.
        #[source]
        source: PairingError,
    },
    /// The leaf certifies a key other than this machine's key.
    #[error(
        "the team leaf certifies key {leaf_fingerprint}, but this machine's key is \
         {machine_fingerprint}; the leaf was issued to another machine or before the key changed"
    )]
    KeyMismatch {
        /// Short fingerprint of the key in the leaf.
        leaf_fingerprint: String,
        /// Short fingerprint of this machine's key.
        machine_fingerprint: String,
    },
    /// A retired or next CA names another team or a generation out of place.
    #[error("{which} CA g{generation}: {reason}")]
    Anchor {
        /// `retired` or `next`.
        which: &'static str,
        /// The generation number as written or as read.
        generation: u32,
        /// What is wrong with it.
        reason: &'static str,
    },
}

/// A team's trust anchors as a machine holds them.
#[derive(Clone, Copy, Debug)]
pub struct TeamAnchors<'a> {
    /// The issuing CA of the current generation, DER.
    pub issuing_ca_der: &'a [u8],
    /// The issuing CAs of retired generations, each below the current one.
    pub retired: &'a [RetiredGeneration],
    /// The issuing CA of the next generation, published before the rotation
    /// that starts it, DER.
    pub next_ca_der: Option<&'a [u8]>,
}

/// A machine's team credentials, checked and held as the PEM text zenoh reads.
#[derive(Clone)]
pub struct TeamAuth {
    leaf: LeafFacts,
    fingerprint: String,
    retired_generations: Vec<u32>,
    next_generation: Option<u32>,
    leaf_pem_b64: String,
    key_pem_b64: Zeroizing<String>,
    data_roots_pem_b64: String,
    lane_roots_pem_b64: Option<String>,
}

impl std::fmt::Debug for TeamAuth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TeamAuth")
            .field("common_name", &self.common_name())
            .field("fingerprint", &self.fingerprint)
            .field("retired_generations", &self.retired_generations)
            .field("next_generation", &self.next_generation)
            .finish_non_exhaustive()
    }
}

impl TeamAuth {
    /// Check and hold team credentials.
    ///
    /// `leaf_der` must satisfy the team leaf profile, be signed by
    /// `anchors.issuing_ca_der` (same team and generation) and certify `key`'s
    /// public key. Each retired CA must satisfy the CA profile, name the same
    /// team and a generation below the current one, at most once. The next CA
    /// must satisfy the CA profile and name the same team and the generation
    /// after the current one.
    ///
    /// Where these certificates come from, and whether the CAs chain to a
    /// trusted root set, is the caller's check: this only builds the session
    /// identity from them.
    ///
    /// # Errors
    ///
    /// [`TeamAuthError`] naming the certificate that fails.
    pub fn new(
        key: &MachineKey,
        leaf_der: &[u8],
        anchors: TeamAnchors<'_>,
    ) -> Result<TeamAuth, TeamAuthError> {
        let cert_err = |what: &str| {
            let what = what.to_string();
            move |source| TeamAuthError::Certificate { what, source }
        };
        let ca = check_team_ca(anchors.issuing_ca_der).map_err(cert_err("issuing CA"))?;
        let leaf = verify_leaf_issued_by(leaf_der, &ca).map_err(cert_err("leaf"))?;
        if leaf.device_key.0 != key.public_key() {
            return Err(TeamAuthError::KeyMismatch {
                leaf_fingerprint: short_fingerprint(&leaf.device_key.0),
                machine_fingerprint: key.short_fingerprint(),
            });
        }
        let mut retired_generations = Vec::with_capacity(anchors.retired.len());
        let mut data_roots = pem("CERTIFICATE", anchors.issuing_ca_der);
        let mut lane_roots = String::new();
        for r in anchors.retired {
            let facts = check_team_ca(&r.ca_der)
                .map_err(cert_err(&format!("retired CA g{}", r.generation)))?;
            let refuse = |reason| TeamAuthError::Anchor {
                which: "retired",
                generation: r.generation,
                reason,
            };
            if facts.generation != r.generation {
                return Err(refuse("the certificate names another generation"));
            }
            if facts.team != leaf.team {
                return Err(refuse("the certificate names another team"));
            }
            if r.generation >= leaf.generation {
                return Err(refuse("a retired generation must be below the current one"));
            }
            if retired_generations.contains(&r.generation) {
                return Err(refuse("the generation is listed twice"));
            }
            retired_generations.push(r.generation);
            lane_roots.push_str(&pem("CERTIFICATE", &r.ca_der));
        }
        let mut next_generation = None;
        if let Some(next) = anchors.next_ca_der {
            let facts = check_team_ca(next).map_err(cert_err("next CA"))?;
            let refuse = |reason| TeamAuthError::Anchor {
                which: "next",
                generation: facts.generation,
                reason,
            };
            if facts.team != leaf.team {
                return Err(refuse("the certificate names another team"));
            }
            if Some(facts.generation) != leaf.generation.checked_add(1) {
                return Err(refuse("the next generation must follow the current one"));
            }
            next_generation = Some(facts.generation);
            data_roots.push_str(&pem("CERTIFICATE", next));
            lane_roots.push_str(&pem("CERTIFICATE", next));
        }
        let b64 = |text: &str| base64::engine::general_purpose::STANDARD.encode(text);
        let key_pem = pem_secret("PRIVATE KEY", &pkcs8_ed25519(key.seed()));
        let key_pem_b64 = Zeroizing::new(b64(&key_pem));
        Ok(TeamAuth {
            fingerprint: key.short_fingerprint(),
            leaf_pem_b64: b64(&pem("CERTIFICATE", leaf_der)),
            key_pem_b64,
            data_roots_pem_b64: b64(&data_roots),
            lane_roots_pem_b64: (!lane_roots.is_empty()).then(|| b64(&lane_roots)),
            retired_generations,
            next_generation,
            leaf,
        })
    }

    /// The leaf's common name, `<role>.g<N>.<team>.cerulion.invalid`.
    pub fn common_name(&self) -> String {
        team::common_name(self.leaf.role, self.leaf.generation, &self.leaf.team)
    }

    /// The role the leaf grants.
    pub fn role(&self) -> TeamRole {
        self.leaf.role
    }

    /// The team generation the leaf belongs to.
    pub fn generation(&self) -> u32 {
        self.leaf.generation
    }

    /// The team short name.
    pub fn team(&self) -> &TeamShort {
        &self.leaf.team
    }

    /// The retired and next generations, whose leaves the bundle session
    /// admits to the bundle exchange.
    pub fn lane_generations(&self) -> Vec<u32> {
        let mut lane = self.retired_generations.clone();
        lane.extend(self.next_generation);
        lane
    }

    /// The next generation, when its CA is an anchor.
    pub fn next_generation(&self) -> Option<u32> {
        self.next_generation
    }

    /// The short fingerprint of the machine key (display only).
    pub fn fingerprint(&self) -> &str {
        &self.fingerprint
    }

    /// The base64 text of the PEM private key zenoh is given.
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn private_key_base64_for_test(&self) -> &str {
        &self.key_pem_b64
    }

    /// The data session's generated zenoh `access_control` object (see the
    /// module docs).
    pub fn access_control(&self) -> Value {
        access_control(
            &self.leaf.team,
            self.leaf.generation,
            self.next_generation.as_slice(),
        )
    }

    /// The bundle session's generated zenoh `access_control` object.
    pub fn bundle_lane_access_control(&self) -> Value {
        bundle_lane_access_control(&self.leaf.team, &self.lane_generations())
    }

    /// Set the team keys on `config` and refuse a listen or connect locator
    /// [`check_team_locator`] refuses (see the module docs for every key).
    ///
    /// # Errors
    ///
    /// [`TransportError::SessionCreation`] naming the refused locator, or the
    /// zenoh key that could not be set.
    pub(crate) fn apply(
        &self,
        config: &mut zenoh::Config,
        listen: &[String],
        connect: &[String],
        session: TeamSession,
    ) -> TransportResult<()> {
        let name = match session {
            TeamSession::Data => "team session",
            TeamSession::BundleLane => "team bundle session",
        };
        for (list, locators) in [("listen", listen), ("connect", connect)] {
            for locator in locators {
                check_team_locator(locator).map_err(|reason| TransportError::SessionCreation {
                    reason: format!("{name} {list} locator: {reason}"),
                })?;
            }
        }
        let (roots, policy) = match session {
            TeamSession::Data => (&self.data_roots_pem_b64, self.access_control()),
            TeamSession::BundleLane => (
                self.lane_roots_pem_b64.as_ref().ok_or_else(|| {
                    TransportError::SessionCreation {
                        reason: "a team bundle session needs a retired or next generation CA, \
                                 and these credentials hold neither"
                            .to_string(),
                    }
                })?,
                self.bundle_lane_access_control(),
            ),
        };
        let tls = "transport/link/tls";
        let no_autoconnect = json!({ "router": [], "peer": [] }).to_string();
        let plain: [(String, String); 8] = [
            (format!("{tls}/enable_mtls"), "true".into()),
            (format!("{tls}/verify_name_on_connect"), "false".into()),
            (format!("{tls}/close_link_on_expiration"), "true".into()),
            (
                "transport/link/protocols".into(),
                json!(["quic", "tls"]).to_string(),
            ),
            ("scouting/gossip/autoconnect".into(), no_autoconnect.clone()),
            ("scouting/multicast/autoconnect".into(), no_autoconnect),
            (format!("{tls}/root_ca_certificate_base64"), quoted(roots)),
            ("access_control".into(), policy.to_string()),
        ];
        let leaf = quoted(&self.leaf_pem_b64);
        let key = Zeroizing::new(quoted(&self.key_pem_b64));
        let certs = [
            (format!("{tls}/listen_certificate_base64"), &leaf),
            (format!("{tls}/connect_certificate_base64"), &leaf),
            (format!("{tls}/listen_private_key_base64"), &*key),
            (format!("{tls}/connect_private_key_base64"), &*key),
        ];
        let set = |config: &mut zenoh::Config, key: &str, value: &str| {
            config
                .insert_json5(key, value)
                .map_err(|e| TransportError::SessionCreation {
                    reason: format!("failed to set {key}: {e}"),
                })
        };
        for (k, v) in &plain {
            set(config, k, v)?;
        }
        for (k, v) in certs {
            set(config, &k, v)?;
        }
        tracing::info!(
            common_name = %self.common_name(),
            fingerprint = %self.fingerprint,
            session = name,
            retired_generations = ?self.retired_generations,
            next_generation = ?self.next_generation,
            "team session: mutual TLS on quic/ and tls/ links"
        );
        Ok(())
    }
}

/// Which of a machine's two team sessions a config is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TeamSession {
    /// The data session.
    Data,
    /// The bundle session.
    BundleLane,
}

/// `text` as a JSON string literal. Base64 text needs no escaping.
fn quoted(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    out.push_str(text);
    out.push('"');
    out
}

/// Refuse a locator a team session cannot authenticate: every carrier but
/// `quic/` and `tls/`, and any locator carrying a `#` endpoint config. The
/// reason names the locator and the spelling to use.
///
/// # Errors
///
/// The reason, as text.
pub fn check_team_locator(locator: &str) -> Result<(), String> {
    let use_instead = "use quic/<host>:<port>, or tls/<host>:<port> where UDP does not get through";
    if locator.contains('#') {
        return Err(format!(
            "'{locator}' carries a '#' endpoint config, which zenoh merges over the team \
             certificate and verification settings for that link; remove everything from '#' on"
        ));
    }
    match classify_locator(locator)? {
        Carrier::Quic { .. } | Carrier::Tls => Ok(()),
        Carrier::Tcp => Err(format!(
            "'{locator}' is plain TCP, which carries no authentication or encryption; \
             {use_instead}"
        )),
        Carrier::UnsecureQuic { .. } => Err(format!(
            "'{locator}' is QUIC with encryption and peer authentication switched off; \
             {use_instead}"
        )),
        Carrier::Udp => Err(format!(
            "'{locator}' is raw UDP, which carries no authentication or encryption; \
             {use_instead}"
        )),
    }
}

/// The bundle session's zenoh `access_control` object for `team`: default
/// deny, and the bundle exchange (query, reply and queryable declaration on
/// `cerulion_auth/bundle`, both flows) for the five common names of each of
/// `lane`.
pub fn bundle_lane_access_control(team: &TeamShort, lane: &[u32]) -> Value {
    let messages = ["query", "reply", "declare_queryable"];
    let names: Vec<String> = lane
        .iter()
        .flat_map(|g| team::common_names(*g, team))
        .collect();
    json!({
        "enabled": true,
        "default_permission": "deny",
        "rules": [
            {"id": "bundle_in", "permission": "allow", "flows": ["ingress"],
             "messages": messages, "key_exprs": [TEAM_BUNDLE_KEY]},
            {"id": "bundle_out", "permission": "allow", "flows": ["egress"],
             "messages": messages, "key_exprs": [TEAM_BUNDLE_KEY]},
        ],
        "subjects": [{ "id": "bundle_lane", "cert_common_names": names }],
        "policies": [{
            "id": "policy_bundle_lane",
            "rules": ["bundle_in", "bundle_out"],
            "subjects": ["bundle_lane"],
        }],
    })
}

/// The data session's zenoh `access_control` object for `team`: default
/// deny, the role rules for the five common names of `generation`, and the
/// bundle exchange for the five common names of each of `lane` (the next
/// generation). See the module docs.
pub fn access_control(team: &TeamShort, generation: u32, lane: &[u32]) -> Value {
    let data = format!("{DATA_KEY_NAMESPACE}/**");
    let lv = format!("{}/**", liveliness_prefix());
    let ann = format!("{}/**", announce_prefix());
    let q = format!("{CERULION_Q_PREFIX}/**");
    let auth_all = format!("{TEAM_AUTH_PREFIX}/**");
    let every_message = [
        "put",
        "delete",
        "declare_subscriber",
        "declare_queryable",
        "query",
        "reply",
        "liveliness_token",
        "declare_liveliness_subscriber",
        "liveliness_query",
    ];
    let liveliness_reads = ["declare_liveliness_subscriber", "liveliness_query"];
    let rule = |id: &str, flow: &str, messages: &[&str], keys: &[&str]| {
        json!({
            "id": id,
            "permission": "allow",
            "flows": [flow],
            "messages": messages,
            "key_exprs": keys,
        })
    };
    let rules = vec![
        // A reader.
        rule("read_in_sub", "ingress", &["declare_subscriber"], &[&data]),
        rule("read_in_demand", "ingress", &["liveliness_token"], &[&lv]),
        rule("read_in_query", "ingress", &["query"], &[&q]),
        rule(
            "read_in_liveliness",
            "ingress",
            &liveliness_reads,
            &[&lv, &ann],
        ),
        rule("read_out_data", "egress", &["put", "delete"], &[&data]),
        rule(
            "read_out_answer",
            "egress",
            &["reply", "declare_queryable"],
            &[&q],
        ),
        rule(
            "read_out_tokens",
            "egress",
            &["liveliness_token"],
            &[&ann, &lv],
        ),
        rule("read_out_liveliness", "egress", &liveliness_reads, &[&lv]),
        // A writer, on top of a reader.
        rule("write_in_data", "ingress", &["put", "delete"], &[&data]),
        rule(
            "write_in_announce",
            "ingress",
            &["liveliness_token"],
            &[&ann],
        ),
        rule(
            "write_in_answer",
            "ingress",
            &["reply", "declare_queryable"],
            &[&q],
        ),
        rule("write_out_sub", "egress", &["declare_subscriber"], &[&data]),
        rule("write_out_query", "egress", &["query"], &[&q]),
        rule("write_out_liveliness", "egress", &liveliness_reads, &[&ann]),
        // Owner and robot: every control message, and the auth namespace.
        rule(
            "full_in",
            "ingress",
            &every_message,
            &[&lv, &ann, &q, &auth_all],
        ),
        rule(
            "full_out",
            "egress",
            &every_message,
            &[&lv, &ann, &q, &auth_all],
        ),
        // The bundle exchange.
        rule(
            "bundle_in",
            "ingress",
            &["query", "reply", "declare_queryable"],
            &[TEAM_BUNDLE_KEY],
        ),
        rule(
            "bundle_out",
            "egress",
            &["query", "reply", "declare_queryable"],
            &[TEAM_BUNDLE_KEY],
        ),
    ];
    let read = [
        "read_in_sub",
        "read_in_demand",
        "read_in_query",
        "read_in_liveliness",
        "read_out_data",
        "read_out_answer",
        "read_out_tokens",
        "read_out_liveliness",
    ];
    let write = [
        "write_in_data",
        "write_in_announce",
        "write_in_answer",
        "write_out_sub",
        "write_out_query",
        "write_out_liveliness",
    ];
    let bundle = ["bundle_in", "bundle_out"];
    let mut subjects = Vec::new();
    let mut policies = Vec::new();
    for role in TeamRole::ALL {
        let mut granted: Vec<&str> = read.to_vec();
        if matches!(role, TeamRole::Operator | TeamRole::Owner | TeamRole::Robot) {
            granted.extend(write);
        }
        if matches!(role, TeamRole::Owner | TeamRole::Robot) {
            granted.extend(["full_in", "full_out"]);
        }
        granted.extend(bundle);
        let id = format!("role_{role}");
        subjects.push(json!({
            "id": id,
            "cert_common_names": [team::common_name(role, generation, team)],
        }));
        policies
            .push(json!({ "id": format!("policy_{role}"), "rules": granted, "subjects": [id] }));
    }
    if !lane.is_empty() {
        let names: Vec<String> = lane
            .iter()
            .flat_map(|g| team::common_names(*g, team))
            .collect();
        subjects.push(json!({ "id": "bundle_lane", "cert_common_names": names }));
        policies.push(json!({
            "id": "policy_bundle_lane",
            "rules": bundle,
            "subjects": ["bundle_lane"],
        }));
    }
    json!({
        "enabled": true,
        "default_permission": "deny",
        "rules": rules,
        "subjects": subjects,
        "policies": policies,
    })
}

/// The RFC 8410 PKCS#8 v1 `OneAsymmetricKey` for an ed25519 seed.
fn pkcs8_ed25519(seed: &[u8; 32]) -> Zeroizing<Vec<u8>> {
    const PREFIX: [u8; 16] = [
        0x30, 0x2e, 0x02, 0x01, 0x00, 0x30, 0x05, 0x06, 0x03, 0x2b, 0x65, 0x70, 0x04, 0x22, 0x04,
        0x20,
    ];
    let mut der = Zeroizing::new(Vec::with_capacity(PREFIX.len() + seed.len()));
    der.extend_from_slice(&PREFIX);
    der.extend_from_slice(seed);
    der
}

/// Append RFC 7468 PEM text for base64 `body` under `label` to `out`.
fn pem_into(label: &str, body: &str, out: &mut String) {
    out.push_str("-----BEGIN ");
    out.push_str(label);
    out.push_str("-----\n");
    let mut rest = body;
    while !rest.is_empty() {
        // Base64 text is ASCII, so any byte index is a char boundary.
        let (line, tail) = rest.split_at(rest.len().min(64));
        out.push_str(line);
        out.push('\n');
        rest = tail;
    }
    out.push_str("-----END ");
    out.push_str(label);
    out.push_str("-----\n");
}

/// The length of the PEM text [`pem_into`] writes, so the buffer is
/// allocated once and never moved while it grows.
fn pem_capacity(label: &str, body_len: usize) -> usize {
    2 * (label.len() + 17) + body_len + body_len.div_ceil(64)
}

/// RFC 7468 PEM text for `der` under `label`.
fn pem(label: &str, der: &[u8]) -> String {
    let body = base64::engine::general_purpose::STANDARD.encode(der);
    let mut out = String::with_capacity(pem_capacity(label, body.len()));
    pem_into(label, &body, &mut out);
    out
}

/// [`pem`] for secret bytes: every buffer holding the text is cleared on drop.
fn pem_secret(label: &str, der: &[u8]) -> Zeroizing<String> {
    let body = Zeroizing::new(base64::engine::general_purpose::STANDARD.encode(der));
    let mut out = Zeroizing::new(String::with_capacity(pem_capacity(label, body.len())));
    pem_into(label, &body, &mut out);
    out
}

/// The display fingerprint of a public key, as `MachineKey::short_fingerprint`
/// prints it: the first 3 bytes of its SHA-256 in lowercase hex.
fn short_fingerprint(public_key: &[u8; 32]) -> String {
    let fp = team::DeviceFingerprint::of(&cerulion_pairing::format::PublicKey(*public_key));
    fp.0[..3].iter().map(|b| format!("{b:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::test_ca::TestTeam;

    #[test]
    fn pkcs8_matches_the_rfc_8410_layout() {
        let der = pkcs8_ed25519(&[7u8; 32]);
        assert_eq!(der.len(), 48);
        assert_eq!(der[..2], [0x30, 0x2e]);
        assert_eq!(der[16..], [7u8; 32]);
    }

    #[test]
    fn pem_wraps_at_64_columns() {
        let text = pem("CERTIFICATE", &[0u8; 100]);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], "-----BEGIN CERTIFICATE-----");
        assert_eq!(lines[1].len(), 64);
        assert_eq!(lines.last(), Some(&"-----END CERTIFICATE-----"));
    }

    #[test]
    fn team_locators_admit_quic_and_tls_only() {
        for ok in [
            "quic/10.0.0.1:7684",
            "quic/h:1?mixed_rel=1",
            "tls/10.0.0.1:7684",
        ] {
            assert_eq!(check_team_locator(ok), Ok(()), "{ok}");
        }
        for (bad, needle) in [
            ("tcp/10.0.0.1:7683", "plain TCP"),
            (
                "udp/10.0.0.1:7684?rel=1;mixed_rel=1",
                "authentication switched off",
            ),
            ("udp/10.0.0.1:7684", "raw UDP"),
            ("tls/10.0.0.1:7684#enable_mtls=false", "'#' endpoint config"),
            (
                "quic/10.0.0.1:7684#root_ca_certificate_file=/x",
                "'#' endpoint config",
            ),
            ("ws/10.0.0.1:1", "'ws' carrier"),
        ] {
            let err = check_team_locator(bad).expect_err(bad);
            assert!(err.contains(needle), "{bad}: {err}");
            if !bad.starts_with("ws/") {
                assert!(err.contains(bad), "the refusal names the locator: {err}");
            }
            if !bad.starts_with("ws/") && !bad.contains('#') {
                assert!(err.contains("quic/<host>:<port>"), "{err}");
            }
        }
    }

    #[test]
    fn a_leaf_for_another_key_is_refused_with_both_fingerprints() {
        let team = TestTeam::new("acme", 2);
        let leaf = team.leaf(TeamRole::Robot, &MachineKey::from_seed([1; 32]));
        let other = MachineKey::from_seed([2; 32]);
        let err = TeamAuth::new(&other, &leaf, team.anchors(None)).unwrap_err();
        assert!(matches!(err, TeamAuthError::KeyMismatch { .. }), "{err}");
        assert!(
            err.to_string().contains(&other.short_fingerprint()),
            "{err}"
        );
    }

    #[test]
    fn a_leaf_from_another_team_ca_is_refused() {
        let acme = TestTeam::new("acme", 2);
        let rival = TestTeam::new("rival", 2);
        let key = MachineKey::from_seed([1; 32]);
        let leaf = rival.leaf(TeamRole::Robot, &key);
        let err = TeamAuth::new(&key, &leaf, acme.anchors(None)).unwrap_err();
        assert!(
            matches!(err, TeamAuthError::Certificate { ref what, .. } if what == "leaf"),
            "{err}"
        );
    }

    #[test]
    fn anchors_must_name_the_team_and_their_place_once() {
        let g3 = TestTeam::new("acme", 3);
        let key = MachineKey::from_seed([1; 32]);
        let leaf = g3.leaf(TeamRole::Viewer, &key);
        let g2 = TestTeam::new("acme", 2).retired();
        let g4 = TestTeam::new("acme", 4).retired();
        let other = TestTeam::new("rival", 1).retired();
        let with = |retired: &[RetiredGeneration], next: Option<&[u8]>| {
            TeamAuth::new(
                &key,
                &leaf,
                TeamAnchors {
                    issuing_ca_der: g3.ca_der(),
                    retired,
                    next_ca_der: next,
                },
            )
        };
        let g4_der = g3.next_ca_der();
        let ok = with(std::slice::from_ref(&g2), Some(&g4_der)).expect("valid anchors");
        assert_eq!(ok.lane_generations(), [2, 4]);
        for (next, needle) in [
            (
                TestTeam::new("acme", 5).ca_der().to_vec(),
                "follow the current",
            ),
            (TestTeam::new("rival", 4).ca_der().to_vec(), "another team"),
        ] {
            let err = with(&[], Some(&next)).unwrap_err();
            assert!(err.to_string().contains(needle), "{err}");
        }
        for (retired, needle) in [
            (vec![g4], "below the current"),
            (vec![other], "another team"),
            (vec![g2.clone(), g2.clone()], "listed twice"),
            (
                vec![RetiredGeneration {
                    generation: 1,
                    ca_der: g2.ca_der.clone(),
                }],
                "another generation",
            ),
        ] {
            let err = with(&retired, None).unwrap_err();
            assert!(err.to_string().contains(needle), "{err}");
        }
    }

    #[test]
    fn retired_names_reach_only_the_bundle_session_policy() {
        let g3 = TestTeam::new("acme", 1).rotated().rotated();
        let auth = g3.machine(TeamRole::Operator, 1);
        assert_eq!(auth.lane_generations(), [2, 1, 4]);
        let data = auth.access_control().to_string();
        assert!(!data.contains("robot.g2.acme") && !data.contains("robot.g1.acme"));
        assert!(
            data.contains("robot.g4.acme"),
            "the next generation's names"
        );
        let lane = auth.bundle_lane_access_control();
        let rules: Vec<&str> = lane["rules"]
            .as_array()
            .unwrap()
            .iter()
            .map(|r| r["id"].as_str().unwrap())
            .collect();
        assert_eq!(rules, ["bundle_in", "bundle_out"]);
        for rule in lane["rules"].as_array().unwrap() {
            assert_eq!(rule["key_exprs"], json!([TEAM_BUNDLE_KEY]));
        }
        let names = lane["subjects"][0]["cert_common_names"].to_string();
        assert!(names.contains("robot.g1.acme") && names.contains("robot.g4.acme"));
        assert!(
            !names.contains("robot.g3.acme"),
            "current names are not lane names"
        );
    }

    #[test]
    fn debug_prints_the_name_and_fingerprint_and_no_key_material() {
        let team = TestTeam::new("acme", 1);
        let auth = team.machine(TeamRole::Operator, 9);
        let text = format!("{auth:?}");
        assert!(text.contains("operator.g1.acme.cerulion.invalid"), "{text}");
        assert!(text.contains(auth.fingerprint()), "{text}");
        assert!(!text.contains(auth.private_key_base64_for_test()));
        assert!(!text.contains(&auth.leaf_pem_b64));
    }

    #[test]
    fn the_policy_lists_exact_names_per_generation_and_role() {
        let team = TeamShort::new("acme").unwrap();
        let acl = access_control(&team, 3, &[4]);
        assert_eq!(acl["default_permission"], "deny");
        let names = |id: &str| -> Vec<String> {
            let subject = acl["subjects"]
                .as_array()
                .unwrap()
                .iter()
                .find(|s| s["id"] == id)
                .unwrap_or_else(|| panic!("subject {id}"));
            serde_json::from_value(subject["cert_common_names"].clone()).unwrap()
        };
        assert_eq!(names("role_viewer"), ["viewer.g3.acme.cerulion.invalid"]);
        let lane = names("bundle_lane");
        assert_eq!(lane.len(), 5);
        assert!(lane.contains(&"owner.g4.acme.cerulion.invalid".to_string()));
        let lane_acl = bundle_lane_access_control(&team, &[2, 1, 4]);
        let lane_names: Vec<String> =
            serde_json::from_value(lane_acl["subjects"][0]["cert_common_names"].clone()).unwrap();
        assert_eq!(lane_names.len(), 15);
        assert!(lane_names.contains(&"robot.g1.acme.cerulion.invalid".to_string()));
        assert!(lane_names
            .iter()
            .all(|n| n.ends_with(".acme.cerulion.invalid")));
        // Every subject names team common names: none matches a peer whose
        // certificate carries another name, such as a Web PKI certificate.
        for subject in acl["subjects"].as_array().unwrap() {
            let cns = subject["cert_common_names"]
                .as_array()
                .expect("names listed");
            assert!(!cns.is_empty());
            assert!(cns
                .iter()
                .all(|c| c.as_str().unwrap().ends_with(".acme.cerulion.invalid")));
        }
        let rules_of = |id: &str| -> Vec<String> {
            let policy = acl["policies"]
                .as_array()
                .unwrap()
                .iter()
                .find(|p| p["subjects"][0] == id)
                .unwrap();
            serde_json::from_value(policy["rules"].clone()).unwrap()
        };
        let has = |id: &str, rule: &str| rules_of(id).iter().any(|r| r == rule);
        assert!(!has("role_viewer", "write_in_data"));
        assert!(!has("role_viewer", "write_in_answer"));
        assert!(!has("role_ci", "write_in_announce"));
        assert!(has("role_operator", "write_in_data"));
        assert!(!has("role_operator", "full_in"));
        assert!(has("role_owner", "full_in"));
        assert!(has("role_robot", "full_out"));
        assert_eq!(rules_of("bundle_lane"), ["bundle_in", "bundle_out"]);
    }
}
