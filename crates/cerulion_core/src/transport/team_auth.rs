// SPDX-License-Identifier: AGPL-3.0-only
//! Mutually authenticated team sessions: the zenoh configuration a machine
//! with team credentials opens its network session with.
//!
//! A team machine holds three things: its machine key (one ed25519 seed per
//! machine, `cerulion_pairing::identity`), a leaf certificate over that key
//! issued by the team issuing CA, and the issuing CA certificates of the
//! team's current and retired generations (`cerulion_pairing::team`). The leaf
//! common name `<role>.g<N>.<team>.cerulion.invalid` names the role.
//!
//! [`TeamAuth::new`] checks those with the `cerulion_pairing` profile checkers
//! and [`apply`](TeamAuth::apply) sets, on the session's zenoh config:
//!
//! | zenoh key | value |
//! |---|---|
//! | `transport/link/tls/enable_mtls` | `true`: both ends present and verify a certificate |
//! | `transport/link/tls/{listen,connect}_certificate_base64` | the leaf, PEM |
//! | `transport/link/tls/{listen,connect}_private_key_base64` | the machine key, PKCS#8 PEM built in memory |
//! | `transport/link/tls/root_ca_certificate_base64` | the current and retired issuing CAs, PEM |
//! | `transport/link/tls/verify_name_on_connect` | `false`: peers are dialed by bare IP; the access policy names the peer instead |
//! | `transport/link/tls/close_link_on_expiration` | `true`: a link closes when the peer leaf's notAfter passes |
//! | `transport/link/protocols` | `["quic", "tls"]`: zenoh refuses any other link, including a gossip-learned one |
//! | `scouting/gossip/autoconnect` | none: zenoh orders a peer's gossiped locators at random, so the session dials no gossip-learned peer; routing through linked peers is unchanged |
//! | `access_control` | [`access_control`], default deny, keyed on the peer leaf common name |
//!
//! The `*_base64` values are zenoh `SecretValue`s: zenoh does not serialize
//! them and redacts them in `Debug`. Nothing here writes key material to disk,
//! the environment or a log line; [`TeamAuth`]'s `Debug` prints the common
//! name and the machine key fingerprint.
//!
//! zenoh's `quic/` and `tls/` links read the same `transport/link/tls`
//! section, so both links carry the same identity. A team session refuses
//! every other listen or connect locator ([`check_team_locator`]).
//!
//! ## Access policy
//!
//! zenoh matches a subject's `cert_common_names` by exact string equality, so
//! the policy lists the five role names of each generation it admits. Rules by
//! role, for links to a peer whose leaf names that role in the current
//! generation:
//!
//! | Role | Data (`cerulion_w2/**`) | Control (`cerulion_lv`, `cerulion_ann`, `cerulion_q`) | `cerulion_auth/**` |
//! |---|---|---|---|
//! | viewer, ci | receive puts; declare subscribers | all declarations, queries, replies, liveliness | bundle query only |
//! | operator | viewer, plus put and delete | as viewer | bundle query only |
//! | owner | operator | as viewer | everything |
//! | robot | operator | as viewer | everything |
//!
//! A leaf of a retired generation (the "upgrade lane") may only query
//! `cerulion_auth/bundle` and receive the reply. Every linked peer may answer
//! this machine's own bundle query: a bundle is verified against the pinned
//! root set before it is used, whoever sends it.
//!
//! In zenoh peer mode a writer forwards a put only to subscribers it has seen
//! declared, and a reader routes a query only to queryables it has seen, so
//! each data and control rule exists in both flows: ingress for what the peer
//! may send, egress for what this machine may send to it.

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
    /// Mutual TLS with team credentials on `quic/` and `tls/` links.
    Team(TeamAuth),
}

/// Why team credentials cannot open a session.
#[derive(Debug, thiserror::Error)]
pub enum TeamAuthError {
    /// The leaf or a CA certificate fails the team profile or its signature.
    #[error("team {what} certificate is not usable: {source}")]
    Certificate {
        /// Which certificate: `leaf`, `issuing CA` or `retired CA g<N>`.
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
    /// A retired CA names another team, or a generation not below the current one,
    /// or repeats a generation.
    #[error("retired CA g{generation}: {reason}")]
    Retired {
        /// The retired generation number as written.
        generation: u32,
        /// What is wrong with it.
        reason: &'static str,
    },
}

/// A machine's team credentials, checked and held as the PEM text zenoh reads.
#[derive(Clone)]
pub struct TeamAuth {
    leaf: LeafFacts,
    fingerprint: String,
    retired_generations: Vec<u32>,
    leaf_pem_b64: String,
    key_pem_b64: Zeroizing<String>,
    roots_pem_b64: String,
}

impl std::fmt::Debug for TeamAuth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TeamAuth")
            .field("common_name", &self.common_name())
            .field("fingerprint", &self.fingerprint)
            .field("retired_generations", &self.retired_generations)
            .finish_non_exhaustive()
    }
}

impl TeamAuth {
    /// Check and hold team credentials.
    ///
    /// `leaf_der` must satisfy the team leaf profile, be signed by
    /// `issuing_ca_der` (same team and generation) and certify `key`'s public
    /// key. Each retired CA must satisfy the CA profile, name the same team and
    /// a generation below the current one, at most once.
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
        issuing_ca_der: &[u8],
        retired: &[RetiredGeneration],
    ) -> Result<TeamAuth, TeamAuthError> {
        let cert_err = |what: &str| {
            let what = what.to_string();
            move |source| TeamAuthError::Certificate { what, source }
        };
        let ca = check_team_ca(issuing_ca_der).map_err(cert_err("issuing CA"))?;
        let leaf = verify_leaf_issued_by(leaf_der, &ca).map_err(cert_err("leaf"))?;
        if leaf.device_key.0 != key.public_key() {
            return Err(TeamAuthError::KeyMismatch {
                leaf_fingerprint: short_fingerprint(&leaf.device_key.0),
                machine_fingerprint: key.short_fingerprint(),
            });
        }
        let mut retired_generations = Vec::with_capacity(retired.len());
        let mut roots_pem = pem("CERTIFICATE", issuing_ca_der);
        for r in retired {
            let facts = check_team_ca(&r.ca_der)
                .map_err(cert_err(&format!("retired CA g{}", r.generation)))?;
            let refuse = |reason| TeamAuthError::Retired {
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
            roots_pem.push_str(&pem("CERTIFICATE", &r.ca_der));
        }
        let b64 = |text: &str| base64::engine::general_purpose::STANDARD.encode(text);
        let key_pem = Zeroizing::new(pem("PRIVATE KEY", &pkcs8_ed25519(key.seed())[..]));
        Ok(TeamAuth {
            fingerprint: key.short_fingerprint(),
            leaf_pem_b64: b64(&pem("CERTIFICATE", leaf_der)),
            key_pem_b64: Zeroizing::new(b64(&key_pem)),
            roots_pem_b64: b64(&roots_pem),
            retired_generations,
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

    /// The retired generations whose CAs this session also trusts.
    pub fn retired_generations(&self) -> &[u32] {
        &self.retired_generations
    }

    /// The short fingerprint of the machine key (display only).
    pub fn fingerprint(&self) -> &str {
        &self.fingerprint
    }

    /// The generated zenoh `access_control` object (see the module docs).
    pub fn access_control(&self) -> Value {
        access_control(
            &self.leaf.team,
            self.leaf.generation,
            &self.retired_generations,
        )
    }

    /// Set the team keys on `config` and refuse a listen or connect locator
    /// that is not `quic/` or `tls/` (see the module docs for every key).
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
    ) -> TransportResult<()> {
        for (list, locators) in [("listen", listen), ("connect", connect)] {
            for locator in locators {
                check_team_locator(locator).map_err(|reason| TransportError::SessionCreation {
                    reason: format!("team session {list} locator: {reason}"),
                })?;
            }
        }
        let tls = "transport/link/tls";
        // Base64 text needs no JSON escaping, so it is quoted directly and no
        // unzeroized copy of the key is left behind in a `serde_json::Value`.
        let secret = |v: &str| format!("\"{v}\"");
        let settings: [(String, String); 11] = [
            (format!("{tls}/enable_mtls"), "true".into()),
            (format!("{tls}/verify_name_on_connect"), "false".into()),
            (format!("{tls}/close_link_on_expiration"), "true".into()),
            (
                format!("{tls}/root_ca_certificate_base64"),
                secret(&self.roots_pem_b64),
            ),
            (
                format!("{tls}/listen_certificate_base64"),
                secret(&self.leaf_pem_b64),
            ),
            (
                format!("{tls}/connect_certificate_base64"),
                secret(&self.leaf_pem_b64),
            ),
            (
                format!("{tls}/listen_private_key_base64"),
                secret(&self.key_pem_b64),
            ),
            (
                format!("{tls}/connect_private_key_base64"),
                secret(&self.key_pem_b64),
            ),
            (
                "transport/link/protocols".into(),
                json!(["quic", "tls"]).to_string(),
            ),
            (
                "scouting/gossip/autoconnect".into(),
                json!({ "router": [], "peer": [] }).to_string(),
            ),
            ("access_control".into(), self.access_control().to_string()),
        ];
        for (key, value) in settings {
            let value = Zeroizing::new(value);
            config
                .insert_json5(&key, &value)
                .map_err(|e| TransportError::SessionCreation {
                    reason: format!("failed to set {key}: {e}"),
                })?;
        }
        tracing::info!(
            common_name = %self.common_name(),
            fingerprint = %self.fingerprint,
            retired_generations = ?self.retired_generations,
            "team session: mutual TLS on quic/ and tls/ links"
        );
        Ok(())
    }
}

/// Refuse a locator a team session cannot authenticate: everything but
/// `quic/` and `tls/`. The reason names the locator and the spelling to use.
///
/// # Errors
///
/// The reason, as text, for any carrier other than `quic/` or `tls/`, and for
/// a locator [`classify_locator`] refuses.
pub fn check_team_locator(locator: &str) -> Result<(), String> {
    let use_instead = "use quic/<host>:<port>, or tls/<host>:<port> where UDP does not get through";
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

/// The generated zenoh `access_control` object for `team`: default deny, the
/// role rules for the five common names of `generation`, the bundle query for
/// the five common names of each retired generation, and the reply to this
/// machine's own bundle query from any linked peer (see the module docs).
pub fn access_control(team: &TeamShort, generation: u32, retired: &[u32]) -> Value {
    let data = format!("{DATA_KEY_NAMESPACE}/**");
    let control = [
        format!("{}/**", liveliness_prefix()),
        format!("{}/**", announce_prefix()),
        format!("{CERULION_Q_PREFIX}/**"),
    ];
    let auth_all = format!("{TEAM_AUTH_PREFIX}/**");
    let control_messages = [
        "declare_subscriber",
        "declare_queryable",
        "query",
        "reply",
        "liveliness_token",
        "declare_liveliness_subscriber",
        "liveliness_query",
    ];
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
    let rule = |id: &str, flows: &[&str], messages: &[&str], keys: &[&str]| {
        json!({
            "id": id,
            "permission": "allow",
            "flows": flows,
            "messages": messages,
            "key_exprs": keys,
        })
    };
    let control_keys: Vec<&str> = control.iter().map(String::as_str).collect();
    let rules = vec![
        rule("control_in", &["ingress"], &control_messages, &control_keys),
        rule("control_out", &["egress"], &control_messages, &control_keys),
        rule(
            "data_read_in",
            &["ingress"],
            &["declare_subscriber"],
            &[&data],
        ),
        rule("data_read_out", &["egress"], &["put", "delete"], &[&data]),
        rule("data_write_in", &["ingress"], &["put", "delete"], &[&data]),
        rule(
            "data_write_out",
            &["egress"],
            &["declare_subscriber"],
            &[&data],
        ),
        rule(
            "auth_admin",
            &["ingress", "egress"],
            &every_message,
            &[&auth_all],
        ),
        rule("bundle_in", &["ingress"], &["query"], &[TEAM_BUNDLE_KEY]),
        rule(
            "bundle_out",
            &["egress"],
            &["reply", "declare_queryable"],
            &[TEAM_BUNDLE_KEY],
        ),
        rule("bundle_ask", &["egress"], &["query"], &[TEAM_BUNDLE_KEY]),
        rule(
            "bundle_answer",
            &["ingress"],
            &["reply", "declare_queryable"],
            &[TEAM_BUNDLE_KEY],
        ),
    ];
    let read = ["control_in", "control_out", "data_read_in", "data_read_out"];
    let write = ["data_write_in", "data_write_out"];
    let bundle = ["bundle_in", "bundle_out"];
    let mut subjects = Vec::new();
    let mut policies = Vec::new();
    for role in TeamRole::ALL {
        let mut granted: Vec<&str> = read.to_vec();
        if matches!(role, TeamRole::Operator | TeamRole::Owner | TeamRole::Robot) {
            granted.extend(write);
        }
        if matches!(role, TeamRole::Owner | TeamRole::Robot) {
            granted.push("auth_admin");
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
    if !retired.is_empty() {
        let lane: Vec<String> = retired
            .iter()
            .flat_map(|g| team::common_names(*g, team))
            .collect();
        subjects.push(json!({ "id": "upgrade_lane", "cert_common_names": lane }));
        policies.push(json!({
            "id": "policy_upgrade_lane",
            "rules": bundle,
            "subjects": ["upgrade_lane"],
        }));
    }
    // A subject with no property matches every transport.
    subjects.push(json!({ "id": "any_peer" }));
    policies.push(json!({
        "id": "policy_any_peer",
        "rules": ["bundle_ask", "bundle_answer"],
        "subjects": ["any_peer"],
    }));
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

/// RFC 7468 PEM text for `der` under `label`.
fn pem(label: &str, der: &[u8]) -> String {
    let body = base64::engine::general_purpose::STANDARD.encode(der);
    let mut out = format!("-----BEGIN {label}-----\n");
    let mut rest = body.as_str();
    while !rest.is_empty() {
        // Base64 text is ASCII, so any byte index is a char boundary.
        let (line, tail) = rest.split_at(rest.len().min(64));
        out.push_str(line);
        out.push('\n');
        rest = tail;
    }
    out.push_str(&format!("-----END {label}-----\n"));
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
            ("ws/10.0.0.1:1", "'ws' carrier"),
        ] {
            let err = check_team_locator(bad).expect_err(bad);
            assert!(err.contains(needle), "{bad}: {err}");
            if !bad.starts_with("ws/") {
                assert!(err.contains(bad), "the refusal names the locator: {err}");
                assert!(err.contains("quic/<host>:<port>"), "{err}");
            }
        }
    }

    #[test]
    fn a_leaf_for_another_key_is_refused_with_both_fingerprints() {
        let team = TestTeam::new("acme", 2);
        let leaf = team.leaf(TeamRole::Robot, &MachineKey::from_seed([1; 32]));
        let other = MachineKey::from_seed([2; 32]);
        let err = TeamAuth::new(&other, &leaf, team.ca_der(), &[]).unwrap_err();
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
        let err = TeamAuth::new(&key, &leaf, acme.ca_der(), &[]).unwrap_err();
        assert!(
            matches!(err, TeamAuthError::Certificate { ref what, .. } if what == "leaf"),
            "{err}"
        );
    }

    #[test]
    fn retired_generations_must_be_below_current_same_team_and_unique() {
        let g3 = TestTeam::new("acme", 3);
        let key = MachineKey::from_seed([1; 32]);
        let leaf = g3.leaf(TeamRole::Viewer, &key);
        let g2 = TestTeam::new("acme", 2).retired();
        let g4 = TestTeam::new("acme", 4).retired();
        let other = TestTeam::new("rival", 1).retired();
        assert!(TeamAuth::new(&key, &leaf, g3.ca_der(), std::slice::from_ref(&g2)).is_ok());
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
            let err = TeamAuth::new(&key, &leaf, g3.ca_der(), &retired).unwrap_err();
            assert!(err.to_string().contains(needle), "{err}");
        }
    }

    #[test]
    fn debug_prints_the_name_and_fingerprint_and_no_key_material() {
        let team = TestTeam::new("acme", 1);
        let auth = team.machine(TeamRole::Operator, 9);
        let text = format!("{auth:?}");
        assert!(text.contains("operator.g1.acme.cerulion.invalid"), "{text}");
        assert!(text.contains(auth.fingerprint()), "{text}");
        assert!(!text.contains(auth.key_pem_b64.as_str()));
        assert!(!text.contains(&auth.leaf_pem_b64));
    }

    #[test]
    fn the_policy_lists_exact_names_per_generation_and_role() {
        let team = TeamShort::new("acme").unwrap();
        let acl = access_control(&team, 3, &[2, 1]);
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
        let lane = names("upgrade_lane");
        assert_eq!(lane.len(), 10);
        assert!(lane.contains(&"robot.g1.acme.cerulion.invalid".to_string()));
        let rules_of = |id: &str| -> Vec<String> {
            let policy = acl["policies"]
                .as_array()
                .unwrap()
                .iter()
                .find(|p| p["subjects"][0] == id)
                .unwrap();
            serde_json::from_value(policy["rules"].clone()).unwrap()
        };
        assert!(!rules_of("role_viewer").contains(&"data_write_in".to_string()));
        assert!(rules_of("role_operator").contains(&"data_write_in".to_string()));
        assert!(!rules_of("role_operator").contains(&"auth_admin".to_string()));
        assert!(rules_of("role_owner").contains(&"auth_admin".to_string()));
        assert_eq!(rules_of("upgrade_lane"), ["bundle_in", "bundle_out"]);
        assert_eq!(rules_of("any_peer"), ["bundle_ask", "bundle_answer"]);
    }
}
