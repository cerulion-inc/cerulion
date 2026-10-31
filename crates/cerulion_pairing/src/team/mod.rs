// SPDX-License-Identifier: MIT OR Apache-2.0
//! Team trust formats: the X.509 profile machines present on mutually
//! authenticated network links, and the intermediate-signed statements that
//! carry a team's trust anchors and device status between peers.
//!
//! ## Names
//!
//! A machine leaf names its role, the team generation and the team in its
//! subject common name: `<role>.g<N>.<team>.cerulion.invalid` (see
//! [`common_name`]). `.invalid` is a reserved top level name, so no public
//! certificate authority issues it. A team issuing CA is named
//! `ca.g<N>.<team>.cerulion.invalid`; `ca` is not a role, so a CA name never
//! parses as a leaf name.
//!
//! ## Chain
//!
//! The team issuing CA certificate of each generation is self-signed by that
//! generation's CA key. A node trusts it only when it arrives inside a
//! [`SignedTeamTrustBundle`] whose signature chains through the online
//! intermediate to the pinned [`crate::format::RootSet`]; the CA certificate's
//! own signature only proves possession of the CA key.
//!
//! ## Statements
//!
//! [`TeamTrustBundle`] (domain `cerulion-pairing:team-trust:v1`) carries the
//! current and retired issuing CA certificates and every robot leaf.
//! [`TeamStatusList`] and [`StatusFreshness`] share the domain
//! `cerulion-pairing:team-status:v1` and are told apart by a statement kind
//! byte written right after the domain, so a signature over one never verifies
//! as the other. The [`rollback`] functions decide whether a verified
//! statement is newer than what a node already holds.

mod der;
#[cfg(feature = "dev-issuer")]
pub mod dev_issuer;
pub mod rollback;
mod statement;
mod x509;

pub use statement::{
    verify_status_freshness, verify_status_list, verify_team_trust_bundle, DeviceStatus,
    RetiredGeneration, SignedStatusFreshness, SignedTeamStatusList, SignedTeamTrustBundle,
    StatusEntry, StatusFreshness, TeamStatusList, TeamTrustBundle, Verified, STATUS_FRESHNESS_KIND,
    STATUS_LIST_KIND, TEAM_STATUS_DOMAIN, TEAM_TRUST_DOMAIN,
};
pub use x509::{
    check_leaf, check_team_ca, issue_leaf, issue_team_ca, verify_leaf_issued_by, LeafFacts,
    LeafParams, TeamCaFacts, BACKDATE_SECS, CI_LIFETIME_SECS, DESK_LIFETIME_SECS,
    NO_EXPIRY_UNIX_SECS,
};

use serde::{Deserialize, Serialize};

use crate::format::{PublicKey, RootSet};
use crate::PairingError;

/// True when this build has the `dev-issuer` feature: it then trusts team
/// credentials from a development issuer (see [`trusted_root_sets`]).
pub const DEV_ISSUER_COMPILED: bool = cfg!(feature = "dev-issuer");

/// The root sets compiled into this build that team trust bundles chain to.
/// Empty, so a build without `dev-issuer` accepts no team credentials.
pub fn release_root_sets() -> Vec<RootSet> {
    Vec::new()
}

/// The root sets team credentials are checked against: the release root sets.
/// A `dev-issuer` build adds the development root set of `layout`.
///
/// # Errors
///
/// None in this build; the `dev-issuer` build can fail reading its state file.
#[cfg(not(feature = "dev-issuer"))]
pub fn trusted_root_sets(
    layout: &crate::identity::IdentityLayout,
) -> Result<Vec<RootSet>, PairingError> {
    let _ = layout;
    Ok(release_root_sets())
}

#[cfg(feature = "dev-issuer")]
pub use dev_issuer::trusted_root_sets;

/// The fixed suffix every team name ends with.
pub const NAME_SUFFIX: &str = "cerulion.invalid";

/// The longest team short name: one DNS label.
pub const TEAM_SHORT_MAX_LEN: usize = 63;

/// The role a machine leaf grants on the network. Its text form is the first
/// label of the leaf common name.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, Serialize, Deserialize)]
pub enum TeamRole {
    /// A robot. Its leaf never expires.
    Robot,
    /// A team owner's desk or laptop.
    Owner,
    /// An operator's desk or laptop.
    Operator,
    /// A viewer's desk or laptop.
    Viewer,
    /// A continuous integration runner.
    Ci,
}

impl TeamRole {
    /// Every role, in the order [`common_names`] lists them.
    pub const ALL: [TeamRole; 5] = [
        TeamRole::Robot,
        TeamRole::Owner,
        TeamRole::Operator,
        TeamRole::Viewer,
        TeamRole::Ci,
    ];

    /// The label written in the common name.
    pub fn as_str(self) -> &'static str {
        match self {
            TeamRole::Robot => "robot",
            TeamRole::Owner => "owner",
            TeamRole::Operator => "operator",
            TeamRole::Viewer => "viewer",
            TeamRole::Ci => "ci",
        }
    }

    /// The role whose label is `s`, if any.
    pub fn from_label(s: &str) -> Option<TeamRole> {
        TeamRole::ALL.into_iter().find(|r| r.as_str() == s)
    }

    /// The leaf lifetime for this role in seconds from issuance, or `None` for
    /// a robot (its notAfter is [`NO_EXPIRY_UNIX_SECS`]).
    pub fn lifetime_secs(self) -> Option<u64> {
        match self {
            TeamRole::Robot => None,
            TeamRole::Owner | TeamRole::Operator | TeamRole::Viewer => Some(DESK_LIFETIME_SECS),
            TeamRole::Ci => Some(CI_LIFETIME_SECS),
        }
    }
}

impl core::fmt::Display for TeamRole {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A team's short name: one DNS label of 1 to 63 characters from `[a-z0-9-]`,
/// not starting or ending with `-`. It is the team label in every common name.
#[derive(Clone, PartialEq, Eq, Hash, Debug, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct TeamShort(String);

impl TeamShort {
    /// Validate and wrap a short name.
    pub fn new(s: impl Into<String>) -> Result<TeamShort, PairingError> {
        let s = s.into();
        let ok_chars = s
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-');
        if s.is_empty()
            || s.len() > TEAM_SHORT_MAX_LEN
            || !ok_chars
            || s.starts_with('-')
            || s.ends_with('-')
        {
            return Err(PairingError::InvalidTeamShort);
        }
        Ok(TeamShort(s))
    }

    /// The short name text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl TryFrom<String> for TeamShort {
    type Error = PairingError;
    fn try_from(s: String) -> Result<Self, PairingError> {
        TeamShort::new(s)
    }
}

impl From<TeamShort> for String {
    fn from(t: TeamShort) -> String {
        t.0
    }
}

impl core::fmt::Display for TeamShort {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(&self.0)
    }
}

/// A stable team identifier, distinct from its short name.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct TeamId(pub [u8; 32]);

impl core::fmt::Debug for TeamId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "TeamId({})", crate::crypto::to_hex(&self.0))
    }
}

/// A device fingerprint: SHA-256 of the device's 32-byte ed25519 public key.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct DeviceFingerprint(pub [u8; 32]);

impl DeviceFingerprint {
    /// The fingerprint of `key`.
    pub fn of(key: &PublicKey) -> DeviceFingerprint {
        use sha2::{Digest, Sha256};
        DeviceFingerprint(Sha256::digest(key.0).into())
    }
}

impl core::fmt::Debug for DeviceFingerprint {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "DeviceFingerprint({})", crate::crypto::to_hex(&self.0))
    }
}

/// The leaf common name `<role>.g<generation>.<team>.cerulion.invalid`.
pub fn common_name(role: TeamRole, generation: u32, team: &TeamShort) -> String {
    format!("{role}.g{generation}.{team}.{NAME_SUFFIX}")
}

/// The common name of every role at one generation, in [`TeamRole::ALL`]
/// order. Name matching on the link is by exact string equality, so an access
/// policy lists these names one by one for each generation it admits.
pub fn common_names(generation: u32, team: &TeamShort) -> [String; 5] {
    TeamRole::ALL.map(|r| common_name(r, generation, team))
}

/// The issuing CA common name `ca.g<generation>.<team>.cerulion.invalid`.
pub fn ca_common_name(generation: u32, team: &TeamShort) -> String {
    format!("ca.g{generation}.{team}.{NAME_SUFFIX}")
}

/// Split `<first>.g<N>.<team>.cerulion.invalid` into its first label, the
/// generation and the team. The generation must be canonical decimal.
fn split_name(cn: &str) -> Result<(&str, u32, TeamShort), PairingError> {
    let bad = || PairingError::TeamNameMismatch;
    let head = cn
        .strip_suffix(NAME_SUFFIX)
        .and_then(|h| h.strip_suffix('.'))
        .ok_or_else(bad)?;
    let mut labels = head.splitn(3, '.');
    let first = labels.next().ok_or_else(bad)?;
    let gen_label = labels.next().ok_or_else(bad)?;
    let team = labels.next().ok_or_else(bad)?;
    let digits = gen_label.strip_prefix('g').ok_or_else(bad)?;
    let canonical = !digits.is_empty()
        && digits.bytes().all(|b| b.is_ascii_digit())
        && (digits == "0" || !digits.starts_with('0'));
    if !canonical {
        return Err(bad());
    }
    let generation: u32 = digits.parse().map_err(|_| bad())?;
    let team = TeamShort::new(team).map_err(|_| bad())?;
    Ok((first, generation, team))
}

/// Parse a leaf common name into its role, generation and team.
pub fn parse_common_name(cn: &str) -> Result<(TeamRole, u32, TeamShort), PairingError> {
    let (first, generation, team) = split_name(cn)?;
    let role = TeamRole::from_label(first).ok_or(PairingError::TeamNameMismatch)?;
    Ok((role, generation, team))
}

/// Parse an issuing CA common name into its generation and team.
pub fn parse_ca_common_name(cn: &str) -> Result<(u32, TeamShort), PairingError> {
    let (first, generation, team) = split_name(cn)?;
    if first != "ca" {
        return Err(PairingError::TeamNameMismatch);
    }
    Ok((generation, team))
}

/// The device SAN URI `urn:cerulion:device:<base32 key>`.
pub fn device_uri(key: &PublicKey) -> String {
    format!("urn:cerulion:device:{}", base32_lower(&key.0))
}

/// The account SAN URI `urn:cerulion:account:<base32 id>`.
pub fn account_uri(account: &crate::format::AccountId) -> String {
    format!("urn:cerulion:account:{}", base32_lower(&account.0))
}

/// RFC 4648 base32, lowercase, no padding.
pub fn base32_lower(bytes: &[u8]) -> String {
    const ALPHABET: &[u8; 32] = b"abcdefghijklmnopqrstuvwxyz234567";
    let mut out = String::with_capacity(bytes.len().div_ceil(5) * 8);
    let mut acc: u32 = 0;
    let mut bits = 0u32;
    for &b in bytes {
        acc = (acc << 8) | b as u32;
        bits += 8;
        while bits >= 5 {
            bits -= 5;
            out.push(ALPHABET[((acc >> bits) & 0x1F) as usize] as char);
        }
    }
    if bits > 0 {
        out.push(ALPHABET[((acc << (5 - bits)) & 0x1F) as usize] as char);
    }
    out
}

/// Decode [`base32_lower`] text; `None` on any character outside the alphabet
/// or nonzero trailing bits.
fn base32_lower_decode(s: &str) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(s.len() * 5 / 8);
    let mut acc: u32 = 0;
    let mut bits = 0u32;
    for c in s.bytes() {
        let v = match c {
            b'a'..=b'z' => c - b'a',
            b'2'..=b'7' => c - b'2' + 26,
            _ => return None,
        };
        acc = (acc << 5) | v as u32;
        bits += 5;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
        }
        acc &= (1 << bits) - 1;
    }
    if acc != 0 {
        return None;
    }
    Some(out)
}

/// The 32 bytes behind `<prefix><base32>`, if `uri` has that shape.
fn uri_bytes32(uri: &str, prefix: &str) -> Option<[u8; 32]> {
    let text = uri.strip_prefix(prefix)?;
    if text.len() != 52 {
        return None;
    }
    base32_lower_decode(text)?.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base32_matches_rfc4648_vectors() {
        // RFC 4648 section 10, lowercased, padding removed.
        let cases: [(&[u8], &str); 7] = [
            (b"", ""),
            (b"f", "my"),
            (b"fo", "mzxq"),
            (b"foo", "mzxw6"),
            (b"foob", "mzxw6yq"),
            (b"fooba", "mzxw6ytb"),
            (b"foobar", "mzxw6ytboi"),
        ];
        for (input, want) in cases {
            assert_eq!(base32_lower(input), want);
            assert_eq!(base32_lower_decode(want).unwrap(), input);
        }
        assert_eq!(base32_lower_decode("MY"), None);
        assert_eq!(base32_lower_decode("mz"), None, "nonzero trailing bits");
    }

    #[test]
    fn split_name_refuses_noncanonical_generations() {
        for bad in [
            "robot.g01.acme.cerulion.invalid",
            "robot.g.acme.cerulion.invalid",
            "robot.g+1.acme.cerulion.invalid",
            "robot.g4294967296.acme.cerulion.invalid",
            "robot.1.acme.cerulion.invalid",
            "robot.g1.acme.cerulion.invalid.",
            "robot.g1.acme.cerulion.example",
            "robot.g1.cerulion.invalid",
            "robot.g1.ac.me.cerulion.invalid",
        ] {
            assert!(parse_common_name(bad).is_err(), "{bad}");
        }
        assert_eq!(
            parse_common_name("robot.g0.acme.cerulion.invalid")
                .unwrap()
                .1,
            0
        );
    }
}
