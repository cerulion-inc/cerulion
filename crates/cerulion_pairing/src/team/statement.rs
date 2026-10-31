// SPDX-License-Identifier: MIT OR Apache-2.0
//! The intermediate-signed team statements: the trust bundle, the status list
//! and the per-request status freshness proof.
//!
//! Each signed container carries the root-signed intermediate it chains
//! through, so a statement relayed peer to peer verifies on its own against
//! the pinned [`RootSet`].

use ed25519_dalek::SigningKey;
use serde::{Deserialize, Serialize};

use super::x509::{check_team_ca, verify_leaf_issued_by};
use super::{DeviceFingerprint, TeamId, TeamRole, TeamShort};
use crate::crypto::{self, VerifyResult};
use crate::format::canonical::CanonicalWriter;
use crate::format::{PublicKey, RootSet, Signature, SignedIntermediateCert, SignedPayload};
use crate::verify::chain::{check_issued_at, verify_intermediate};
use crate::{CertKind, PairingError};

/// Domain tag of [`TeamTrustBundle`] signing bytes.
pub const TEAM_TRUST_DOMAIN: &[u8] = b"cerulion-pairing:team-trust:v1";
/// Domain tag shared by [`TeamStatusList`] and [`StatusFreshness`].
pub const TEAM_STATUS_DOMAIN: &[u8] = b"cerulion-pairing:team-status:v1";
/// Statement kind byte of a [`TeamStatusList`], written after the domain.
pub const STATUS_LIST_KIND: u8 = 1;
/// Statement kind byte of a [`StatusFreshness`], written after the domain.
pub const STATUS_FRESHNESS_KIND: u8 = 2;

/// A statement that passed verification. Only this module constructs one, so
/// a function that takes `&Verified<T>` cannot be handed an unchecked `T`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Verified<T>(T);

impl<T> Verified<T> {
    /// The verified statement.
    pub fn get(&self) -> &T {
        &self.0
    }

    /// Unwrap the verified statement.
    pub fn into_inner(self) -> T {
        self.0
    }
}

fn verify_sig(
    key: &PublicKey,
    payload: &[u8],
    sig: &Signature,
    kind: CertKind,
) -> Result<(), PairingError> {
    match crypto::ed25519_verify(key, payload, sig) {
        VerifyResult::Ok => Ok(()),
        VerifyResult::BadKey => Err(PairingError::BadKey),
        VerifyResult::BadSig => Err(PairingError::BadSignature(kind)),
    }
}

/// The intermediate chain, the issuer key match, the signature and the
/// issuance time: the checks every team statement shares.
#[allow(clippy::too_many_arguments)]
fn verify_chain<T: SignedPayload>(
    root_set: &RootSet,
    intermediate: &SignedIntermediateCert,
    issuer_key: &PublicKey,
    body: &T,
    signature: &Signature,
    issued_at_ns: u64,
    now_ns: u64,
    kind: CertKind,
) -> Result<(), PairingError> {
    // `RootSet` fields are public, so a caller can hand in a set with
    // threshold 0, which `verify_intermediate` would satisfy with no root
    // signature at all.
    root_set.validate()?;
    verify_intermediate(root_set, intermediate, now_ns)?;
    if *issuer_key != intermediate.cert.intermediate_key {
        return Err(PairingError::IssuerMismatch(kind));
    }
    verify_sig(issuer_key, &body.signing_payload(), signature, kind)?;
    check_issued_at(issued_at_ns, now_ns, kind)
}

// ===========================================================================
// Trust bundle
// ===========================================================================

/// A retired generation whose issuing CA the bundle still carries.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RetiredGeneration {
    /// The retired generation number.
    pub generation: u32,
    /// Its issuing CA certificate, DER.
    pub ca_der: Vec<u8>,
}

/// A team's trust anchors at one generation: the current issuing CA, the
/// issuing CA of generation `generation + 1`, the retired generations the
/// bundle still carries, and the leaf of every robot in the team.
/// [`verify_team_trust_bundle`] checks that the next CA names the bundle's team
/// and the generation after the current one, so a bundle at generation
/// `u32::MAX` never verifies.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct TeamTrustBundle {
    /// Format version.
    pub version: u16,
    /// The team.
    pub team_id: TeamId,
    /// The team short name in every common name.
    pub team_short: TeamShort,
    /// The current generation.
    pub generation: u32,
    /// Issuance time, Unix nanoseconds.
    pub issued_at_ns: u64,
    /// The current issuing CA certificate, DER.
    pub issuing_ca_der: Vec<u8>,
    /// The issuing CA certificate of generation `generation + 1`, DER.
    pub next_ca_der: Vec<u8>,
    /// Retired generations, strictly descending by generation, each below
    /// `generation`.
    pub retired: Vec<RetiredGeneration>,
    /// Robot leaves of the current generation, DER.
    pub robot_leaves: Vec<Vec<u8>>,
    /// The intermediate key that signs this bundle.
    pub issuer_key: PublicKey,
}

impl SignedPayload for TeamTrustBundle {
    const DOMAIN: &'static [u8] = TEAM_TRUST_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u16(self.version)
            .fixed(&self.team_id.0)
            .bytes(self.team_short.as_str().as_bytes())
            .fixed(&self.generation.to_le_bytes())
            .u64(self.issued_at_ns)
            .bytes(&self.issuing_ca_der)
            .bytes(&self.next_ca_der)
            .count(self.retired.len());
        for r in &self.retired {
            w.fixed(&r.generation.to_le_bytes()).bytes(&r.ca_der);
        }
        w.count(self.robot_leaves.len());
        for leaf in &self.robot_leaves {
            w.bytes(leaf);
        }
        w.key(&self.issuer_key);
        w.finish()
    }
}

/// A [`TeamTrustBundle`] with its intermediate signature and the intermediate
/// it chains through.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedTeamTrustBundle {
    /// The bundle.
    pub bundle: TeamTrustBundle,
    /// The intermediate's signature over the bundle signing bytes.
    pub signature: Signature,
    /// The root-signed intermediate whose key is `bundle.issuer_key`.
    pub intermediate: SignedIntermediateCert,
}

impl TeamTrustBundle {
    /// Sign with the intermediate key.
    pub fn sign(
        self,
        intermediate_key: &SigningKey,
        intermediate: SignedIntermediateCert,
    ) -> SignedTeamTrustBundle {
        let signature = crypto::ed25519_sign(intermediate_key, &self.signing_payload());
        SignedTeamTrustBundle {
            bundle: self,
            signature,
            intermediate,
        }
    }
}

/// Verify a trust bundle: the intermediate against `root_set`, the bundle
/// signature, its issuance time, the current, next and retired CA
/// certificates (team and generation must match their slot), and every robot
/// leaf (role robot, issued by the current CA).
pub fn verify_team_trust_bundle(
    root_set: &RootSet,
    signed: &SignedTeamTrustBundle,
    now_ns: u64,
) -> Result<Verified<SignedTeamTrustBundle>, PairingError> {
    let b = &signed.bundle;
    verify_chain(
        root_set,
        &signed.intermediate,
        &b.issuer_key,
        b,
        &signed.signature,
        b.issued_at_ns,
        now_ns,
        CertKind::TeamTrustBundle,
    )?;
    let invalid = PairingError::TeamStatementInvalid;
    let current = check_team_ca(&b.issuing_ca_der)?;
    if current.team != b.team_short || current.generation != b.generation {
        return Err(invalid("the issuing CA names another team or generation"));
    }
    let next = check_team_ca(&b.next_ca_der)?;
    if next.team != b.team_short || Some(next.generation) != b.generation.checked_add(1) {
        return Err(invalid(
            "the next CA must name the same team and the generation after the current one",
        ));
    }
    let mut below = b.generation;
    for r in &b.retired {
        if r.generation >= below {
            return Err(invalid(
                "retired generations must be strictly descending and below the current one",
            ));
        }
        below = r.generation;
        let ca = check_team_ca(&r.ca_der)?;
        if ca.team != b.team_short || ca.generation != r.generation {
            return Err(invalid("a retired CA names another team or generation"));
        }
    }
    for leaf in &b.robot_leaves {
        let facts = verify_leaf_issued_by(leaf, &current)?;
        if facts.role != TeamRole::Robot {
            return Err(invalid("a bundle carries robot leaves only"));
        }
    }
    Ok(Verified(signed.clone()))
}

// ===========================================================================
// Status list
// ===========================================================================

/// A machine status other than active.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
#[repr(u8)]
pub enum DeviceStatus {
    /// Marked stolen.
    Stolen = 1,
    /// Revoked (decommissioned).
    Revoked = 2,
}

/// One machine that is not active.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct StatusEntry {
    /// The machine.
    pub device: DeviceFingerprint,
    /// Its status.
    pub status: DeviceStatus,
    /// The list sequence at which this entry was last changed; at most the
    /// list sequence.
    pub entry_seq: u64,
    /// When the status was set, Unix nanoseconds.
    pub since_ns: u64,
}

/// The team's machines that are stolen or revoked, at one list sequence.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct TeamStatusList {
    /// Format version.
    pub version: u16,
    /// The team.
    pub team_id: TeamId,
    /// The list sequence, increasing by at least one on every change.
    pub seq: u64,
    /// Issuance time, Unix nanoseconds.
    pub issued_at_ns: u64,
    /// Entries, strictly ascending by device fingerprint.
    pub entries: Vec<StatusEntry>,
    /// The intermediate key that signs this list.
    pub issuer_key: PublicKey,
}

impl SignedPayload for TeamStatusList {
    const DOMAIN: &'static [u8] = TEAM_STATUS_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u8(STATUS_LIST_KIND)
            .u16(self.version)
            .fixed(&self.team_id.0)
            .u64(self.seq)
            .u64(self.issued_at_ns)
            .count(self.entries.len());
        for e in &self.entries {
            w.fixed(&e.device.0)
                .u8(e.status as u8)
                .u64(e.entry_seq)
                .u64(e.since_ns);
        }
        w.key(&self.issuer_key);
        w.finish()
    }
}

impl TeamStatusList {
    /// Sign with the intermediate key.
    pub fn sign(
        self,
        intermediate_key: &SigningKey,
        intermediate: SignedIntermediateCert,
    ) -> SignedTeamStatusList {
        let signature = crypto::ed25519_sign(intermediate_key, &self.signing_payload());
        SignedTeamStatusList {
            list: self,
            signature,
            intermediate,
        }
    }

    /// The entry for `device`, if it is listed.
    pub fn entry_for(&self, device: &DeviceFingerprint) -> Option<&StatusEntry> {
        self.entries
            .binary_search_by(|e| e.device.cmp(device))
            .ok()
            .map(|i| &self.entries[i])
    }
}

/// A [`TeamStatusList`] with its intermediate signature and the intermediate
/// it chains through.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedTeamStatusList {
    /// The list.
    pub list: TeamStatusList,
    /// The intermediate's signature over the list signing bytes.
    pub signature: Signature,
    /// The root-signed intermediate whose key is `list.issuer_key`.
    pub intermediate: SignedIntermediateCert,
}

/// Verify a status list: the chain, the signature, its issuance time, entries
/// strictly ascending by fingerprint, and no entry sequence above the list
/// sequence.
pub fn verify_status_list(
    root_set: &RootSet,
    signed: &SignedTeamStatusList,
    now_ns: u64,
) -> Result<Verified<SignedTeamStatusList>, PairingError> {
    let l = &signed.list;
    verify_chain(
        root_set,
        &signed.intermediate,
        &l.issuer_key,
        l,
        &signed.signature,
        l.issued_at_ns,
        now_ns,
        CertKind::TeamStatusList,
    )?;
    if l.entries.windows(2).any(|w| w[0].device >= w[1].device) {
        return Err(PairingError::TeamStatementInvalid(
            "status entries must be strictly ascending by device fingerprint",
        ));
    }
    if l.entries.iter().any(|e| e.entry_seq > l.seq) {
        return Err(PairingError::TeamStatementInvalid(
            "an entry sequence is above the list sequence",
        ));
    }
    Ok(Verified(signed.clone()))
}

// ===========================================================================
// Status freshness
// ===========================================================================

/// The service's statement that, at `service_time_ns`, the team's status list
/// was at `list_seq`, bound to one asker's nonce.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct StatusFreshness {
    /// Format version.
    pub version: u16,
    /// The team.
    pub team_id: TeamId,
    /// The machine that asked.
    pub asker: DeviceFingerprint,
    /// The asker's nonce.
    pub nonce: [u8; 16],
    /// The status list sequence current at the service.
    pub list_seq: u64,
    /// The service time, Unix nanoseconds.
    pub service_time_ns: u64,
    /// The intermediate key that signs this proof.
    pub issuer_key: PublicKey,
}

impl SignedPayload for StatusFreshness {
    const DOMAIN: &'static [u8] = TEAM_STATUS_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u8(STATUS_FRESHNESS_KIND)
            .u16(self.version)
            .fixed(&self.team_id.0)
            .fixed(&self.asker.0)
            .fixed(&self.nonce)
            .u64(self.list_seq)
            .u64(self.service_time_ns)
            .key(&self.issuer_key);
        w.finish()
    }
}

impl StatusFreshness {
    /// Sign with the intermediate key.
    pub fn sign(
        self,
        intermediate_key: &SigningKey,
        intermediate: SignedIntermediateCert,
    ) -> SignedStatusFreshness {
        let signature = crypto::ed25519_sign(intermediate_key, &self.signing_payload());
        SignedStatusFreshness {
            freshness: self,
            signature,
            intermediate,
        }
    }
}

/// A [`StatusFreshness`] with its intermediate signature and the intermediate
/// it chains through.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedStatusFreshness {
    /// The proof.
    pub freshness: StatusFreshness,
    /// The intermediate's signature over the proof signing bytes.
    pub signature: Signature,
    /// The root-signed intermediate whose key is `freshness.issuer_key`.
    pub intermediate: SignedIntermediateCert,
}

/// Verify a freshness proof: the chain, the signature, the service time, and
/// that it answers this asker's question (team, fingerprint and nonce).
pub fn verify_status_freshness(
    root_set: &RootSet,
    signed: &SignedStatusFreshness,
    team_id: &TeamId,
    asker: &DeviceFingerprint,
    nonce: &[u8; 16],
    now_ns: u64,
) -> Result<Verified<SignedStatusFreshness>, PairingError> {
    let f = &signed.freshness;
    verify_chain(
        root_set,
        &signed.intermediate,
        &f.issuer_key,
        f,
        &signed.signature,
        f.service_time_ns,
        now_ns,
        CertKind::StatusFreshness,
    )?;
    if f.team_id != *team_id {
        return Err(PairingError::TeamMismatch);
    }
    if f.asker != *asker || f.nonce != *nonce {
        return Err(PairingError::FreshnessMismatch);
    }
    Ok(Verified(signed.clone()))
}
