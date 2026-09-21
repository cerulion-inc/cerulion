// SPDX-License-Identifier: MIT OR Apache-2.0
//! Certificate, grant, and epoch structures with their canonical signing bytes.
//!
//! Every timestamp is **nanoseconds since the Unix epoch (UTC)**.

use ed25519_dalek::SigningKey;
use serde::{Deserialize, Serialize};

use super::canonical::CanonicalWriter;
use super::{AccountId, PublicKey, RobotId, Signature, SignedPayload, FORMAT_VERSION};
use crate::crypto;

// -- domain-separation tags (never reused across types) ----------------------
const INTERMEDIATE_DOMAIN: &[u8] = b"cerulion-pairing:intermediate-cert:v1";
const DEVICE_DOMAIN: &[u8] = b"cerulion-pairing:device-cert:v1";
const GRANT_DOMAIN: &[u8] = b"cerulion-pairing:grant:v1";
// A6 bumped this from `:v1` → `:v2` when `AccessListEpoch` gained
// `revoked_devices` (folded into the signed payload). Nothing has shipped a v1
// epoch, so this is a clean format change, not a migration.
const EPOCH_DOMAIN: &[u8] = b"cerulion-pairing:access-epoch:v2";
// A5: the owner-signed access grant is a DISTINCT artifact from the
// cloud-issued `GRANT_DOMAIN` grant — it is signed by the ROBOT OWNER's device key
// (not the intermediate) and verified OFFLINE against the robot's own owner chain.
// Its own domain tag guarantees an owner-signed access grant can never be replayed
// as a cloud grant (or vice versa). The crate uses the `cerulion-pairing:<type>:v1`
// convention for every domain (NOT the slashy `cerulion/access-grant/v1` form the
// A5 brief loosely suggested) — kept consistent across all five signable types.
const ACCESS_GRANT_DOMAIN: &[u8] = b"cerulion-pairing:access-grant:v1";

/// A role, encoded as an open integer so a robot on older firmware can still
/// parse a certificate carrying a role it does not yet recognize (forward
/// compatibility). Compare/authorize on the numeric value.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Role(pub u16);

impl Role {
    /// Full control, including granting/revoking others. Lowest numeric value =
    /// highest privilege (see [`Scope::intersect`]).
    pub const OWNER: Role = Role(1);
    /// Operate the robot (teleop, run graphs) but not manage access.
    pub const OPERATOR: Role = Role(2);
    /// Read-only observation.
    pub const VIEWER: Role = Role(3);
}

/// Capability bitmask. New capabilities take new bits; unknown bits are
/// preserved through parse/serialize round-trips (forward compatibility).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Scope {
    /// The role.
    pub role: Role,
    /// Fine-grained capability bits (see the `CAP_*` constants).
    pub caps: u64,
}

impl Scope {
    /// May delegate access to another account (a depth-1 grant).
    pub const CAP_DELEGATE: u64 = 1 << 0;
    /// May teleoperate / command the robot.
    pub const CAP_TELEOP: u64 = 1 << 1;
    /// May observe topics / telemetry.
    pub const CAP_OBSERVE: u64 = 1 << 2;
    /// May record / download bags.
    pub const CAP_RECORD: u64 = 1 << 3;

    /// A conservative default scope for code-paired (fallback) entries: observe
    /// only, no teleop, no delegation.
    pub const CODE_PAIR_DEFAULT: Scope = Scope {
        role: Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    };

    /// Full owner scope: the owner role with every capability. Assigned to the
    /// owner row written by a physical-possession claim.
    pub const OWNER_FULL: Scope = Scope {
        role: Role::OWNER,
        caps: Scope::CAP_DELEGATE | Scope::CAP_TELEOP | Scope::CAP_OBSERVE | Scope::CAP_RECORD,
    };

    /// The least-privilege intersection of two scopes: the *higher* (larger)
    /// numeric role and the AND of the capability bits. Used to combine a device
    /// cert's scope with a grant's scope so neither can widen the other.
    pub fn intersect(a: Scope, b: Scope) -> Scope {
        Scope {
            role: Role(a.role.0.max(b.role.0)),
            caps: a.caps & b.caps,
        }
    }

    /// True iff `self` is an **attenuation** of `parent` — it confers no more
    /// privilege than `parent`. A delegated grant must satisfy this against the
    /// delegator's own grant, so delegation can only narrow, never widen, access:
    /// `self`'s role must be no more privileged (numerically ≥) than `parent`'s,
    /// and `self`'s capability bits must be a subset of `parent`'s.
    pub fn is_attenuation_of(&self, parent: &Scope) -> bool {
        self.role.0 >= parent.role.0 && (self.caps & !parent.caps) == 0
    }
}

/// Whether a principal is a human operator or an autonomous/machine account.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
#[repr(u8)]
pub enum PrincipalKind {
    /// A human account.
    Human = 1,
    /// A machine / service account.
    Machine = 2,
}

/// A `[not_before, not_after)` validity window in Unix nanoseconds.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Validity {
    /// Inclusive lower bound (Unix ns).
    pub not_before_ns: u64,
    /// Exclusive upper bound (Unix ns).
    pub not_after_ns: u64,
}

impl Validity {
    /// True iff `now_ns` is within `[not_before, not_after)`.
    pub fn contains(&self, now_ns: u64) -> bool {
        now_ns >= self.not_before_ns && now_ns < self.not_after_ns
    }
}

// ============================================================================
// Root set (the trust anchor) + intermediate certificate (M-of-N root-signed)
// ============================================================================

/// The air-gapped root of trust: a *set* of root public keys with an M-of-N
/// signing threshold. Robots trust the set, not a single key, so a compromised
/// or rotated single root does not break the anchor.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RootSet {
    /// Format version.
    pub version: u16,
    /// The N root public keys.
    pub keys: Vec<PublicKey>,
    /// The threshold M: an intermediate must be signed by at least M distinct
    /// keys from `keys`.
    pub threshold: u8,
}

impl RootSet {
    /// Build and validate a root set.
    pub fn new(keys: Vec<PublicKey>, threshold: u8) -> Result<Self, crate::PairingError> {
        let s = RootSet {
            version: FORMAT_VERSION,
            keys,
            threshold,
        };
        s.validate()?;
        Ok(s)
    }

    /// Structural validity: non-empty, `1 <= threshold <= N`, no duplicate keys.
    pub fn validate(&self) -> Result<(), crate::PairingError> {
        use crate::PairingError::InvalidRootSet;
        if self.keys.is_empty() {
            return Err(InvalidRootSet("root set has no keys"));
        }
        if self.threshold == 0 {
            return Err(InvalidRootSet("threshold must be >= 1"));
        }
        if self.threshold as usize > self.keys.len() {
            return Err(InvalidRootSet("threshold exceeds the number of root keys"));
        }
        for i in 0..self.keys.len() {
            for j in (i + 1)..self.keys.len() {
                if self.keys[i] == self.keys[j] {
                    return Err(InvalidRootSet("duplicate root key"));
                }
            }
        }
        Ok(())
    }
}

/// The rotatable online intermediate certificate. Signed M-of-N by the roots; it
/// in turn issues device certs and grants.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct IntermediateCert {
    /// Format version.
    pub version: u16,
    /// The intermediate's public key (the online issuer key).
    pub intermediate_key: PublicKey,
    /// Validity window.
    pub validity: Validity,
    /// Issuance time (feeds the verifier's anti-rollback high-water mark).
    pub issued_at_ns: u64,
    /// The maximum scope this intermediate may confer (future policy hook).
    pub max_scope: Scope,
}

/// One root's signature over an [`IntermediateCert`].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct RootSignature {
    /// Which root key produced this signature.
    pub root_key: PublicKey,
    /// The signature over the intermediate's canonical signing bytes.
    pub signature: Signature,
}

/// An intermediate certificate with its M-of-N root signatures.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedIntermediateCert {
    /// The certificate body.
    pub cert: IntermediateCert,
    /// Root signatures (verified against the [`RootSet`] threshold).
    pub signatures: Vec<RootSignature>,
}

impl SignedPayload for IntermediateCert {
    const DOMAIN: &'static [u8] = INTERMEDIATE_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u16(self.version)
            .key(&self.intermediate_key)
            .validity(&self.validity)
            .u64(self.issued_at_ns)
            .scope(&self.max_scope);
        w.finish()
    }
}

impl IntermediateCert {
    /// Sign this intermediate with a set of root signing keys (the air-gapped
    /// M-of-N ceremony). Produces one [`RootSignature`] per provided key.
    pub fn sign_by_roots(self, roots: &[&SigningKey]) -> SignedIntermediateCert {
        let payload = self.signing_payload();
        let signatures = roots
            .iter()
            .map(|sk| RootSignature {
                root_key: PublicKey(sk.verifying_key().to_bytes()),
                signature: crypto::ed25519_sign(sk, &payload),
            })
            .collect();
        SignedIntermediateCert {
            cert: self,
            signatures,
        }
    }
}

// ============================================================================
// Device certificate ("device key K belongs to account X until D")
// ============================================================================

/// A short-lived certificate binding a device key to an account. The device key
/// is the transport identity; the verifier asserts it equals the authenticated
/// peer key.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct DeviceCert {
    /// Format version.
    pub version: u16,
    /// The device (transport) public key this cert attests.
    pub device_key: PublicKey,
    /// The account that owns the device key.
    pub account: AccountId,
    /// Human vs. machine principal.
    pub principal_kind: PrincipalKind,
    /// The scope this device key may exercise.
    pub scope: Scope,
    /// Validity window (short-lived).
    pub validity: Validity,
    /// Issuance time (feeds anti-rollback high-water).
    pub issued_at_ns: u64,
    /// The issuer key that signs this cert (the intermediate).
    pub issuer_key: PublicKey,
}

/// A device certificate with its issuer signature.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedDeviceCert {
    /// The certificate body.
    pub cert: DeviceCert,
    /// The issuer's signature over the canonical signing bytes.
    pub signature: Signature,
}

impl SignedPayload for DeviceCert {
    const DOMAIN: &'static [u8] = DEVICE_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u16(self.version)
            .key(&self.device_key)
            .account(&self.account)
            .principal(self.principal_kind)
            .scope(&self.scope)
            .validity(&self.validity)
            .u64(self.issued_at_ns)
            .key(&self.issuer_key);
        w.finish()
    }
}

impl DeviceCert {
    /// Sign this cert with the issuer (intermediate) key.
    pub fn sign(self, issuer: &SigningKey) -> SignedDeviceCert {
        let signature = crypto::ed25519_sign(issuer, &self.signing_payload());
        SignedDeviceCert {
            cert: self,
            signature,
        }
    }
}

impl SignedDeviceCert {
    /// The account this cert binds `device_key` to — **iff** the cert attests
    /// exactly `device_key` (the app-layer device↔account binding read,
    /// invariant I1). A cert that attests a DIFFERENT key describes a different
    /// machine's binding and is refused with [`PeerKeyMismatch`] — never trusted
    /// as this key's account.
    ///
    /// [`PeerKeyMismatch`]: crate::PairingError::PeerKeyMismatch
    ///
    /// This is a pure identity read: it does **not** verify the issuer signature
    /// chain (that is the robot-side verifier's job at pairing time — see
    /// [`crate::verify`]). A desk / gateway trusts a cert it obtained over its OWN
    /// authenticated login session, so it needs only the I1 key match to know which
    /// cloud account the cert binds its device key to. It is the ONE binding-check
    /// shared by the desk-side resolvers (`cerulion_cli_engine::device_binding` and
    /// `cerulion_wireclient::config::resolve_desk_account`) so the two can never
    /// derive a DIFFERENT account from the same cert (the "reuse, do not
    /// invent a parallel binding" rule).
    pub fn account_for_device_key(
        &self,
        device_key: &PublicKey,
    ) -> Result<AccountId, crate::PairingError> {
        if self.cert.device_key != *device_key {
            return Err(crate::PairingError::PeerKeyMismatch);
        }
        Ok(self.cert.account)
    }
}

// ============================================================================
// Grant ("account B may access robot R")
// ============================================================================

/// An issuer-signed grant of access from an account to a robot. Grants travel
/// *with* the client; the robot verifies them offline.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Grant {
    /// Format version.
    pub version: u16,
    /// The account being granted access.
    pub subject: AccountId,
    /// The robot the access is for.
    pub robot: RobotId,
    /// The granted scope.
    pub scope: Scope,
    /// Human vs. machine.
    pub principal_kind: PrincipalKind,
    /// Delegation depth. `0` = issued directly by the intermediate; `1` = a
    /// single delegation. Capped at [`crate::MAX_DELEGATION_DEPTH`] at verify.
    pub delegation_depth: u8,
    /// Validity window.
    pub validity: Validity,
    /// Issuance time (feeds anti-rollback high-water).
    pub issued_at_ns: u64,
    /// The issuing account.
    pub issuer: AccountId,
    /// The key that signs this grant (the intermediate for depth 0; a delegator
    /// device key for depth 1).
    pub issuer_key: PublicKey,
}

/// A grant with its issuer signature.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedGrant {
    /// The grant body.
    pub grant: Grant,
    /// The issuer's signature.
    pub signature: Signature,
}

impl SignedPayload for Grant {
    const DOMAIN: &'static [u8] = GRANT_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u16(self.version)
            .account(&self.subject)
            .robot(&self.robot)
            .scope(&self.scope)
            .principal(self.principal_kind)
            .u8(self.delegation_depth)
            .validity(&self.validity)
            .u64(self.issued_at_ns)
            .account(&self.issuer)
            .key(&self.issuer_key);
        w.finish()
    }
}

impl Grant {
    /// Sign this grant with the issuer key.
    pub fn sign(self, issuer: &SigningKey) -> SignedGrant {
        let signature = crypto::ed25519_sign(issuer, &self.signing_payload());
        SignedGrant {
            grant: self,
            signature,
        }
    }
}

/// The material needed to verify a **depth-1 (delegated)** grant: the delegator's
/// own depth-0 grant (which must carry [`Scope::CAP_DELEGATE`]) and the delegator's
/// device cert (which binds the delegating signing key to the delegator account).
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct Delegation {
    /// The delegator's depth-0 grant (issued by the intermediate).
    pub parent: SignedGrant,
    /// The delegator's device cert, binding the depth-1 grant's `issuer_key` to
    /// the delegator account.
    pub delegator_cert: SignedDeviceCert,
}

// ============================================================================
// Owner-signed access grant — desk-carried, robot-verified offline
// ============================================================================

/// An **owner-signed** grant of access to a robot: "the owner of robot R grants
/// account B access at scope S". Distinct from a cloud-issued [`Grant`] (which the
/// online intermediate signs): an [`AccessGrant`] is signed by the ROBOT OWNER's
/// own device key, so the robot verifies it OFFLINE against its OWN owner chain —
/// no cloud contact, ever (the A5 "desk-carried owner-signed grants" model;
/// the robot-cached-ACL sync path is deliberately NOT built).
///
/// The desk carries the grant (bundled with the owner's device cert + the
/// intermediate) and presents it at dial time; the robot's verifier checks that the
/// signing key belongs to THIS robot's claimed owner (via the presented owner device
/// cert) before admitting the subject. See
/// [`crate::verify::TrustStore::establish_by_owner_grant`].
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct AccessGrant {
    /// Format version.
    pub version: u16,
    /// The account being granted access.
    pub subject: AccountId,
    /// The robot the access is for.
    pub robot: RobotId,
    /// The granted scope. Bounded at verify to an attenuation of the owner's own
    /// device-cert scope (an owner can never grant more than it holds).
    pub scope: Scope,
    /// Human vs. machine principal for the granted subject.
    pub principal_kind: PrincipalKind,
    /// Validity window. `not_after_ns == u64::MAX` encodes an unbounded (no-expiry)
    /// grant; a finite bound gives the owner an offline-honorable expiry. The bound
    /// is honored at BOTH ends: at presentation the owner-grant verifier rejects an
    /// out-of-window grant, and the finite bound is stamped onto the
    /// durable [`crate::verify::AccessRow::expires_at_ns`] so
    /// [`crate::verify::TrustStore::is_allowed`] DENIES the established row once the
    /// robot's clock passes it, with NO explicit revoke. A time-boxed contractor grant
    /// therefore stops working on its own, offline.
    pub validity: Validity,
    /// Issuance time (feeds the verifier's anti-rollback high-water mark).
    pub issued_at_ns: u64,
    /// The OWNER account issuing this grant. The verifier asserts this equals the
    /// robot's claimed owner (a non-owner-signed grant is refused).
    pub owner: AccountId,
    /// The owner's **device (signing) key** that signs this grant. The verifier
    /// asserts the presented owner device cert binds this exact key to `owner`, then
    /// checks the signature against it.
    pub owner_device_key: PublicKey,
}

/// An [`AccessGrant`] with the owner's device-key signature.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedAccessGrant {
    /// The grant body.
    pub grant: AccessGrant,
    /// The owner device key's signature over the canonical signing bytes.
    pub signature: Signature,
}

impl SignedPayload for AccessGrant {
    const DOMAIN: &'static [u8] = ACCESS_GRANT_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u16(self.version)
            .account(&self.subject)
            .robot(&self.robot)
            .scope(&self.scope)
            .principal(self.principal_kind)
            .validity(&self.validity)
            .u64(self.issued_at_ns)
            .account(&self.owner)
            .key(&self.owner_device_key);
        w.finish()
    }
}

impl AccessGrant {
    /// Sign this access grant with the owner's device signing key. The caller MUST
    /// pass the key whose public half equals `self.owner_device_key`; the verifier
    /// checks the signature against `owner_device_key`, so a mismatch fails there.
    pub fn sign(self, owner_device_key: &SigningKey) -> SignedAccessGrant {
        let signature = crypto::ed25519_sign(owner_device_key, &self.signing_payload());
        SignedAccessGrant {
            grant: self,
            signature,
        }
    }
}

// ============================================================================
// Access-list epoch (CRL-lite revocation)
// ============================================================================

/// A monotonic, signed revocation epoch. Every trusted client pushes the latest
/// epoch on connect; the robot applies it if the epoch number is newer.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct AccessListEpoch {
    /// Format version.
    pub version: u16,
    /// The robot this epoch is for.
    pub robot: RobotId,
    /// The monotonic epoch number.
    pub epoch: u64,
    /// Accounts revoked as of this epoch.
    pub revoked_accounts: Vec<AccountId>,
    /// Device (transport) keys revoked as of this epoch. A revoked
    /// device is denied at the accept gate INDEPENDENTLY of its account row, so an
    /// owner can cut ONE compromised desk without revoking the whole account — the
    /// account's OTHER devices keep their access. Like `revoked_accounts` this set is
    /// authoritative and monotonic: `apply_epoch` REPLACES it wholesale, so a newer
    /// epoch dropping a device is the only re-admit (a revoked device cannot
    /// resurrect itself by re-presenting an old signed grant — the tombstone).
    pub revoked_devices: Vec<PublicKey>,
    /// Issuance time (feeds anti-rollback high-water).
    pub issued_at_ns: u64,
    /// The key that signs this epoch (the intermediate).
    pub issuer_key: PublicKey,
}

/// A signed access-list epoch.
#[derive(Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub struct SignedEpoch {
    /// The epoch body.
    pub epoch_data: AccessListEpoch,
    /// The issuer's signature.
    pub signature: Signature,
}

impl SignedPayload for AccessListEpoch {
    const DOMAIN: &'static [u8] = EPOCH_DOMAIN;
    fn signing_payload(&self) -> Vec<u8> {
        let mut w = CanonicalWriter::new(Self::DOMAIN);
        w.u16(self.version)
            .robot(&self.robot)
            .u64(self.epoch)
            .count(self.revoked_accounts.len());
        for a in &self.revoked_accounts {
            w.account(a);
        }
        // A6: the revoked-device set is folded into the SAME signed payload
        // (its own length prefix then each key) so an owner's device revocation is
        // as tamper-evident as an account revocation. Appended AFTER the accounts
        // collection, BEFORE `issued_at_ns` — the `EPOCH_DOMAIN` bump to `:v2`
        // marks this layout change (nothing has shipped a v1 epoch).
        w.count(self.revoked_devices.len());
        for d in &self.revoked_devices {
            w.key(d);
        }
        w.u64(self.issued_at_ns).key(&self.issuer_key);
        w.finish()
    }
}

impl AccessListEpoch {
    /// Sign this epoch with the issuer key.
    pub fn sign(self, issuer: &SigningKey) -> SignedEpoch {
        let signature = crypto::ed25519_sign(issuer, &self.signing_payload());
        SignedEpoch {
            epoch_data: self,
            signature,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(role: u16, caps: u64) -> Scope {
        Scope {
            role: Role(role),
            caps,
        }
    }

    #[test]
    fn is_attenuation_of_oracle_vectors() {
        const OWNER: u16 = 1;
        const OP: u16 = 2;
        const VIEWER: u16 = 3;
        const A: u64 = 1 << 0;
        const B: u64 = 1 << 1;
        const C: u64 = 1 << 2;

        // (self, parent, expected). `self` attenuates `parent` iff it is NO more
        // privileged: numerically-higher-or-equal role AND a subset of the caps.
        let cases: &[(Scope, Scope, bool)] = &[
            // Reflexive: a scope always attenuates itself.
            (s(OP, A | B), s(OP, A | B), true),
            // Narrower role (higher number) with subset caps attenuates.
            (s(VIEWER, C), s(OP, C), true),
            // Wider role (lower number) is NOT an attenuation, even with equal caps.
            (s(OWNER, C), s(OP, C), false),
            // Same role, strict-subset caps attenuates.
            (s(OP, C), s(OP, B | C), true),
            // Same role, an EXTRA cap bit not in the parent does NOT attenuate.
            (s(OP, B | C), s(OP, C), false),
            // Empty caps always attenuates (subset of anything) at an equal role.
            (s(OP, 0), s(OP, A | B | C), true),
            // OWNER_FULL is never an attenuation of the conservative code-pair default.
            (Scope::OWNER_FULL, Scope::CODE_PAIR_DEFAULT, false),
            // ...and the code-pair default IS an attenuation of OWNER_FULL.
            (Scope::CODE_PAIR_DEFAULT, Scope::OWNER_FULL, true),
        ];
        for (i, (child, parent, expected)) in cases.iter().enumerate() {
            assert_eq!(
                child.is_attenuation_of(parent),
                *expected,
                "case {i}: {child:?} attenuates {parent:?}?"
            );
        }
    }

    /// Build a signed device cert binding `device_key` → `account` (a throwaway
    /// issuer — `account_for_device_key` does NOT check the signature).
    fn signed_cert(device_key: [u8; 32], account: [u8; 32]) -> SignedDeviceCert {
        let issuer = SigningKey::from_bytes(&[0xAB; 32]);
        DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(device_key),
            account: AccountId(account),
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: u64::MAX,
            },
            issued_at_ns: 0,
            issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
        }
        .sign(&issuer)
    }

    /// I1: a cert whose `device_key` matches yields the bound account;
    /// a cert for a DIFFERENT key is refused with `PeerKeyMismatch`. Hand oracle —
    /// the returned account bytes are the exact bytes put in, NOT a self-compare of
    /// two derivations.
    #[test]
    fn account_for_device_key_matches_and_rejects() {
        let key = [7u8; 32];
        let account = [0x42u8; 32];
        let cert = signed_cert(key, account);

        // Matching key → the exact bound account.
        assert_eq!(
            cert.account_for_device_key(&PublicKey(key)).unwrap(),
            AccountId(account),
            "a matching device key resolves to the bound account"
        );

        // A different key → PeerKeyMismatch (never a fabricated/silent account).
        let other = [8u8; 32];
        assert!(
            matches!(
                cert.account_for_device_key(&PublicKey(other)),
                Err(crate::PairingError::PeerKeyMismatch)
            ),
            "a cert for a different device key must be refused"
        );
    }
}
