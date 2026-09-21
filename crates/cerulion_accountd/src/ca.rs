// SPDX-License-Identifier: AGPL-3.0-only
//! The certificate authority: the issuer end of the SHIPPED `cerulion_pairing`
//! cert chain.
//!
//! Production holds ONLY the rotatable **intermediate** signing key; the root set
//! is air-gapped M-of-N and signs the intermediate in an offline ceremony
//! ([`Ca::from_parts`] loads that shape). For tests + local bring-up,
//! [`Ca::dev_provision`] generates a fresh root set + intermediate IN-PROCESS —
//! this is DEV/TEST provisioning only; the production root ceremony is not
//! implemented here (fork 7, deferred to a dedicated security review).
//!
//! Every artifact issued here is built from `cerulion_pairing`'s own structures
//! and signed with its `.sign()` methods, so the bytes are identical to what the
//! robot-side verifier expects — no second crypto stack, no re-implemented
//! signing-byte layout.

use ed25519_dalek::SigningKey;

use cerulion_pairing::format::{
    AccessListEpoch, AccountId, DeviceCert, Grant, IntermediateCert, PrincipalKind, PublicKey,
    RobotId, RootSet, Scope, SignedDeviceCert, SignedEpoch, SignedGrant, SignedIntermediateCert,
    Validity, FORMAT_VERSION,
};

use crate::config::SEC_NS;
use crate::error::{AccountdError, Result};
use crate::rng;

/// CA / issuance parameters (validity windows + the dev root-ceremony shape).
#[derive(Clone, Debug)]
pub struct CaConfig {
    /// How many root keys the dev ceremony generates.
    pub root_count: u8,
    /// The M-of-N threshold (distinct roots that must sign the intermediate).
    pub root_threshold: u8,
    /// Intermediate-cert lifetime.
    pub intermediate_ttl_ns: u64,
    /// Device-cert lifetime (short — days).
    pub device_cert_ttl_ns: u64,
    /// Grant lifetime.
    pub grant_ttl_ns: u64,
}

impl Default for CaConfig {
    fn default() -> Self {
        CaConfig {
            root_count: 3,
            root_threshold: 2,
            intermediate_ttl_ns: 90 * 24 * 3600 * SEC_NS, // 90 days
            device_cert_ttl_ns: 7 * 24 * 3600 * SEC_NS,   // 7 days
            grant_ttl_ns: 30 * 24 * 3600 * SEC_NS,        // 30 days
        }
    }
}

/// The certificate authority. Holds the rotatable intermediate signing key and
/// the pinned root set, and issues device certs / grants / epochs.
pub struct Ca {
    root_set: RootSet,
    /// The intermediate signing key (the ONLY signing key production holds online).
    intermediate_key: SigningKey,
    intermediate: SignedIntermediateCert,
    /// The CA's stable issuer account id, stamped into grants + epochs.
    issuer_account: AccountId,
    config: CaConfig,
}

impl Ca {
    /// DEV/TEST provisioning: generate a fresh root set + intermediate IN-PROCESS
    /// and self-sign the intermediate M-of-N. **Not a production path** — the
    /// production root ceremony is air-gapped M-of-N and is not implemented here.
    pub fn dev_provision(config: CaConfig, now_ns: u64) -> Result<Self> {
        // Generate the root keys.
        let mut dev_root_keys = Vec::with_capacity(config.root_count as usize);
        for _ in 0..config.root_count {
            dev_root_keys.push(SigningKey::from_bytes(&rng::id_32()?));
        }
        let root_pubs: Vec<PublicKey> = dev_root_keys
            .iter()
            .map(|k| PublicKey(k.verifying_key().to_bytes()))
            .collect();
        let root_set = RootSet::new(root_pubs, config.root_threshold)?;

        // Generate + sign the intermediate (all roots sign; the verifier counts
        // distinct valid trusted roots up to the threshold).
        let intermediate_key = SigningKey::from_bytes(&rng::id_32()?);
        let cert = IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: PublicKey(intermediate_key.verifying_key().to_bytes()),
            validity: Validity {
                not_before_ns: now_ns,
                not_after_ns: now_ns.saturating_add(config.intermediate_ttl_ns),
            },
            issued_at_ns: now_ns,
            // The intermediate may confer up to full owner scope; per-credential
            // bounds come from the device cert + grant scopes.
            max_scope: Scope::OWNER_FULL,
        };
        let root_refs: Vec<&SigningKey> = dev_root_keys.iter().collect();
        let intermediate = cert.sign_by_roots(&root_refs);
        // The root keys are dropped here: they served only to sign the
        // intermediate. Production never holds them online at all (air-gapped
        // M-of-N ceremony); a dev CA re-provisions on restart.

        Ok(Ca {
            root_set,
            intermediate_key,
            intermediate,
            issuer_account: AccountId(rng::id_32()?),
            config,
        })
    }

    /// Load a production-shaped CA from an offline ceremony's outputs: the pinned
    /// root set, the root-signed intermediate, and the intermediate signing key.
    /// (A0 does not exercise this — it exists so the dev path is clearly the
    /// *only* place roots are generated in-process.)
    ///
    /// Refuses with a LOUD `Err` if `intermediate_key` does not form a key pair
    /// with the public key embedded in `intermediate.cert.intermediate_key` — a
    /// mismatched load would sign every issued cert with a key that does not match
    /// the `issuer_key` it stamps, so the shipped verifier's issuer-key check would
    /// fail confusingly downstream (or, worse, verify against the wrong key). The
    /// consistency is asserted HERE, at load, not discovered at first issuance.
    pub fn from_parts(
        root_set: RootSet,
        intermediate: SignedIntermediateCert,
        intermediate_key: SigningKey,
        issuer_account: AccountId,
        config: CaConfig,
    ) -> Result<Self> {
        let derived_public = PublicKey(intermediate_key.verifying_key().to_bytes());
        if derived_public != intermediate.cert.intermediate_key {
            return Err(AccountdError::Internal(
                "intermediate signing key does not match the public key embedded in the \
                 intermediate certificate (mismatched key pair loaded)"
                    .to_string(),
            ));
        }
        Ok(Ca {
            root_set,
            intermediate_key,
            intermediate,
            issuer_account,
            config,
        })
    }

    /// The pinned root set (served at `GET /.well-known/cerulion-roots`).
    pub fn root_set(&self) -> &RootSet {
        &self.root_set
    }

    /// The root-signed intermediate certificate (presented in a pairing chain).
    pub fn intermediate(&self) -> &SignedIntermediateCert {
        &self.intermediate
    }

    /// The intermediate public key (the issuer key stamped into every cert/grant).
    pub fn intermediate_public_key(&self) -> PublicKey {
        self.intermediate.cert.intermediate_key
    }

    /// The CA's issuer account id.
    pub fn issuer_account(&self) -> AccountId {
        self.issuer_account
    }

    /// Issue a short-lived device cert binding `device_key` → `account`. `now_ns`
    /// is the issuance instant (a parameter so the byte-parity cross-check can pin
    /// a fixed time); the validity window is `[now, now + device_cert_ttl)`.
    pub fn issue_device_cert(
        &self,
        device_key: PublicKey,
        account: AccountId,
        principal_kind: PrincipalKind,
        scope: Scope,
        now_ns: u64,
    ) -> SignedDeviceCert {
        DeviceCert {
            version: FORMAT_VERSION,
            device_key,
            account,
            principal_kind,
            scope,
            validity: Validity {
                not_before_ns: now_ns,
                not_after_ns: now_ns.saturating_add(self.config.device_cert_ttl_ns),
            },
            issued_at_ns: now_ns,
            issuer_key: self.intermediate.cert.intermediate_key,
        }
        .sign(&self.intermediate_key)
    }

    /// Issue a depth-0 grant ("`subject` may access `robot` : `scope`"). Grant
    /// issuance *endpoints* land in A4; this method exists so A0's acceptance
    /// cross-check can build a complete pairing presentation the verifier accepts.
    pub fn issue_grant(
        &self,
        subject: AccountId,
        robot: RobotId,
        scope: Scope,
        principal_kind: PrincipalKind,
        now_ns: u64,
    ) -> SignedGrant {
        Grant {
            version: FORMAT_VERSION,
            subject,
            robot,
            scope,
            principal_kind,
            delegation_depth: 0,
            validity: Validity {
                not_before_ns: now_ns,
                not_after_ns: now_ns.saturating_add(self.config.grant_ttl_ns),
            },
            issued_at_ns: now_ns,
            issuer: self.issuer_account,
            issuer_key: self.intermediate.cert.intermediate_key,
        }
        .sign(&self.intermediate_key)
    }

    /// Issue a signed revocation epoch for `robot` (the robot's ACL
    /// sync artifact). Carries BOTH the revoked-account set AND the revoked-device
    /// set so an owner can cut a whole account OR one compromised desk; the robot
    /// applies it monotonically via `TrustStore::apply_epoch`.
    pub fn issue_epoch(
        &self,
        robot: RobotId,
        epoch: u64,
        revoked_accounts: Vec<AccountId>,
        revoked_devices: Vec<PublicKey>,
        now_ns: u64,
    ) -> SignedEpoch {
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot,
            epoch,
            revoked_accounts,
            revoked_devices,
            issued_at_ns: now_ns,
            issuer_key: self.intermediate.cert.intermediate_key,
        }
        .sign(&self.intermediate_key)
    }
}

impl std::fmt::Debug for Ca {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Never print signing-key material.
        f.debug_struct("Ca")
            .field("intermediate_key", &self.intermediate_public_key())
            .field("issuer_account", &self.issuer_account)
            .field("root_keys", &self.root_set.keys.len())
            .field("threshold", &self.root_set.threshold)
            .finish_non_exhaustive()
    }
}
