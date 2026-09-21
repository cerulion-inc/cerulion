// SPDX-License-Identifier: MIT OR Apache-2.0
//! Device-side (client) pairing support: the key-management seam, cert carry, and
//! a thin pairing driver.
//!
//! The client holds the device identity (the ed25519 key whose public half is the
//! transport `EndpointId`), carries its certificate chain + grants, and drives the
//! two pairing paths. The **strong** path produces a [`PairingPresentation`]; the
//! **fallback** CPace path is a *separate, explicit* entry point — it is never
//! auto-negotiated after a strong-path failure (that deliberate user choice lives
//! at a higher layer).

use ed25519_dalek::SigningKey;

use crate::crypto;
use crate::format::{
    Delegation, PublicKey, RobotId, Signature, SignedDeviceCert, SignedGrant,
    SignedIntermediateCert,
};
use crate::pake::{CpaceInitiator, PakeIdentities};
use crate::verify::PairingPresentation;

/// The device identity: an ed25519 keypair whose 32-byte public key **is** the
/// transport identity (the iroh `EndpointId`).
///
/// This is the key-management **seam**: the firmware supplies the 32-byte seed
/// from its secure storage via [`DeviceIdentity::from_seed`]. This crate holds no
/// persistence or key-generation policy — a caller that needs a fresh key draws
/// 32 bytes of entropy from its platform RNG and passes them here.
pub struct DeviceIdentity {
    signing_key: SigningKey,
}

impl core::fmt::Debug for DeviceIdentity {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Never print secret material; the public key is safe to show.
        write!(f, "DeviceIdentity(pub={:?})", self.public_key())
    }
}

impl DeviceIdentity {
    /// Construct from a 32-byte ed25519 seed (from firmware secure storage).
    pub fn from_seed(seed: &[u8; 32]) -> Self {
        DeviceIdentity {
            signing_key: SigningKey::from_bytes(seed),
        }
    }

    /// The device (transport) public key.
    pub fn public_key(&self) -> PublicKey {
        PublicKey(self.signing_key.verifying_key().to_bytes())
    }

    /// Sign an arbitrary message with the device key.
    pub fn sign(&self, msg: &[u8]) -> Signature {
        crypto::ed25519_sign(&self.signing_key, msg)
    }

    /// Borrow the raw signing key — needed to sign a delegated (depth-1) grant or
    /// a device cert as this device. Handle with care.
    pub fn signing_key(&self) -> &SigningKey {
        &self.signing_key
    }
}

/// The certificate material the client carries and presents. Grants travel with
/// the client so the robot never has to poll the cloud.
#[derive(Clone, Debug)]
pub struct CertBundle {
    /// The root-signed intermediate cert.
    pub intermediate: SignedIntermediateCert,
    /// This device's cert.
    pub device_cert: SignedDeviceCert,
    /// The grants the client may present — typically one per robot, but a robot
    /// may have more than one (e.g. a direct depth-0 grant AND a delegated depth-1
    /// grant); [`CertBundle::presentation_for`] selects a verification-shaped one.
    pub grants: Vec<SignedGrant>,
    /// The delegation chains for this client's delegated (depth-1) grants — one
    /// per delegator the client presents on behalf of. A single delegation binds
    /// exactly one robot (its depth-0 parent grant is robot-scoped, per
    /// `verify::chain::verify_delegation`), so a multi-robot delegated client
    /// carries one entry per robot; [`CertBundle::presentation_for`] selects the
    /// one that binds the requested robot's grant.
    pub delegations: Vec<Delegation>,
}

impl CertBundle {
    /// A bundle with an intermediate + device cert and no grants yet.
    pub fn new(intermediate: SignedIntermediateCert, device_cert: SignedDeviceCert) -> Self {
        CertBundle {
            intermediate,
            device_cert,
            grants: Vec::new(),
            delegations: Vec::new(),
        }
    }

    /// Add a grant.
    pub fn with_grant(mut self, grant: SignedGrant) -> Self {
        self.grants.push(grant);
        self
    }

    /// Attach a delegation chain (for a depth-1 grant). **Additive** — call once
    /// per delegator/robot the client carries a delegated grant for.
    /// [`CertBundle::presentation_for`] selects the entry that binds the target
    /// robot's grant, so a multi-robot delegated client pushes one per robot.
    pub fn with_delegation(mut self, delegation: Delegation) -> Self {
        self.delegations.push(delegation);
        self
    }

    /// Build the presentation for a specific robot, if a grant for it is carried.
    ///
    /// The grant is chosen to be **verification-shaped** — among the grants
    /// carried for this robot, a depth-0 (direct) grant is preferred (it verifies
    /// on its own, needing no delegation), else a depth-1 (delegated) grant that
    /// has a matching delegation, else the first grant for the robot (a plain
    /// failure downstream). So a bundle carrying BOTH a direct and a delegated
    /// grant for one robot — e.g. a teammate upgraded from delegated to direct
    /// access without pruning the old grant — presents the one that verifies, not
    /// whichever was pushed first.
    ///
    /// For a delegated (depth-1) grant this ALSO selects the delegation that
    /// binds this robot's grant — every binding
    /// `verify::chain::verify_delegation` enforces, so a bundle carrying more than
    /// one delegation with the same delegator device key cannot select one the
    /// verifier would reject: the delegation's depth-0 parent grant is for this
    /// robot (`parent.grant.robot == robot`) and names the depth-1 grant's issuer
    /// as its subject (`parent.grant.subject == grant.issuer`), and the delegator
    /// cert binds the delegator account AND signing key to that same issuer
    /// (`delegator_cert.cert.account == grant.issuer` AND
    /// `delegator_cert.cert.device_key == grant.issuer_key`). A depth-0 grant has
    /// no matching delegation, so `delegation` stays `None` for it.
    pub fn presentation_for(&self, robot: &RobotId) -> Option<PairingPresentation> {
        // The delegation that binds a specific grant — every binding
        // `verify::chain::verify_delegation` enforces on the parent grant + the
        // delegator cert. `None` for a grant with no such delegation carried.
        let delegation_for = |grant: &SignedGrant| -> Option<Delegation> {
            self.delegations
                .iter()
                .find(|d| {
                    d.parent.grant.robot == *robot
                        && d.parent.grant.subject == grant.grant.issuer
                        && d.delegator_cert.cert.account == grant.grant.issuer
                        && d.delegator_cert.cert.device_key == grant.grant.issuer_key
                })
                .cloned()
        };

        // Prefer a grant that will actually verify: a depth-0 (direct) grant needs
        // no delegation; else a depth-1 (delegated) grant WITH a matching
        // delegation (selected as a pair); else the first robot-matching grant, a
        // plain failure downstream.
        let (grant, delegation) = if let Some(direct) = self
            .grants
            .iter()
            .find(|g| g.grant.robot == *robot && g.grant.delegation_depth == 0)
        {
            (direct.clone(), None)
        } else if let Some((delegated, del)) = self
            .grants
            .iter()
            .filter(|g| g.grant.robot == *robot)
            .find_map(|g| delegation_for(g).map(|d| (g, d)))
        {
            (delegated.clone(), Some(del))
        } else {
            let first = self.grants.iter().find(|g| g.grant.robot == *robot)?;
            let del = delegation_for(first);
            (first.clone(), del)
        };

        Some(PairingPresentation {
            intermediate: self.intermediate.clone(),
            device_cert: self.device_cert.clone(),
            grant,
            delegation,
        })
    }
}

/// A thin client-side pairing driver over an identity + a cert bundle.
#[derive(Debug)]
pub struct PairingClient {
    identity: DeviceIdentity,
    bundle: CertBundle,
}

impl PairingClient {
    /// Build a client from its identity and cert bundle.
    pub fn new(identity: DeviceIdentity, bundle: CertBundle) -> Self {
        PairingClient { identity, bundle }
    }

    /// The client's device (transport) public key.
    pub fn public_key(&self) -> PublicKey {
        self.identity.public_key()
    }

    /// The device identity (key seam).
    pub fn identity(&self) -> &DeviceIdentity {
        &self.identity
    }

    /// The carried cert bundle.
    pub fn bundle(&self) -> &CertBundle {
        &self.bundle
    }

    /// **Strong path:** the presentation to send to `robot`, if a grant for it is
    /// carried. `None` means the client has no grant for that robot (it must fall
    /// back — a deliberate, separate choice).
    pub fn strong_presentation(&self, robot: &RobotId) -> Option<PairingPresentation> {
        self.bundle.presentation_for(robot)
    }

    /// **Fallback path:** begin a CPace ceremony against `responder_key`. This is a
    /// *separate, explicit* call — it is never invoked automatically as a
    /// consequence of a strong-path failure. `context` is channel-binding material
    /// (e.g. a TLS exporter) shared with the responder.
    pub fn begin_code_pairing(
        &self,
        code: impl Into<String>,
        responder_key: PublicKey,
        context: Vec<u8>,
    ) -> CpaceInitiator {
        let identities = PakeIdentities::new(self.identity.public_key(), responder_key, context);
        CpaceInitiator::new(code, identities)
    }
}
