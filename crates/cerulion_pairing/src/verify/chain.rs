// SPDX-License-Identifier: MIT OR Apache-2.0
//! Pure certificate-chain verification: crypto + structural checks only. No
//! persisted state (rollback floor, revocation, ownership) is touched here — that
//! belongs to [`crate::verify::TrustStore`].

use crate::crypto::{self, VerifyResult};
use crate::error::{CertKind, PairingError};
use crate::format::{
    AccountId, Delegation, PublicKey, RobotId, RootSet, Scope, Signature, SignedAccessGrant,
    SignedDeviceCert, SignedGrant, SignedIntermediateCert, SignedPayload, Validity,
};
use crate::MAX_DELEGATION_DEPTH;

/// Allowed forward clock skew for issuance times (5 minutes). A credential whose
/// `issued_at_ns` is beyond `now + this` is rejected — issuance can never be
/// meaningfully in the future, and accepting one would let it poison the
/// anti-rollback high-water mark.
pub(crate) const ALLOWED_ISSUANCE_SKEW_NS: u64 = 5 * 60 * 1_000_000_000;

/// Reject a credential that claims issuance in the future beyond the allowed skew.
pub(crate) fn check_issued_at(
    issued_at_ns: u64,
    now_ns: u64,
    kind: CertKind,
) -> Result<(), PairingError> {
    if issued_at_ns > now_ns.saturating_add(ALLOWED_ISSUANCE_SKEW_NS) {
        return Err(PairingError::IssuedInFuture {
            kind,
            issued_at_ns,
            now_ns,
        });
    }
    Ok(())
}

/// `now_ns` must be within the window, else a precise not-yet / expired error.
fn check_validity(v: &Validity, now_ns: u64, kind: CertKind) -> Result<(), PairingError> {
    if now_ns < v.not_before_ns {
        return Err(PairingError::NotYetValid {
            kind,
            now_ns,
            not_before_ns: v.not_before_ns,
        });
    }
    if now_ns >= v.not_after_ns {
        return Err(PairingError::Expired {
            kind,
            now_ns,
            not_after_ns: v.not_after_ns,
        });
    }
    Ok(())
}

fn verify_sig(
    key: &PublicKey,
    msg: &[u8],
    sig: &Signature,
    kind: CertKind,
) -> Result<(), PairingError> {
    match crypto::ed25519_verify(key, msg, sig) {
        VerifyResult::Ok => Ok(()),
        VerifyResult::BadKey => Err(PairingError::BadKey),
        VerifyResult::BadSig => Err(PairingError::BadSignature(kind)),
    }
}

/// Verify the intermediate against the root set: time-valid AND signed by at
/// least `threshold` **distinct** trusted roots. Untrusted-root and duplicate
/// signatures are ignored (not errors) — only distinct valid trusted roots count.
pub(crate) fn verify_intermediate(
    root_set: &RootSet,
    signed: &SignedIntermediateCert,
    now_ns: u64,
) -> Result<(), PairingError> {
    check_validity(&signed.cert.validity, now_ns, CertKind::Intermediate)?;
    check_issued_at(signed.cert.issued_at_ns, now_ns, CertKind::Intermediate)?;
    let payload = signed.cert.signing_payload();
    let mut distinct: Vec<PublicKey> = Vec::new();
    for rs in &signed.signatures {
        if !root_set.keys.contains(&rs.root_key) {
            continue; // signature from a key not in the trusted set
        }
        if distinct.contains(&rs.root_key) {
            continue; // the same root signed twice — counts once
        }
        if matches!(
            crypto::ed25519_verify(&rs.root_key, &payload, &rs.signature),
            VerifyResult::Ok
        ) {
            distinct.push(rs.root_key);
        }
    }
    if distinct.len() < root_set.threshold as usize {
        return Err(PairingError::RootThresholdNotMet {
            have: distinct.len().min(u8::MAX as usize) as u8,
            need: root_set.threshold,
        });
    }
    Ok(())
}

/// Verify a device cert's issuer CHAIN only — issued by the intermediate,
/// time-valid, and its scope within the intermediate's `max_scope`. Deliberately
/// does NOT assert the `device_key == authenticated peer` binding, so it can verify
/// a cert whose subject is NOT the dialing peer (the A5 owner cert: the
/// owner signs an access grant but is not the party dialing the robot). The
/// peer-binding gate is layered on top by [`verify_device_cert`].
pub(crate) fn verify_device_cert_chain(
    signed: &SignedDeviceCert,
    intermediate_key: &PublicKey,
    intermediate_max_scope: &Scope,
    now_ns: u64,
) -> Result<(), PairingError> {
    if signed.cert.issuer_key != *intermediate_key {
        return Err(PairingError::IssuerMismatch(CertKind::Device));
    }
    verify_sig(
        &signed.cert.issuer_key,
        &signed.cert.signing_payload(),
        &signed.signature,
        CertKind::Device,
    )?;
    check_validity(&signed.cert.validity, now_ns, CertKind::Device)?;
    check_issued_at(signed.cert.issued_at_ns, now_ns, CertKind::Device)?;
    // The device cert is intermediate-issued; it may not confer a scope wider than
    // the intermediate's own declared `max_scope` (checked after authenticating it).
    if !signed.cert.scope.is_attenuation_of(intermediate_max_scope) {
        return Err(PairingError::ScopeExceedsIntermediate);
    }
    Ok(())
}

/// Verify the device cert: the full issuer chain ([`verify_device_cert_chain`])
/// AND its `device_key` equal to the authenticated peer key (the app-layer
/// binding — the offline chain proves K belongs to the account, and the transport
/// proved the peer *is* K).
pub(crate) fn verify_device_cert(
    signed: &SignedDeviceCert,
    intermediate_key: &PublicKey,
    intermediate_max_scope: &Scope,
    authenticated_peer_key: &PublicKey,
    now_ns: u64,
) -> Result<(), PairingError> {
    verify_device_cert_chain(signed, intermediate_key, intermediate_max_scope, now_ns)?;
    if signed.cert.device_key != *authenticated_peer_key {
        return Err(PairingError::PeerKeyMismatch);
    }
    Ok(())
}

/// A5: verify an **owner-signed** access grant against the already-verified
/// owner device cert. Pure crypto + structural checks; the store layer supplies the
/// owner device key + scope (from a cert it verified) and the robot's claimed owner,
/// and applies revocation/rollback separately. Checks, in order:
///
/// 1. the grant is for THIS robot ([`PairingError::WrongRobot`]);
/// 2. the grant's `subject` equals the verified subject account
///    ([`PairingError::SubjectMismatch`]);
/// 3. the grant's stated `owner` equals the robot's claimed owner
///    ([`PairingError::NotRobotOwner`]);
/// 4. the grant's `owner_device_key` equals the presented owner cert's device key
///    ([`PairingError::OwnerGrantKeyMismatch`]) — the grant is signed by exactly the
///    key the owner cert authenticates;
/// 5. time validity + issuance skew (as [`CertKind::AccessGrant`]);
/// 6. the ed25519 signature verifies against `owner_device_key`;
/// 7. the granted scope is an attenuation of the owner's own device-cert scope
///    ([`PairingError::OwnerGrantScopeExceedsOwner`]) — an owner can never grant more
///    than it holds.
#[allow(clippy::too_many_arguments)]
pub(crate) fn verify_owner_access_grant(
    signed: &SignedAccessGrant,
    owner_device_key: &PublicKey,
    owner_cert_scope: &Scope,
    robot: &RobotId,
    subject_account: &AccountId,
    owner_account: &AccountId,
    now_ns: u64,
) -> Result<(), PairingError> {
    let g = &signed.grant;
    if g.robot != *robot {
        return Err(PairingError::WrongRobot);
    }
    if g.subject != *subject_account {
        return Err(PairingError::SubjectMismatch);
    }
    if g.owner != *owner_account {
        return Err(PairingError::NotRobotOwner);
    }
    if g.owner_device_key != *owner_device_key {
        return Err(PairingError::OwnerGrantKeyMismatch);
    }
    check_validity(&g.validity, now_ns, CertKind::AccessGrant)?;
    check_issued_at(g.issued_at_ns, now_ns, CertKind::AccessGrant)?;
    verify_sig(
        &g.owner_device_key,
        &g.signing_payload(),
        &signed.signature,
        CertKind::AccessGrant,
    )?;
    // ATTENUATION: the owner may confer no more than its own device cert holds.
    if !g.scope.is_attenuation_of(owner_cert_scope) {
        return Err(PairingError::OwnerGrantScopeExceedsOwner);
    }
    Ok(())
}

/// Verify the grant: correct robot + subject, delegation depth capped, time
/// valid, signed by an authorized issuer (the intermediate for depth 0, a
/// delegator for depth 1), and scope-bounded by the intermediate's `max_scope`
/// (for a depth-1 grant, the intermediate-issued depth-0 parent grant is bounded
/// too). Revocation is checked by the store, not here.
pub(crate) fn verify_grant(
    signed: &SignedGrant,
    intermediate_key: &PublicKey,
    intermediate_max_scope: &Scope,
    robot: &RobotId,
    subject_account: &AccountId,
    now_ns: u64,
    delegation: Option<&Delegation>,
) -> Result<(), PairingError> {
    let g = &signed.grant;
    if g.robot != *robot {
        return Err(PairingError::WrongRobot);
    }
    if g.subject != *subject_account {
        return Err(PairingError::SubjectMismatch);
    }
    if g.delegation_depth > MAX_DELEGATION_DEPTH {
        return Err(PairingError::DelegationDepthExceeded {
            depth: g.delegation_depth,
            max: MAX_DELEGATION_DEPTH,
        });
    }
    check_validity(&g.validity, now_ns, CertKind::Grant)?;
    check_issued_at(g.issued_at_ns, now_ns, CertKind::Grant)?;

    match g.delegation_depth {
        0 => {
            if g.issuer_key != *intermediate_key {
                return Err(PairingError::IssuerMismatch(CertKind::Grant));
            }
            verify_sig(
                &g.issuer_key,
                &g.signing_payload(),
                &signed.signature,
                CertKind::Grant,
            )?;
        }
        1 => {
            let del = delegation.ok_or(PairingError::InvalidDelegation)?;
            verify_delegation(
                del,
                intermediate_key,
                robot,
                now_ns,
                &g.issuer,
                &g.issuer_key,
            )?;
            // The delegator's depth-0 parent grant is intermediate-issued; it may
            // not exceed the intermediate's declared max_scope (checked after
            // `verify_delegation` has authenticated it). Without this, a
            // VIEWER-scoped intermediate could sign a parent grant carrying
            // OWNER_FULL and the delegatee would inherit privilege the intermediate
            // was never authorized to confer.
            if !del
                .parent
                .grant
                .scope
                .is_attenuation_of(intermediate_max_scope)
            {
                return Err(PairingError::ScopeExceedsIntermediate);
            }
            verify_sig(
                &g.issuer_key,
                &g.signing_payload(),
                &signed.signature,
                CertKind::Grant,
            )?;
            // ATTENUATION (after authenticating the grant): a delegated grant may
            // never confer more privilege than the delegator's own parent grant —
            // no privilege escalation via delegation.
            if !g.scope.is_attenuation_of(&del.parent.grant.scope) {
                return Err(PairingError::ScopeEscalation);
            }
        }
        // > MAX_DELEGATION_DEPTH already rejected above; MAX is 1, so this is
        // unreachable, but kept total rather than `unreachable!()`.
        _ => return Err(PairingError::InvalidDelegation),
    }
    // The presented grant's own scope must not exceed the intermediate's max_scope.
    // For depth 0 this is THE intermediate-issuance bound; for depth 1 it is
    // implied transitively (grant <= parent <= max via the checks above) but gating
    // it directly keeps every issued credential individually bounded by the
    // intermediate's authority.
    if !g.scope.is_attenuation_of(intermediate_max_scope) {
        return Err(PairingError::ScopeExceedsIntermediate);
    }
    Ok(())
}

/// Verify the delegation chain for a depth-1 grant. Every failure collapses to
/// [`PairingError::InvalidDelegation`] — the delegated path is authorized or it
/// is not.
fn verify_delegation(
    del: &Delegation,
    intermediate_key: &PublicKey,
    robot: &RobotId,
    now_ns: u64,
    delegator_account: &AccountId,
    delegator_key: &PublicKey,
) -> Result<(), PairingError> {
    let invalid = |_: PairingError| PairingError::InvalidDelegation;

    // The delegator's depth-0 grant must be intermediate-issued, valid, for this
    // robot, name the delegator as its subject, and carry CAP_DELEGATE.
    let p = &del.parent.grant;
    if p.delegation_depth != 0
        || p.robot != *robot
        || p.subject != *delegator_account
        || p.issuer_key != *intermediate_key
        || (p.scope.caps & Scope::CAP_DELEGATE) == 0
    {
        return Err(PairingError::InvalidDelegation);
    }
    check_validity(&p.validity, now_ns, CertKind::Grant).map_err(invalid)?;
    check_issued_at(p.issued_at_ns, now_ns, CertKind::Grant).map_err(invalid)?;
    verify_sig(
        &p.issuer_key,
        &p.signing_payload(),
        &del.parent.signature,
        CertKind::Grant,
    )
    .map_err(invalid)?;

    // The delegator's device cert must bind the depth-1 grant's issuer_key to the
    // delegator account, be intermediate-issued, and be valid.
    let dc = &del.delegator_cert.cert;
    if dc.issuer_key != *intermediate_key
        || dc.account != *delegator_account
        || dc.device_key != *delegator_key
    {
        return Err(PairingError::InvalidDelegation);
    }
    check_validity(&dc.validity, now_ns, CertKind::Device).map_err(invalid)?;
    check_issued_at(dc.issued_at_ns, now_ns, CertKind::Device).map_err(invalid)?;
    verify_sig(
        &dc.issuer_key,
        &dc.signing_payload(),
        &del.delegator_cert.signature,
        CertKind::Device,
    )
    .map_err(invalid)?;

    Ok(())
}
