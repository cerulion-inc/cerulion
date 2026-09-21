// SPDX-License-Identifier: MIT OR Apache-2.0
//! Shared, deterministic test fixtures. All keys are derived from fixed seeds
//! (Principle #13: deterministic fixed test keys, never fake/simulated data).

#![allow(dead_code)] // each test file uses a subset of these helpers.

use cerulion_pairing::format::*;
use ed25519_dalek::SigningKey;

/// A fixed "current time" well inside every fixture validity window (Unix ns).
pub const T_NOW: u64 = 1_000_000_000_000;
/// A fixed issuance time (before `T_NOW`).
pub const ISSUED: u64 = 500_000_000_000;
/// A per-unit random-looking chassis secret (a fixed test value here).
pub const CHASSIS: &[u8] = b"unit-42-random-chassis-secret-not-serial-derived";
/// A fixed trust-store MAC key (firmware would draw this from secure storage).
pub const MAC_KEY: &[u8] = b"trust-store-mac-key-32-bytes";

/// An ed25519 signing key from a single seed byte.
pub fn sk(seed: u8) -> SigningKey {
    SigningKey::from_bytes(&[seed; 32])
}

/// The public key of a signing key.
pub fn pk(k: &SigningKey) -> PublicKey {
    PublicKey(k.verifying_key().to_bytes())
}

/// An account id from a single byte.
pub fn acct(b: u8) -> AccountId {
    AccountId([b; 32])
}

/// A robot id from a single byte.
pub fn rob(b: u8) -> RobotId {
    RobotId([b; 32])
}

/// A wide validity window that contains [`T_NOW`].
pub fn wide_validity() -> Validity {
    Validity {
        not_before_ns: 0,
        not_after_ns: 100_000_000_000_000,
    }
}

/// An operator scope (teleop + observe).
pub fn op_scope() -> Scope {
    Scope {
        role: Role::OPERATOR,
        caps: Scope::CAP_TELEOP | Scope::CAP_OBSERVE,
    }
}

pub fn make_intermediate(int_pk: PublicKey, v: Validity, issued: u64) -> IntermediateCert {
    IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: int_pk,
        validity: v,
        issued_at_ns: issued,
        max_scope: Scope::OWNER_FULL,
    }
}

pub fn make_device_cert(
    device_pk: PublicKey,
    account: AccountId,
    issuer_pk: PublicKey,
    scope: Scope,
    v: Validity,
    issued: u64,
) -> DeviceCert {
    DeviceCert {
        version: FORMAT_VERSION,
        device_key: device_pk,
        account,
        principal_kind: PrincipalKind::Human,
        scope,
        validity: v,
        issued_at_ns: issued,
        issuer_key: issuer_pk,
    }
}

#[allow(clippy::too_many_arguments)]
pub fn make_grant(
    subject: AccountId,
    robot: RobotId,
    scope: Scope,
    depth: u8,
    issuer_acct: AccountId,
    issuer_pk: PublicKey,
    v: Validity,
    issued: u64,
) -> Grant {
    Grant {
        version: FORMAT_VERSION,
        subject,
        robot,
        scope,
        principal_kind: PrincipalKind::Human,
        delegation_depth: depth,
        validity: v,
        issued_at_ns: issued,
        issuer: issuer_acct,
        issuer_key: issuer_pk,
    }
}

pub fn make_epoch(
    robot: RobotId,
    epoch: u64,
    revoked: Vec<AccountId>,
    issuer_pk: PublicKey,
    issued: u64,
) -> AccessListEpoch {
    make_epoch_with_devices(robot, epoch, revoked, Vec::new(), issuer_pk, issued)
}

/// A6: an epoch carrying BOTH a revoked-account set and a revoked-device
/// set (the device-revocation fixtures use this; `make_epoch` is the account-only
/// convenience that passes an empty device set).
pub fn make_epoch_with_devices(
    robot: RobotId,
    epoch: u64,
    revoked_accounts: Vec<AccountId>,
    revoked_devices: Vec<PublicKey>,
    issuer_pk: PublicKey,
    issued: u64,
) -> AccessListEpoch {
    AccessListEpoch {
        version: FORMAT_VERSION,
        robot,
        epoch,
        revoked_accounts,
        revoked_devices,
        issued_at_ns: issued,
        issuer_key: issuer_pk,
    }
}
