// SPDX-License-Identifier: MIT OR Apache-2.0
//! Unified crate error type.

use thiserror::Error;

/// Which link in the chain a validity / signature error refers to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CertKind {
    /// The online intermediate certificate (root-signed, M-of-N).
    Intermediate,
    /// The short-lived device certificate.
    Device,
    /// An issuer-signed access grant.
    Grant,
    /// A signed access-list epoch (CRL-lite).
    Epoch,
    /// An OWNER-signed access grant — signed by the robot owner's
    /// device key and verified offline against the robot's own owner chain.
    AccessGrant,
}

impl core::fmt::Display for CertKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let s = match self {
            CertKind::Intermediate => "intermediate cert",
            CertKind::Device => "device cert",
            CertKind::Grant => "grant",
            CertKind::Epoch => "epoch",
            CertKind::AccessGrant => "owner-signed access grant",
        };
        f.write_str(s)
    }
}

/// The single error type for the whole crate (per the workspace convention).
///
/// Not `PartialEq` (it carries `std::io::Error`); tests discriminate with
/// `matches!(err, PairingError::Variant { .. })`.
#[derive(Debug, Error)]
pub enum PairingError {
    // ---- format / signature ------------------------------------------------
    /// An ed25519 public key was structurally invalid (not a canonical point).
    #[error("invalid ed25519 public key")]
    BadKey,
    /// An ed25519 signature failed strict verification.
    #[error("invalid signature on {0}")]
    BadSignature(CertKind),
    /// A container failed to (de)serialize.
    #[error("serialization error: {0}")]
    Serialization(String),
    /// A canonical signing-bytes field length exceeded `u32::MAX` — encoding it
    /// would silently truncate the length prefix (a domain-confusion risk), so it
    /// is rejected instead.
    #[error("canonical signing-bytes field length exceeds u32::MAX")]
    CanonicalOverflow,

    // ---- chain verification ------------------------------------------------
    /// A certificate is past its `not_after`.
    #[error("{kind} expired: now={now_ns} not_after={not_after_ns}")]
    Expired {
        kind: CertKind,
        now_ns: u64,
        not_after_ns: u64,
    },
    /// A certificate is before its `not_before`.
    #[error("{kind} not yet valid: now={now_ns} not_before={not_before_ns}")]
    NotYetValid {
        kind: CertKind,
        now_ns: u64,
        not_before_ns: u64,
    },
    /// Too few distinct trusted roots signed the intermediate.
    #[error("root threshold not met: have {have} distinct trusted roots, need {need}")]
    RootThresholdNotMet { have: u8, need: u8 },
    /// The signing key of a lower cert did not match the issuer above it.
    #[error("issuer mismatch on {0}")]
    IssuerMismatch(CertKind),
    /// The device cert's `device_key` did not match the authenticated peer key.
    ///
    /// This is the app-layer binding: the offline chain proves K belongs to the
    /// account, and the transport proved the peer *is* K.
    #[error("device cert key does not match the authenticated peer key")]
    PeerKeyMismatch,
    /// The grant names a different robot than this one.
    #[error("grant is for a different robot")]
    WrongRobot,
    /// The grant subject did not match the device cert's account.
    #[error("grant subject does not match the device cert account")]
    SubjectMismatch,
    /// A grant's delegation depth exceeded [`crate::MAX_DELEGATION_DEPTH`].
    #[error("delegation depth {depth} exceeds the maximum of {max}")]
    DelegationDepthExceeded { depth: u8, max: u8 },
    /// A depth-1 grant was presented without a valid delegation chain.
    #[error("delegated grant is missing or has an invalid delegation chain")]
    InvalidDelegation,
    /// A delegated grant tried to confer more scope than the delegator holds
    /// (a delegation must attenuate, never widen, privilege).
    #[error("delegated grant escalates scope beyond the delegator's own grant")]
    ScopeEscalation,
    /// An intermediate-issued credential (device cert or grant, including a
    /// delegation's depth-0 parent grant) claimed a scope wider than the
    /// intermediate's own `max_scope`. The intermediate can never confer more
    /// authority than the roots delegated to it, so such a credential is rejected.
    #[error("credential scope exceeds the intermediate certificate's max_scope")]
    ScopeExceedsIntermediate,
    /// A credential (cert / grant / epoch) claims an issuance time in the future
    /// beyond the allowed clock skew — issuance can never be later than "now".
    #[error("{kind} issued in the future: issued_at={issued_at_ns} now={now_ns}")]
    IssuedInFuture {
        kind: CertKind,
        issued_at_ns: u64,
        now_ns: u64,
    },
    /// An account on the delegation chain (the delegator) is revoked, so its
    /// delegatees may not establish new pairings.
    #[error("an account on the delegation chain is revoked")]
    DelegatorRevoked,

    // ---- revocation / rollback --------------------------------------------
    /// The subject account is revoked by the current access-list epoch.
    #[error("account is revoked by the current access-list epoch (epoch {epoch})")]
    RevokedByEpoch { epoch: u64 },
    /// The presenting DEVICE key is revoked by the current access-list epoch
    /// (per-device revocation). Distinct from [`PairingError::RevokedByEpoch`] (which cuts the
    /// whole ACCOUNT) so the operator signal is accurate about WHICH revocation
    /// fired: a device revocation cuts ONE desk while the account's other devices
    /// keep their access, and a newer epoch dropping the device is the only
    /// re-admit — re-presenting an old signed grant cannot resurrect it (tombstone).
    #[error("device key is revoked by the current access-list epoch (epoch {epoch})")]
    RevokedDeviceByEpoch { epoch: u64 },
    /// The subject account is locally revoked by an owner "revoke-now" (the sticky
    /// `revoked` flag on its access-list row) rather than by an epoch push. Kept
    /// distinct from [`PairingError::RevokedByEpoch`] so the operator signal is
    /// accurate about WHICH revocation fired; an explicit owner re-admit is required
    /// to restore pairability.
    #[error("account is locally revoked by the owner (revoke-now); an owner re-admit is required before it can pair again")]
    RevokedByOwner,
    /// The presented time is behind the persisted anti-rollback high-water mark.
    #[error("clock rollback detected: now={now_ns} is behind high-water={high_water_ns}")]
    RollbackDetected { now_ns: u64, high_water_ns: u64 },

    // ---- trust store -------------------------------------------------------
    /// The trust store's HMAC tag did not verify — the file was tampered with.
    #[error("trust store MAC verification failed (tamper detected)")]
    TamperDetected,
    /// The trust store file is structurally corrupt (bad magic/version/length).
    #[error("trust store is corrupt: {0}")]
    StoreCorrupt(&'static str),
    /// A trust-store operation needed a backing path but none was set.
    #[error("trust store has no backing path")]
    NoStorePath,
    /// Underlying filesystem error.
    #[error("trust store I/O error: {0}")]
    Io(#[from] std::io::Error),
    /// The provided root set was invalid (bad threshold / duplicate / empty).
    #[error("invalid root set: {0}")]
    InvalidRootSet(&'static str),

    // ---- ownership lifecycle ----------------------------------------------
    /// A claim was attempted on an already-claimed robot.
    #[error("robot is already claimed")]
    AlreadyClaimed,
    /// An operation required an owner but the robot is unclaimed.
    #[error("robot is unclaimed")]
    Unclaimed,
    /// A new pairing (strong path or code path) was attempted on an UNCLAIMED
    /// robot. The FIRST pairing must prove physical possession via `claim()`; an
    /// unclaimed robot must carry zero access rows, so no pairing may mint a
    /// witness or persist a row before it is claimed.
    #[error("robot is unclaimed: claim it first (prove physical possession via claim()) before establishing any pairing")]
    RobotUnclaimed,
    /// The presented chassis secret did not match (physical possession failed).
    #[error("chassis secret mismatch (physical possession not proven)")]
    WrongChassisSecret,
    /// An owner-only operation was requested by a non-owner account.
    #[error("operation requires the owner account")]
    NotOwner,
    /// A login-gated owner claim ([`crate::verify::TrustStore::claim_by_owner_grant`])
    /// was presented a grant that does not confer the OWNER role. Ownership requires
    /// an `OWNER`-scoped grant (the account service issues `Scope::OWNER_FULL` for
    /// the installer at `POST /v1/robots`); a viewer/operator grant cannot claim.
    #[error("presented grant does not confer the owner role (an OWNER-scoped grant is required to claim ownership)")]
    NotOwnerGrant,

    // ---- owner-signed access grant ---------------------------------------
    /// An owner-signed [`crate::format::AccessGrant`] was presented whose owner
    /// device cert binds an account that is NOT this robot's claimed owner — only
    /// the robot's OWN owner may sign a grant the robot honors offline. Refused so a
    /// grant signed by some other account's owner-looking key can never admit a
    /// subject.
    #[error("owner-signed access grant was not signed by this robot's owner (the presented owner cert binds a different account than the robot's claimed owner)")]
    NotRobotOwner,
    /// An owner-signed access grant's `owner_device_key` did not match the device key
    /// bound by the presented owner device cert — the grant claims to be signed by a
    /// different key than the one the chain authenticates for the owner.
    #[error(
        "owner-signed access grant's signing key does not match the presented owner device cert"
    )]
    OwnerGrantKeyMismatch,
    /// An owner-signed access grant tried to confer more scope than the owner's own
    /// device cert holds (an owner can never grant more than it has). A distinct
    /// signal from [`PairingError::ScopeEscalation`] (the depth-1 cloud-delegation
    /// case) so the operator knows the OWNER-grant attenuation rule fired.
    #[error("owner-signed access grant confers more scope than the owner's own device cert holds")]
    OwnerGrantScopeExceedsOwner,
    /// An owner-signed access grant named the OWNER itself as its subject. The owner
    /// already has full access via its claim row (the owner needs no grant for its own
    /// access), and establishing an owner-signed guest row for the owner could only
    /// DOWNGRADE the owner's own row — so it is refused rather than silently applied.
    #[error("owner-signed access grant targets the owner itself; the owner already has full access and needs no grant")]
    OwnerGrantForOwner,

    // ---- PAKE --------------------------------------------------------------
    /// The pairing ceremony TTL elapsed.
    #[error("pairing ceremony expired")]
    PakeExpired,
    /// The pairing code was burned after too many failed attempts.
    #[error("pairing code burned: all {max} attempts exhausted")]
    AttemptsExhausted { max: u8 },
    /// Key confirmation failed — almost always a wrong pairing code.
    #[error("pairing key confirmation failed (wrong code?)")]
    ConfirmationFailed,
    /// The single-session ceremony was already consumed (succeeded).
    #[error("pairing ceremony already completed")]
    CeremonyConsumed,
    /// A code-pairing witness was presented for a different account than the one
    /// being established — a confirmation for account A cannot persist account B.
    #[error("code-pairing witness account does not match the account being established")]
    WitnessAccountMismatch,
    /// A code-pairing witness was minted by a CPace ceremony against a DIFFERENT
    /// robot's transport identity than this store's own. A `CpaceConfirmed` proven
    /// against robot R1 cannot be replayed to establish a row on robot R2 (a
    /// cross-robot witness replay), so the witness's `responder_key` must equal the
    /// robot's own transport key recorded at provision time.
    #[error("code-pairing witness robot does not match this robot's transport identity")]
    WitnessRobotMismatch,
    /// A configuration value was out of the permitted range.
    #[error("invalid configuration: {0}")]
    InvalidConfig(&'static str),
    /// The underlying CPace primitive rejected a message.
    #[error("CPace protocol error: {0}")]
    PakeProtocol(&'static str),
}

pub(crate) fn ser_err(e: postcard::Error) -> PairingError {
    PairingError::Serialization(format!("{e}"))
}
