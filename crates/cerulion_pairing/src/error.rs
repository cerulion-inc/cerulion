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
    /// A team X.509 certificate (a machine leaf or a team issuing CA).
    TeamCert,
    /// A team trust bundle.
    TeamTrustBundle,
    /// A team status list.
    TeamStatusList,
    /// A status freshness proof.
    StatusFreshness,
}

impl core::fmt::Display for CertKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let s = match self {
            CertKind::Intermediate => "intermediate cert",
            CertKind::Device => "device cert",
            CertKind::Grant => "grant",
            CertKind::Epoch => "epoch",
            CertKind::AccessGrant => "owner-signed access grant",
            CertKind::TeamCert => "team certificate",
            CertKind::TeamTrustBundle => "team trust bundle",
            CertKind::TeamStatusList => "team status list",
            CertKind::StatusFreshness => "status freshness proof",
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

    /// An owner device certificate cannot authorize the owner's stored scope.
    #[error("owner device certificate does not cover the existing owner access scope")]
    OwnerCertificateScopeInsufficient,
    /// An owner certificate cannot renew a finite, expired access row.
    #[error("the existing owner access row has expired")]
    OwnerAccessExpired,
    /// A code pairing named the robot's owner account; it would replace the
    /// owner's access row with the code-pairing scope.
    #[error("code pairing cannot bind the robot's owner account")]
    CodePairOwnerAccount,
    /// A cloud device certificate was presented while this robot's
    /// allow-from-cloud switch is OFF.
    #[error(
        "this robot does not allow access from the cloud (its allow-from-cloud switch is off)"
    )]
    CloudAccessDisabled,

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

    // ---- team trust formats ------------------------------------------------
    /// A team short name is not one DNS label of `[a-z0-9-]`, 1 to 63
    /// characters, not starting or ending with `-`.
    #[error("team short name must be one DNS label: 1 to 63 characters of a-z, 0-9 and '-', not starting or ending with '-'")]
    InvalidTeamShort,
    /// A common name is not `<role>.g<N>.<team>.cerulion.invalid` (or, for a
    /// CA, `ca.g<N>.<team>.cerulion.invalid`).
    #[error("common name is not a team name of the form <role>.g<N>.<team>.cerulion.invalid")]
    TeamNameMismatch,
    /// A team certificate is not well formed DER or leaves the profile.
    #[error("team certificate is malformed: {0}")]
    TeamCertMalformed(&'static str),
    /// A team certificate's key or signature algorithm is not ed25519.
    #[error("team certificate key or signature algorithm is not ed25519")]
    TeamCertNotEd25519,
    /// A certificate presented as a machine leaf is a CA certificate.
    #[error("machine leaf is a CA certificate")]
    TeamCertIsCa,
    /// A machine leaf's keyUsage is absent, lacks digitalSignature or has
    /// keyCertSign.
    #[error("machine leaf keyUsage must be digitalSignature without keyCertSign")]
    TeamCertKeyUsage,
    /// A machine leaf has no extKeyUsage extension.
    #[error("machine leaf has no extKeyUsage; it must list serverAuth and clientAuth")]
    TeamCertMissingEku,
    /// A machine leaf's extKeyUsage lacks serverAuth or clientAuth.
    #[error("machine leaf extKeyUsage must list both serverAuth and clientAuth")]
    TeamCertIncompleteEku,
    /// A machine leaf's validity does not match its role: a robot leaf must
    /// not expire, any other leaf must not outlive the role lifetime.
    #[error("{role} leaf validity does not match the role lifetime")]
    TeamCertLifetime { role: crate::team::TeamRole },
    /// A certificate time is outside what X.509 can express.
    #[error("certificate time is beyond 9999-12-31T23:59:59Z")]
    TeamCertTimeOutOfRange,
    /// A certificate's issuer name is not the team CA name it must be.
    #[error("certificate issuer name is not the team CA name of the same team and generation")]
    TeamCertIssuerName,
    /// A machine leaf's device URI is missing, repeated, or names a key other
    /// than the subject key.
    #[error("machine leaf must carry exactly one device URI naming its own key")]
    TeamCertDeviceUriMismatch,
    /// A team certificate carries a critical extension outside the profile.
    #[error("team certificate carries an unknown critical extension")]
    TeamCertUnknownCriticalExtension,
    /// A team issuing CA certificate leaves the CA profile.
    #[error("team CA certificate: {0}")]
    TeamCaProfile(&'static str),
    /// A leaf names a different team or generation than the CA checking it.
    #[error("leaf names a different team or generation than its issuing CA")]
    TeamCertIssuerMismatch,
    /// A team statement is structurally invalid.
    #[error("team statement is invalid: {0}")]
    TeamStatementInvalid(&'static str),
    /// A team statement is for a different team than expected.
    #[error("team statement is for a different team")]
    TeamMismatch,
    /// A freshness proof answers a different asker or nonce.
    #[error("status freshness proof answers a different device or nonce")]
    FreshnessMismatch,
    /// A team statement is older than the one already held.
    #[error("{what} rolled back: held {held}, offered {offered}")]
    TeamRollback {
        what: &'static str,
        held: u64,
        offered: u64,
    },
    // ---- machine identity on disk (`identity`) -----------------------------
    /// Neither `CERULION_HOME` nor a home directory resolves, so there is no
    /// place for the machine identity.
    #[error("no home directory to hold the machine identity; set CERULION_HOME to a directory")]
    NoConfigDir,
    /// A team directory name is not one DNS label (1 to 63 of `a-z`, `0-9`,
    /// `-`, not starting or ending with `-`).
    #[error(
        "invalid team name {0:?}: use 1 to 63 lowercase letters, digits or '-', not starting \
         or ending with '-'"
    )]
    InvalidTeamName(String),
    /// A machine key file exists but is not exactly 32 bytes. It is never
    /// regenerated in place, because robots may already trust the key it held.
    #[error(
        "the machine key {path} is {len} bytes, not 32; restore it from a backup, or delete it \
         to create a new key (robots paired with the old key must then be paired again)"
    )]
    MachineKeyMalformed {
        /// The file that holds the wrong number of bytes.
        path: std::path::PathBuf,
        /// Its length in bytes.
        len: usize,
    },
    /// The canonical machine key and the legacy `desk.key` both exist and hold
    /// different keys. Neither is chosen or modified.
    #[error(
        "{canonical} and {legacy} hold different machine keys; keep the one your robots are \
         paired with, move the other one out of the way, and run the command again"
    )]
    MachineKeyConflict {
        /// `<config>/identity/device.key`.
        canonical: std::path::PathBuf,
        /// `<config>/desk.key`.
        legacy: std::path::PathBuf,
    },
    /// A file operation on the machine identity failed.
    #[error("{op} {path}: {source}")]
    IdentityIo {
        /// What was being done (`reading`, `creating`, ...).
        op: &'static str,
        /// The path it was done to.
        path: std::path::PathBuf,
        /// The underlying error.
        #[source]
        source: std::io::Error,
    },
    /// The high-water file's MAC does not verify under this machine's key.
    #[error(
        "the identity high-water file failed its integrity check (edited, or copied from another \
         machine); delete it only if this machine's clock and team state are known good"
    )]
    HighWaterTampered,
    /// The high-water file has the wrong length or magic after its MAC verified,
    /// or before a MAC could be checked.
    #[error("the identity high-water file is malformed: {0}")]
    HighWaterCorrupt(&'static str),
}

pub(crate) fn ser_err(e: postcard::Error) -> PairingError {
    PairingError::Serialization(format!("{e}"))
}
