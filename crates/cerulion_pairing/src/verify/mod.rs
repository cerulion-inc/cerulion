// SPDX-License-Identifier: MIT OR Apache-2.0
//! Robot-side verification: the offline certificate-chain verifier, the durable
//! access list, and the tamper-evident, anti-rollback trust store.
//!
//! The entry point is [`TrustStore`], which owns the root set, the ownership
//! state, the access list, the current revocation epoch, and the anti-rollback
//! high-water mark. Chain verification (the internal `chain` submodule) is pure
//! (crypto + structural checks); everything that touches persisted state
//! (rollback floor, revocation, ownership lifecycle) lives on the store.

mod access;
mod chain;
mod store;

pub use access::{AccessRow, EpochOutcome, OwnershipState, PairingSource};
pub use store::TrustStore;

use crate::format::{
    AccountId, Delegation, PrincipalKind, PublicKey, Scope, SignedAccessGrant, SignedDeviceCert,
    SignedEpoch, SignedGrant, SignedIntermediateCert,
};
use crate::PairingError;
use serde::{Deserialize, Serialize};

/// Everything a client presents to establish a **new** pairing over the strong
/// path: the M-of-N-signed intermediate, the device cert (whose key must equal
/// the authenticated peer), the grant, and — for a depth-1 grant — the
/// delegation chain.
#[derive(Clone, Debug)]
pub struct PairingPresentation {
    /// The root-signed intermediate certificate.
    pub intermediate: SignedIntermediateCert,
    /// The device certificate (its key must equal the authenticated peer key).
    pub device_cert: SignedDeviceCert,
    /// The access grant for this robot.
    pub grant: SignedGrant,
    /// Present only for a depth-1 (delegated) grant.
    pub delegation: Option<Delegation>,
}

/// The owner account's device certificate chain for a new device binding.
/// This proof carries no robot grant: the store must already belong to the
/// certified account. It cannot create or replace an owner access row.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OwnerCertificatePresentationWire {
    /// Envelope version, first on the wire.
    pub version: u16,
    /// Root-signed intermediate certificate.
    pub intermediate: SignedIntermediateCert,
    /// Device certificate bound to the authenticated transport key.
    pub device_cert: SignedDeviceCert,
}

impl OwnerCertificatePresentationWire {
    /// Version of the first owner-certificate envelope.
    pub const FORMAT_VERSION: u16 = 1;

    /// Construct the current envelope from its signed certificates.
    pub fn new(intermediate: SignedIntermediateCert, device_cert: SignedDeviceCert) -> Self {
        Self {
            version: Self::FORMAT_VERSION,
            intermediate,
            device_cert,
        }
    }

    /// Encode the shared desk/robot postcard representation.
    pub fn to_postcard(&self) -> Result<Vec<u8>, PairingError> {
        self.validate_version()?;
        postcard::to_stdvec(self).map_err(crate::error::ser_err)
    }

    /// Decode exactly one supported envelope, refusing appended fields.
    pub fn from_postcard(bytes: &[u8]) -> Result<Self, PairingError> {
        let (version, _): (u16, _) =
            postcard::take_from_bytes(bytes).map_err(crate::error::ser_err)?;
        if version != Self::FORMAT_VERSION {
            return Err(Self::unsupported_version(version));
        }
        let (wire, rest): (Self, _) =
            postcard::take_from_bytes(bytes).map_err(crate::error::ser_err)?;
        if !rest.is_empty() {
            return Err(PairingError::Serialization(
                "owner-certificate artifact has trailing bytes".into(),
            ));
        }
        Ok(wire)
    }

    pub(crate) fn validate_version(&self) -> Result<(), PairingError> {
        if self.version != Self::FORMAT_VERSION {
            return Err(Self::unsupported_version(self.version));
        }
        Ok(())
    }

    fn unsupported_version(version: u16) -> PairingError {
        PairingError::Serialization(format!(
            "owner-certificate artifact version {version} is unsupported"
        ))
    }
}

/// Everything the DESK carries + presents to establish access from a
/// **desk-carried, owner-signed access grant**. The robot's owner signs an
/// [`SignedAccessGrant`] for the subject account; the desk carries it (bundled with
/// the owner's device cert + the intermediate) and, at dial time, combines it with
/// its OWN device cert (the subject cert, which authenticates the dialing peer). The
/// robot verifies the whole thing OFFLINE against its own claimed owner — no cloud
/// contact, no synced ACL. See [`TrustStore::establish_by_owner_grant`].
#[derive(Clone, Debug)]
pub struct OwnerGrantPresentation {
    /// The root-signed intermediate certificate (the shared trust anchor above both
    /// the owner cert and the subject cert).
    pub intermediate: SignedIntermediateCert,
    /// The OWNER's device certificate, binding the owner's signing device key to the
    /// owner account. Verified via the issuer chain only (its key is NOT the dialing
    /// peer); its account must equal the robot's claimed owner.
    pub owner_cert: SignedDeviceCert,
    /// The SUBJECT's (dialing desk's) device certificate, binding the subject device
    /// key to the subject account. Its `device_key` MUST equal the authenticated peer
    /// key (exactly like the strong-path `device_cert`).
    pub subject_cert: SignedDeviceCert,
    /// The owner-signed grant: "this owner grants this subject access to this robot
    /// at this scope". Signed by `owner_cert`'s device key.
    pub access_grant: SignedAccessGrant,
}

/// The **serde-transportable** form of an [`OwnerGrantPresentation`] —
/// the ONE canonical wire shape shared by BOTH the desk carriage
/// (`cerulion_wireclient`, which builds + encodes it) and the robot's `present-grant`
/// ops verb (`cerulion_remoted`, which decodes it), so the two can NEVER drift in
/// field order (both go through this struct's postcard). It lives in `cerulion_pairing`
/// — the crate that owns the byte-oriented serde for these cert/grant types and the
/// common dependency of both sides — so there is no `wireclient ↔ remoted` package edge.
///
/// It rides a **postcard** blob (hex-encoded for the JSON ops arg), NOT raw JSON: the
/// cert/grant/`Signature` types are byte-oriented serde (an ed25519 `Signature`
/// (de)serializes as raw bytes) and do NOT round-trip through serde_json.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OwnerGrantPresentationWire {
    /// The root-signed intermediate certificate.
    pub intermediate: SignedIntermediateCert,
    /// The OWNER's device cert (its account must equal the robot's claimed owner).
    pub owner_cert: SignedDeviceCert,
    /// The SUBJECT's device cert (its key must equal the authenticated peer key).
    pub subject_cert: SignedDeviceCert,
    /// The owner-signed access grant for this robot + subject.
    pub access_grant: SignedAccessGrant,
}

impl OwnerGrantPresentationWire {
    /// Reconstruct the [`OwnerGrantPresentation`].
    pub fn into_presentation(self) -> OwnerGrantPresentation {
        OwnerGrantPresentation {
            intermediate: self.intermediate,
            owner_cert: self.owner_cert,
            subject_cert: self.subject_cert,
            access_grant: self.access_grant,
        }
    }

    /// Encode as a deterministic postcard blob (the raw bytes the ops arg hex-wraps).
    pub fn to_postcard(&self) -> Result<Vec<u8>, PairingError> {
        postcard::to_stdvec(self).map_err(crate::error::ser_err)
    }

    /// Decode a postcard blob into a live [`OwnerGrantPresentation`], erroring loudly
    /// (never fail-open) on a malformed blob.
    pub fn from_postcard(bytes: &[u8]) -> Result<OwnerGrantPresentation, PairingError> {
        let wire: OwnerGrantPresentationWire =
            postcard::from_bytes(bytes).map_err(crate::error::ser_err)?;
        Ok(wire.into_presentation())
    }
}

/// The **serde-transportable** access-list-epoch SYNC artifact — the ONE
/// canonical wire shape shared by BOTH the desk carriage (`cerulion_wireclient`,
/// which reads it from the desk's epoch cache and encodes it) and the robot's
/// `sync_epoch` `cerulion/wire/1` control verb (`cerulion_remoted`, which decodes it
/// and feeds [`TrustStore::apply_epoch`]). Homed here — in the crate that owns the
/// byte-oriented serde for these cert/epoch types and is the common dependency of
/// both sides — so the two can NEVER drift in field order and there is no
/// `wireclient ↔ remoted` package edge. This is the EXACT pattern
/// [`OwnerGrantPresentationWire`] established for the desk-carried grants.
///
/// It carries BOTH halves [`TrustStore::apply_epoch`] needs: the CA-signed epoch and
/// the root-signed `intermediate` that issued it (the robot re-verifies the
/// intermediate against its OWN root set, so a forged intermediate is refused — the
/// desk is a courier, never a trust anchor).
///
/// Like the A5 presentation it rides a **postcard** blob, NOT raw JSON: the
/// cert/epoch/`Signature` types are byte-oriented serde (an ed25519 `Signature`
/// (de)serializes as raw bytes) and do NOT round-trip through serde_json. The blob is
/// hex-encoded for the JSON control-frame field (matching A5's `grant_postcard`) and
/// base64url-encoded at rest in the desk's epoch cache (matching the A4 `device.cert`
/// / A5 `<robot>.grant` on-disk convention).
/// The version stamped into every [`EpochSyncWire`] envelope.
///
/// postcard is NOT self-describing: it decodes positionally and IGNORES trailing
/// bytes, so a future artifact that appends a field would otherwise decode
/// SILENTLY (and wrongly) on an older peer. The leading discriminant + the strict
/// trailing-byte check in [`EpochSyncWire::from_postcard`] make that a LOUD
/// refusal instead. Bump this whenever the envelope's shape changes; an older
/// peer then refuses the artifact by name rather than mis-reading it.
///
/// # Scope of the "refuses by name" guarantee
///
/// The discriminant is a plain leading `u16`, NOT a magic — so it separates version `N`
/// from version `M`, and it does that by NAME. It cannot separate a versioned envelope
/// from a hypothetical UNVERSIONED one whose first bytes happen to look like a version
/// (that artifact never existed: version 1 is the first shape the epoch sync ever produced).
/// An unversioned blob is still refused LOUDLY — it fails the structural decode or the
/// trailing-byte check rather than being mis-read as a valid artifact — just not with
/// the version-named message. Pinned by
/// `cerulion_pairing/tests/format_vectors_test.rs::an_unversioned_epoch_envelope_is_refused_loudly`.
pub const EPOCH_SYNC_WIRE_VERSION: u16 = 1;

/// The substring the robot's control-plane answer carries when it could
/// not DECODE the request at all — the `handle_request` malformed-request arm.
///
/// It is the discriminator a desk uses to tell "this robot does not know the
/// `sync_epoch` verb (a build predating it)" apart from "this robot rejected the
/// epoch": an older robot fails `serde_json::from_slice::<WireRequest>` on the
/// unknown `verb` tag and answers with THIS marker, never with
/// [`NO_EPOCH_SINK_NEEDLE`] (which only a sink-aware build can emit).
///
/// It is deliberately OUR OWN string, not serde's error prose: matching serde_json's
/// `unknown variant \`sync_epoch\`, expected one of …` phrasing would couple the desk
/// to a third-party crate's error formatting. This marker has been the robot's
/// malformed-request prefix since the wire plane shipped, so it ALSO
/// classifies robots that predate the epoch sync and can never be changed — which is why no
/// serde-text fallback is needed. Pinned against the REAL serde error text by
/// `cerulion_connectd/tests/protocol_parity_test.rs`.
pub const UNDECODABLE_REQUEST_NEEDLE: &str = "malformed wire control request";

/// The substring [`EpochSyncWire::from_postcard`] puts at the FRONT of its
/// version-mismatch refusal — the discriminator a desk uses to tell an ENVELOPE-VERSION
/// skew apart from a rejected epoch.
///
/// A robot that refuses the artifact because its envelope version is not the one this
/// build understands is NOT saying "this epoch is forged / for another robot / your
/// clock is skewed" (the [`crate::verify::TrustStore::apply_epoch`] refusal class). It
/// is saying "one of us is the wrong build" — an UPGRADE-shaped outcome. Classifying it
/// as a rejected epoch sends an operator hunting a forgery when the real fix is a
/// version bump on one side, which is the same misdirection
/// [`UNDECODABLE_REQUEST_NEEDLE`] exists to prevent one generation earlier.
///
/// Shared `pub const` for the same reason as its two siblings: the side that EMITS the
/// text and the side that CLASSIFIES it must never drift.
pub const EPOCH_VERSION_UNSUPPORTED_NEEDLE: &str = "epoch-sync artifact version";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EpochSyncWire {
    /// The envelope version ([`EPOCH_SYNC_WIRE_VERSION`]) — FIRST on the wire, so a
    /// peer reads it before anything else and refuses an artifact it cannot
    /// understand instead of mis-decoding one.
    pub version: u16,
    /// The root-signed intermediate certificate that issued `signed_epoch`. The robot
    /// re-verifies it against its own root set before trusting the epoch.
    pub intermediate: SignedIntermediateCert,
    /// The CA-signed access-list epoch to apply. Monotonic at the robot: a stale
    /// (not-newer) epoch is a harmless no-op, never a rollback.
    pub signed_epoch: SignedEpoch,
}

/// The substring a `sync_epoch` refusal carries when the robot accepts NO
/// epoch sync at all (no sink installed — including a build predating the verb), as
/// opposed to REJECTING a specific epoch.
///
/// A desk distinguishes the two because they call for DIFFERENT operator actions —
/// "upgrade/configure the robot" vs "investigate this epoch" — so the marker is a
/// shared `pub const` here (the common dependency of the robot that emits it and the
/// desk that classifies it, exactly like [`EpochSyncWire`]) rather than a string
/// literal duplicated on both sides that could silently drift apart.
pub const NO_EPOCH_SINK_NEEDLE: &str = "does not accept access-list epoch sync";

/// The desk's on-disk epoch-cache file name for `robot` (inside the epochs
/// directory): `<robot>.epoch`, mirroring A5's `<robot>.grant`.
///
/// Homed HERE — the crate both halves of the cache already depend on — because the
/// WRITER (`cerulion_cli_engine::account_cmd`, which must stay iroh-free and so cannot
/// see `cerulion_wireclient`) and the READERS (`cerulion_wireclient::config` /
/// `cerulion_netd`'s WAN plane) must agree BYTE-FOR-BYTE on the name. A mismatch is
/// invisible at both ends: the writer reports "cached", the reader reports "nothing
/// cached", and revocations silently never travel. One function, one convention.
/// (`cerulion_wireclient::config::epoch_cache_file_name` re-exports this verbatim.)
///
/// Path SEPARATORS (`/`, `\`, `:`, NUL) in `robot` are replaced with `_`, so a
/// `..`-bearing name is defanged by losing its separators (`../../etc/passwd` becomes
/// `.._.._etc_passwd.epoch`) and can never traverse out of the epoch directory; a name
/// that is ONLY dots is additionally rewritten so it cannot become a bare `..epoch`.
/// (Defense in depth: the name comes from the desk's OWN records, and the bytes still
/// have to verify as a CA-signed epoch robot-side — but a config-driven path join
/// should not be traversable.)
pub fn epoch_cache_file_name(robot: &str) -> String {
    let safe: String = robot
        .trim()
        .chars()
        .map(|c| match c {
            '/' | '\\' | ':' | '\0' => '_',
            c => c,
        })
        .collect();
    // A name that is only dots (`.`, `..`) would otherwise produce `..epoch` /
    // `...epoch` — harmless, but pin the intent explicitly rather than relying on the
    // extension. An EMPTY (or whitespace-only) name must not fall into the same arm
    // vacuously: `"".chars().all(..)` is true, and the replace would yield the hidden
    // dotfile `.epoch` that every empty name then collides on. Give it a sentinel of
    // its own instead.
    let safe = if safe.is_empty() {
        "_unnamed_".to_string()
    } else if safe.chars().all(|c| c == '.') {
        safe.replace('.', "_")
    } else {
        safe
    };
    format!("{safe}.epoch")
}

/// The desk's on-disk epoch-cache PATH for `robot` inside `epochs_dir` —
/// [`epoch_cache_file_name`] joined onto the directory.
///
/// THE one keying rule. Homed here beside the name convention (and NOT duplicated at
/// each call site) because three independent places resolve it: the WRITER
/// (`cerulion_cli_engine::account_cmd`, iroh-free), the `cerulion connect` session
/// (`cerulion_wireclient`), and `cerulion-netd`'s WAN plane. A hand-rolled join at any
/// one of them is invisible at BOTH ends — the writer reports "cached", the reader
/// reports "nothing cached", and revocations silently never travel.
pub fn epoch_cache_path_in(epochs_dir: &std::path::Path, robot: &str) -> std::path::PathBuf {
    epochs_dir.join(epoch_cache_file_name(robot))
}

/// Env var naming the desk's revocation-epoch cache DIRECTORY — an explicit
/// override honored by EVERY desk path.
///
/// Deliberately NOT `CERULION_NETD_*`-prefixed: the epoch cache is a DESK-WIDE artifact
/// location (like the desk key, `device.cert`, and `grants/`), resolved by `cerulion
/// connect`, `cerulion-netd`, and the account-page cache WRITER alike. A netd-scoped
/// name would be a lie the moment another path honored it — and a name only ONE path
/// honored is exactly the silent divergence [`resolve_epoch_dir`] exists to prevent:
/// the writer relocates its cache, the other reader keeps looking at the old directory,
/// and revocations stop travelling with NO symptom at either end.
///
/// Unset / blank ⇒ the `epochs/` directory NEXT TO the desk key file
/// ([`resolve_epoch_dir`]).
pub const EPOCH_DIR_ENV: &str = "CERULION_EPOCH_DIR";

/// THE one resolver for WHERE this desk's `<robot>.epoch` artifacts live.
///
/// - an explicit `explicit` ([`EPOCH_DIR_ENV`]) wins (blank/whitespace-only is ignored);
/// - otherwise the SIBLING `epochs/` directory next to the desk `key_file` (both live
///   under `~/.cerulion`, matching how `device.cert` and `grants/` are found);
/// - `None` when neither is available (an ephemeral desk key with no explicit env —
///   there is no cache, so nothing is pushed and the dial is unchanged).
///
/// Homed here with [`epoch_cache_file_name`] / [`epoch_cache_path_in`] for the same
/// reason: this is the one crate BOTH desk push paths (`cerulion connect`'s session
/// driver and `cerulion-netd`'s WAN plane) and the cache WRITER
/// (`cerulion_cli_engine::account_cmd`, which must stay iroh-free and so cannot see
/// `cerulion_wireclient`) all depend on. Two implementations would mean an epoch cached
/// by one path is INVISIBLE to another under relocation — a silent non-delivery with no
/// symptom at either end. Pure — oracle-tested.
pub fn resolve_epoch_dir(
    explicit: Option<&str>,
    key_file: Option<&std::path::Path>,
) -> Option<std::path::PathBuf> {
    if let Some(p) = explicit.map(str::trim).filter(|s| !s.is_empty()) {
        return Some(std::path::PathBuf::from(p));
    }
    key_file.map(|kf| match kf.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join("epochs"),
        // A bare `desk.key` (no parent component) ⇒ `epochs/` in the same
        // (current) directory.
        _ => std::path::PathBuf::from("epochs"),
    })
}

impl EpochSyncWire {
    /// Build an artifact stamped with the CURRENT [`EPOCH_SYNC_WIRE_VERSION`] — the
    /// one constructor every producer (the account-page sync, a test fixture) should
    /// use, so the version can never be forgotten or hand-set to a wrong value.
    pub fn new(intermediate: SignedIntermediateCert, signed_epoch: SignedEpoch) -> Self {
        Self {
            version: EPOCH_SYNC_WIRE_VERSION,
            intermediate,
            signed_epoch,
        }
    }

    /// Encode as a deterministic postcard blob (the raw bytes the control frame
    /// hex-wraps and the desk cache base64url-wraps).
    pub fn to_postcard(&self) -> Result<Vec<u8>, PairingError> {
        postcard::to_stdvec(self).map_err(crate::error::ser_err)
    }

    /// Decode a postcard blob, erroring loudly (never fail-open) on a malformed blob.
    ///
    /// STRICT on both axes postcard is lenient about:
    ///
    /// - **Version**: an envelope stamped with anything other than
    ///   [`EPOCH_SYNC_WIRE_VERSION`] is refused BY NAME (never re-interpreted under
    ///   this build's field layout).
    /// - **Trailing bytes**: postcard's `from_bytes` decodes positionally and IGNORES
    ///   anything after the last field, so a future artifact carrying an appended
    ///   field would decode "successfully" here while silently dropping it. Any
    ///   remainder is a loud error.
    pub fn from_postcard(bytes: &[u8]) -> Result<Self, PairingError> {
        // Peek the version varint BEFORE decoding the rest: `version` is the first
        // field, so this read is layout-stable across versions, and a v2 whose LATER
        // fields changed shape gets the loud version refusal below instead of a
        // cryptic postcard decode error from those fields.
        let (peeked_version, _): (u16, &[u8]) =
            postcard::take_from_bytes(bytes).map_err(crate::error::ser_err)?;
        if peeked_version != EPOCH_SYNC_WIRE_VERSION {
            return Err(PairingError::Serialization(format!(
                "{EPOCH_VERSION_UNSUPPORTED_NEEDLE} {peeked_version} is not supported by this \
                 build (expected {EPOCH_SYNC_WIRE_VERSION}) — upgrade the peer that produced \
                 it, or re-sync the epoch",
            )));
        }
        let (wire, rest): (Self, &[u8]) =
            postcard::take_from_bytes(bytes).map_err(crate::error::ser_err)?;
        if !rest.is_empty() {
            return Err(PairingError::Serialization(format!(
                "epoch-sync artifact carries {} unexpected trailing byte(s) — refusing it rather \
                 than silently ignoring data this build does not understand",
                rest.len()
            )));
        }
        Ok(wire)
    }
}

/// The result of a successful chain verification: the account, its effective
/// (least-privilege) scope, principal kind, and the verified device key.
///
/// This is an **unforgeable capability token**: its fields are private, so a
/// value can only be minted inside this crate's verification path
/// ([`TrustStore::verify_new_pairing`]). External code cannot hand-construct one
/// and pass it to [`TrustStore::establish_pairing`] to persist unverified access
/// — the type system forbids it:
///
/// A token is minted only for a **claimed** robot and a subject that is **not
/// revoked** — neither by the current epoch nor by a sticky owner "revoke-now"
/// (an owner-revoked leaf is refused at [`TrustStore::verify_new_pairing`], not
/// silently minted). Holding one therefore reflects the subject's pairability at
/// mint time. Persistence is gated a second time: [`TrustStore::establish_pairing`]
/// independently refuses on an unclaimed robot, so a token minted while claimed
/// cannot be replayed after a `factory_reset` returned the robot to unclaimed.
///
/// ```compile_fail
/// use cerulion_pairing::verify::VerifiedPairing;
/// use cerulion_pairing::format::{AccountId, PublicKey, Scope, Role, PrincipalKind};
/// // ERROR: fields are private; a "verified" token cannot be forged.
/// let _forged = VerifiedPairing {
///     account: AccountId([0; 32]),
///     scope: Scope { role: Role::OWNER, caps: u64::MAX },
///     principal_kind: PrincipalKind::Human,
///     device_key: PublicKey([0; 32]),
/// };
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerifiedPairing {
    account: AccountId,
    scope: Scope,
    principal_kind: PrincipalKind,
    device_key: PublicKey,
}

impl VerifiedPairing {
    /// Mint a verified pairing. Crate-internal: only the verification path calls
    /// this, which is what makes the token unforgeable from outside the crate.
    pub(crate) fn new(
        account: AccountId,
        scope: Scope,
        principal_kind: PrincipalKind,
        device_key: PublicKey,
    ) -> Self {
        VerifiedPairing {
            account,
            scope,
            principal_kind,
            device_key,
        }
    }

    /// The account that owns the presented device key.
    pub fn account(&self) -> AccountId {
        self.account
    }
    /// Effective (least-privilege) scope.
    pub fn scope(&self) -> Scope {
        self.scope
    }
    /// Human vs. machine principal.
    pub fn principal_kind(&self) -> PrincipalKind {
        self.principal_kind
    }
    /// The verified device (transport) key.
    pub fn device_key(&self) -> PublicKey {
        self.device_key
    }
}
