// SPDX-License-Identifier: MIT OR Apache-2.0
//! The tamper-evident, anti-rollback trust store — the robot-side root of the
//! whole system.
//!
//! ## On-disk format (versioned)
//!
//! ```text
//! magic "CERPAIR\x01" (8) | format_version u16-LE (2) | body_len u32-LE (4)
//!   | body = postcard(TrustStoreInner) | HMAC-SHA256 tag (32)
//! ```
//!
//! The HMAC covers the header + body. On load the tag is verified first (in
//! constant time); a mismatch is [`PairingError::TamperDetected`]. Writes are
//! atomic (write a sibling `.tmp` then `rename`). The MAC key is supplied by the
//! caller (firmware secure storage) on every load/save — this crate holds no
//! key-management policy.
//!
//! ## Anti-rollback
//!
//! `high_water_ns` is the maximum validated issuance/observation time ever seen
//! (fed by NTP-when-reachable via the `now_ns` passed to verification). A
//! verification whose `now_ns` is *behind* the high-water is rejected
//! ([`PairingError::RollbackDetected`]) — a clock-rollback cannot resurrect an
//! expired cert. The floor is monotonic and persists across factory resets.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use super::access::{AccessRow, EpochOutcome, OwnershipState, PairingSource};
use super::chain;
use super::{OwnerGrantPresentation, PairingPresentation, VerifiedPairing};
use crate::crypto::{self, VerifyResult};
use crate::error::{ser_err, CertKind, PairingError};
use crate::format::canonical::checked_len_u32;
use crate::format::{
    AccountId, PublicKey, RobotId, Role, RootSet, Scope, SignedEpoch, SignedIntermediateCert,
    SignedPayload,
};
use crate::pake::CpaceConfirmed;

const STORE_MAGIC: [u8; 8] = *b"CERPAIR\x01";
// v2: `TrustStoreInner` gained
// `robot_transport_key` (the robot's own transport identity, used to reject a
// cross-robot code-pairing witness replay). The crate is unmerged, so there is no
// on-disk compatibility burden — a v1 file is refused as an unsupported version
// rather than migrated.
// v3: `TrustStoreInner` gained `revoked_devices` (the per-device
// revocation set, applied by `apply_epoch` and consulted at the accept gate). The
// crate is unmerged, so a v2 file is refused as an unsupported version rather than
// migrated (same policy the v1→v2 bump used).
const STORE_FORMAT_VERSION: u16 = 3;
const CHASSIS_DSI: &[u8] = b"cerulion-pairing:chassis-secret:v1";
const TAG_LEN: usize = 32;
const HEADER_LEN: usize = 8 + 2 + 4; // magic + version + body_len

/// The persisted, MAC-covered state of the trust store.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct TrustStoreInner {
    store_version: u16,
    robot: RobotId,
    /// The robot's OWN transport public key (its iroh `EndpointId`), recorded at
    /// provision time. A code-pairing witness ([`CpaceConfirmed`]) is minted by a
    /// ceremony run against a specific robot's transport identity (its
    /// `responder_key`); binding it to this field lets `establish_code_pairing`
    /// reject a witness proven against a DIFFERENT robot (cross-robot replay).
    robot_transport_key: PublicKey,
    root_set: RootSet,
    ownership: OwnershipState,
    /// SHA-256(domain || chassis_secret). The raw per-unit random chassis secret
    /// is never stored.
    chassis_secret_hash: [u8; 32],
    access_rows: Vec<AccessRow>,
    /// The current (monotonic) revocation epoch number.
    current_epoch: u64,
    /// The authoritative set of accounts revoked by the current epoch.
    revoked_accounts: Vec<AccountId>,
    /// The authoritative set of DEVICE (transport) keys revoked by the current epoch
    /// (the per-device set). A revoked device is denied at the accept gate
    /// ([`TrustStore::is_device_revoked`]) INDEPENDENTLY of its account row, so an
    /// owner can cut ONE compromised desk without revoking the whole account. Like
    /// `revoked_accounts` it is authoritative and monotonic — `apply_epoch` REPLACES
    /// it wholesale, so re-presenting an old signed grant cannot resurrect a revoked
    /// device (only a newer epoch dropping it re-admits it — the tombstone).
    revoked_devices: Vec<PublicKey>,
    /// Anti-rollback high-water mark (Unix ns).
    high_water_ns: u64,
}

/// The trust store: root set, ownership, access list, revocation epoch, and the
/// anti-rollback floor. See the module docs for the on-disk format.
///
/// `Debug` prints only the chassis-secret *hash* (never the raw secret) and no
/// key material.
///
/// `Clone` is a transient-snapshot affordance: a consumer that must
/// persist a mutation WITHOUT holding a lock across the blocking disk write clones
/// the store, releases the lock, and writes the clone (then discards it). The
/// clone is a full deep copy — never a second live source of truth.
#[derive(Clone, Debug)]
pub struct TrustStore {
    inner: TrustStoreInner,
    path: Option<PathBuf>,
}

impl TrustStore {
    // -- construction --------------------------------------------------------

    /// Provision a fresh, **unclaimed** store at the factory. `robot_transport_key`
    /// is the robot's own transport public key (its iroh `EndpointId`) — the
    /// factory knows it, and recording it here lets code pairing reject a witness
    /// minted against a different robot. `chassis_secret` must be a per-unit
    /// high-entropy random value (never serial-derived); only its hash is stored.
    /// `now_ns` seeds the anti-rollback floor. The returned store has no backing
    /// path — attach one with [`TrustStore::with_path`] and call [`TrustStore::save`].
    pub fn provision(
        robot: RobotId,
        robot_transport_key: PublicKey,
        root_set: RootSet,
        chassis_secret: &[u8],
        now_ns: u64,
    ) -> Result<Self, PairingError> {
        root_set.validate()?;
        Ok(TrustStore {
            inner: TrustStoreInner {
                store_version: STORE_FORMAT_VERSION,
                robot,
                robot_transport_key,
                root_set,
                ownership: OwnershipState::Unclaimed,
                chassis_secret_hash: crypto::sha256_domain(CHASSIS_DSI, chassis_secret),
                access_rows: Vec::new(),
                current_epoch: 0,
                revoked_accounts: Vec::new(),
                revoked_devices: Vec::new(),
                high_water_ns: now_ns,
            },
            path: None,
        })
    }

    /// Attach a backing file path (does not write).
    pub fn with_path(mut self, path: impl Into<PathBuf>) -> Self {
        self.path = Some(path.into());
        self
    }

    /// Load and verify a store from disk. Fails with [`PairingError::TamperDetected`]
    /// if the MAC does not verify.
    pub fn load(path: impl AsRef<Path>, mac_key: &[u8]) -> Result<Self, PairingError> {
        let bytes = std::fs::read(path.as_ref())?;
        let inner = Self::decode(&bytes, mac_key)?;
        Ok(TrustStore {
            inner,
            path: Some(path.as_ref().to_path_buf()),
        })
    }

    /// Atomically write the store to its backing path (write `.tmp`, then
    /// `rename`), authenticated with `mac_key`.
    pub fn save(&self, mac_key: &[u8]) -> Result<(), PairingError> {
        let path = self.path.as_ref().ok_or(PairingError::NoStorePath)?;
        let bytes = self.encode(mac_key)?;
        let tmp = tmp_path(path);
        std::fs::write(&tmp, &bytes)?;
        std::fs::rename(&tmp, path)?;
        Ok(())
    }

    fn encode(&self, mac_key: &[u8]) -> Result<Vec<u8>, PairingError> {
        let body = postcard::to_stdvec(&self.inner).map_err(ser_err)?;
        let mut framed = Vec::with_capacity(HEADER_LEN + body.len() + TAG_LEN);
        framed.extend_from_slice(&STORE_MAGIC);
        framed.extend_from_slice(&STORE_FORMAT_VERSION.to_le_bytes());
        // Use the crate's checked length rule (never a silent `as u32` truncation
        // >4 GiB — a truncated body_len would desync the on-disk framing). Mirrors
        // `format::canonical::checked_len_u32` / `OVERFLOW_MSG`.
        framed.extend_from_slice(&checked_len_u32(body.len())?.to_le_bytes());
        framed.extend_from_slice(&body);
        let tag = crypto::hmac_sha256(mac_key, &framed);
        framed.extend_from_slice(&tag);
        Ok(framed)
    }

    fn decode(bytes: &[u8], mac_key: &[u8]) -> Result<TrustStoreInner, PairingError> {
        if bytes.len() < HEADER_LEN + TAG_LEN {
            return Err(PairingError::StoreCorrupt("file shorter than header + tag"));
        }
        let (framed, tag) = bytes.split_at(bytes.len() - TAG_LEN);
        // Verify authenticity BEFORE structurally trusting any field.
        let expected = crypto::hmac_sha256(mac_key, framed);
        if !crypto::ct_eq(&expected, tag) {
            return Err(PairingError::TamperDetected);
        }
        if framed[..8] != STORE_MAGIC {
            return Err(PairingError::StoreCorrupt("bad magic"));
        }
        let ver = u16::from_le_bytes([framed[8], framed[9]]);
        if ver != STORE_FORMAT_VERSION {
            return Err(PairingError::StoreCorrupt("unsupported format version"));
        }
        let body_len =
            u32::from_le_bytes([framed[10], framed[11], framed[12], framed[13]]) as usize;
        let body = &framed[HEADER_LEN..];
        if body.len() != body_len {
            return Err(PairingError::StoreCorrupt("body length mismatch"));
        }
        postcard::from_bytes(body).map_err(ser_err)
    }

    // -- accessors -----------------------------------------------------------

    /// The robot this store belongs to.
    pub fn robot_id(&self) -> RobotId {
        self.inner.robot
    }
    /// The robot's own transport public key (recorded at provision time). A
    /// code-pairing witness must be proven against this key.
    pub fn robot_transport_key(&self) -> PublicKey {
        self.inner.robot_transport_key
    }
    /// The root set (trust anchor).
    pub fn root_set(&self) -> &RootSet {
        &self.inner.root_set
    }
    /// The ownership state.
    pub fn ownership(&self) -> OwnershipState {
        self.inner.ownership
    }
    /// The owner account, if claimed.
    pub fn owner(&self) -> Option<AccountId> {
        self.inner.ownership.owner()
    }
    /// Whether the robot has been claimed.
    pub fn is_claimed(&self) -> bool {
        matches!(self.inner.ownership, OwnershipState::Claimed(_))
    }
    /// The current revocation epoch number.
    pub fn current_epoch(&self) -> u64 {
        self.inner.current_epoch
    }
    /// The anti-rollback high-water mark (Unix ns).
    pub fn high_water_ns(&self) -> u64 {
        self.inner.high_water_ns
    }
    /// The durable access-list rows.
    pub fn access_rows(&self) -> &[AccessRow] {
        &self.inner.access_rows
    }

    /// The DEVICE (transport) keys revoked by the current epoch.
    pub fn revoked_devices(&self) -> &[PublicKey] {
        &self.inner.revoked_devices
    }

    /// Whether a specific DEVICE key is revoked by the current epoch.
    /// The accept gate consults this INDEPENDENTLY of the device's account row, so a
    /// revoked desk is denied even while its account (and the account's OTHER
    /// devices) keep access. A revoked device cannot re-pair to escape the tombstone
    /// (the establishment paths refuse it — see `verify_chain` / `verify_owner_grant`).
    pub fn is_device_revoked(&self, device_key: &PublicKey) -> bool {
        self.inner.revoked_devices.contains(device_key)
    }

    /// Whether `account` currently has access at `now_ns`: it has a
    /// non-owner-revoked row that is not revoked by the current epoch AND is not past
    /// its expiry. Established pairings never expire (their `expires_at_ns` is
    /// `None`); a **desk-carried owner-signed grant** MAY carry a finite
    /// `expires_at_ns`, so an owner's time-boxed guest access DENIES here once
    /// `now_ns >= expires_at_ns` — offline, with no explicit revoke needed. The
    /// robot's own trusted clock supplies `now_ns` (the same anti-rollback-protected
    /// time every credential path uses); the accept gate reads it here, so an expired
    /// grant refuses at connection accept.
    pub fn is_allowed(&self, account: &AccountId, now_ns: u64) -> Option<&AccessRow> {
        if self.inner.revoked_accounts.contains(account) {
            return None;
        }
        self.inner.access_rows.iter().find(|r| {
            &r.account == account
                && !r.revoked
                // `Some(exp)` is a finite bound: allowed only strictly before it
                // (mirrors the `[not_before, not_after)` half-open validity window,
                // so `now_ns >= not_after_ns` denies). `None` never expires.
                && r.expires_at_ns.is_none_or(|exp| now_ns < exp)
        })
    }

    // -- ownership lifecycle -------------------------------------------------

    /// Claim an unclaimed robot by proving physical possession (the chassis
    /// secret). Writes the owner row and returns an error if already claimed or
    /// if the secret is wrong.
    ///
    /// **This is now the factory-reset / recovery claim path.** The
    /// primary owner claim is the login-gated [`TrustStore::claim_by_owner_grant`]
    /// (the installer's account, bound at install via an owner grant). Chassis
    /// claim survives unchanged as the recovery route (e.g. after a
    /// [`TrustStore::factory_reset`], or when no account is reachable).
    pub fn claim(
        &mut self,
        account: AccountId,
        presented_chassis_secret: &[u8],
        principal_kind: crate::format::PrincipalKind,
        now_ns: u64,
    ) -> Result<(), PairingError> {
        if self.is_claimed() {
            return Err(PairingError::AlreadyClaimed);
        }
        self.verify_chassis(presented_chassis_secret)?;
        // Defense-in-depth: an unclaimed robot must carry ZERO access rows (the
        // pairing seams — `verify_new_pairing`, `establish_pairing`,
        // `establish_code_pairing` — all refuse on an unclaimed store, and both
        // `provision` and `factory_reset` leave the list empty, so this branch is
        // unreachable via the current public API). If a future code path ever left
        // a row on an unclaimed store, those rows are illegitimate by construction
        // (no owner authorized them and nobody could see or revoke them), so the
        // legitimate owner's claim wipes them rather than inheriting a hidden
        // backdoor. Loud so the invariant violation never passes silently.
        if !self.inner.access_rows.is_empty() {
            tracing::warn!(
                rows = self.inner.access_rows.len(),
                "claim() found access rows on an unclaimed robot; clearing them as \
                 illegitimate (no owner authorized them) before writing the owner row"
            );
            self.inner.access_rows.clear();
        }
        self.inner.ownership = OwnershipState::Claimed(account);
        // The owner row supersedes any prior row for that account.
        self.upsert_active_row(AccessRow {
            account,
            scope: Scope::OWNER_FULL,
            principal_kind,
            source: PairingSource::Claim,
            name: Some("owner".to_string()),
            added_at_ns: now_ns,
            // The owner row never expires (only owner-signed guest grants can).
            expires_at_ns: None,
            revoked: false,
        });
        self.inner.high_water_ns = self.inner.high_water_ns.max(now_ns);
        tracing::info!("robot claimed by owner account");
        Ok(())
    }

    /// Physically factory-reset the robot (proven by the chassis secret = physical
    /// possession): the resale-wipe.
    /// Returns the robot to the
    /// unclaimed state with a blank access list and epoch, but the anti-rollback
    /// floor SURVIVES.
    ///
    /// Invariants (a future refactor must not silently break these — each is
    /// pinned by a test, see `factory_reset_preserves_high_water_ns` and
    /// `stale_revoked_credential_is_still_rejected_after_factory_reset`):
    /// - `high_water_ns` is **deliberately PRESERVED** — the anti-rollback floor is
    ///   monotonic and survives resale, so a new owner cannot roll the clock back
    ///   to resurrect a credential that was already expired before the reset. Do
    ///   NOT zero it here.
    /// - `revoked_accounts` + `revoked_devices` + `current_epoch` are **deliberately
    ///   CLEARED** — a new owner starts from a blank slate; the previous owner's
    ///   cloud epoch does not bind the new owner. (The floor still defends the
    ///   credential-replay vector regardless of the cleared revocation sets.)
    pub fn factory_reset(&mut self, presented_chassis_secret: &[u8]) -> Result<(), PairingError> {
        self.verify_chassis(presented_chassis_secret)?;
        self.inner.ownership = OwnershipState::Unclaimed;
        self.inner.access_rows.clear();
        self.inner.current_epoch = 0;
        self.inner.revoked_accounts.clear();
        // The device-revocation set is CLEARED for the same reason as
        // `revoked_accounts` — a new owner starts from a blank slate (the
        // anti-rollback floor still defends the credential-replay vector).
        self.inner.revoked_devices.clear();
        // NOTE: self.inner.high_water_ns is intentionally NOT reset here (see the
        // doc invariants above — the anti-rollback floor survives resale).
        tracing::info!("robot factory-reset to unclaimed");
        Ok(())
    }

    fn verify_chassis(&self, presented: &[u8]) -> Result<(), PairingError> {
        let h = crypto::sha256_domain(CHASSIS_DSI, presented);
        if crypto::ct_eq(&h, &self.inner.chassis_secret_hash) {
            Ok(())
        } else {
            Err(PairingError::WrongChassisSecret)
        }
    }

    // -- new-pairing verification (strong path) ------------------------------

    /// Verify a new-pairing presentation offline against the root set and assert
    /// the device key equals the authenticated peer key. On success, advances the
    /// anti-rollback floor and returns the verified pairing (does **not** yet add
    /// an access row — call [`TrustStore::establish_pairing`]).
    ///
    /// `authenticated_peer_key` is the key the transport already mutually
    /// authenticated; `now_ns` is the best current time (NTP when reachable).
    pub fn verify_new_pairing(
        &mut self,
        pres: &PairingPresentation,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
    ) -> Result<VerifiedPairing, PairingError> {
        // A new pairing can only be established on a CLAIMED robot: the first
        // pairing must prove physical possession via `claim()` (which writes the
        // owner). Refuse at this earliest verification seam — the witness must
        // never be minted (and the anti-rollback floor never advanced) on an
        // unclaimed robot, consistent with the two persistence seams
        // (`establish_pairing` / `establish_code_pairing`).
        if !self.is_claimed() {
            return Err(PairingError::RobotUnclaimed);
        }
        self.verify_chain(pres, authenticated_peer_key, now_ns)
    }

    /// The pure offline chain verification shared by [`TrustStore::verify_new_pairing`]
    /// (new pairing on a claimed robot) and [`TrustStore::claim_by_owner_grant`]
    /// (the login-gated FIRST owner claim on an unclaimed robot). It performs the
    /// anti-rollback check, verifies the intermediate + device cert + grant chain,
    /// applies the revocation gates, and advances the anti-rollback floor on success.
    /// The **ownership** precondition is deliberately NOT checked here — each caller
    /// asserts its own (claimed vs. unclaimed) before calling. Extracting this keeps
    /// the two verification paths byte-identical, so the owner-claim path cannot
    /// silently diverge from the shipped, fuzzed new-pairing verifier.
    fn verify_chain(
        &mut self,
        pres: &PairingPresentation,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
    ) -> Result<VerifiedPairing, PairingError> {
        // Anti-rollback first: reject time-travel behind the known-good floor.
        if now_ns < self.inner.high_water_ns {
            return Err(PairingError::RollbackDetected {
                now_ns,
                high_water_ns: self.inner.high_water_ns,
            });
        }

        let intermediate_key = pres.intermediate.cert.intermediate_key;
        let intermediate_max_scope = pres.intermediate.cert.max_scope;
        chain::verify_intermediate(&self.inner.root_set, &pres.intermediate, now_ns)?;
        chain::verify_device_cert(
            &pres.device_cert,
            &intermediate_key,
            &intermediate_max_scope,
            authenticated_peer_key,
            now_ns,
        )?;
        let subject = pres.device_cert.cert.account;
        chain::verify_grant(
            &pres.grant,
            &intermediate_key,
            &intermediate_max_scope,
            &self.inner.robot,
            &subject,
            now_ns,
            pres.delegation.as_ref(),
        )?;

        // Revocation gates new pairings for the leaf subject, symmetric with the
        // delegator check below (both consult epoch + sticky owner revocation). A
        // distinct error keeps the operator signal accurate about WHICH revocation
        // fired: an owner-revoked leaf is refused here (it does NOT verify), so a
        // re-presented grant cannot silently mint a fresh witness — only an
        // explicit `owner_readmit` restores pairability.
        if self.inner.revoked_accounts.contains(&subject) {
            return Err(PairingError::RevokedByEpoch {
                epoch: self.inner.current_epoch,
            });
        }
        // Reaching here with `is_account_revoked` true means the epoch set did NOT
        // contain the subject (checked above), so it is a sticky owner revoke-now.
        if self.is_account_revoked(&subject) {
            return Err(PairingError::RevokedByOwner);
        }
        // For a delegated grant, the DELEGATOR must not be revoked — by epoch OR
        // by a local owner revoke-now — else its delegatees could establish new
        // pairings after the delegator was cut off.
        if let Some(del) = pres.delegation.as_ref() {
            if self.is_account_revoked(&pres.grant.grant.issuer) {
                return Err(PairingError::DelegatorRevoked);
            }
            // Security: the delegator's SIGNING device must not be
            // epoch-revoked. A depth-1 delegated grant is signed by the delegator's
            // DEVICE key (`del.delegator_cert.cert.device_key`), so a stolen delegator
            // laptop whose device key is revoked would — WITHOUT this gate — keep
            // signing delegated grants that verify (the delegator ACCOUNT stays valid,
            // only that device's authority dies). Symmetric with the owner-signing
            // device gate in `verify_owner_grant`.
            if self
                .inner
                .revoked_devices
                .contains(&del.delegator_cert.cert.device_key)
            {
                return Err(PairingError::RevokedDeviceByEpoch {
                    epoch: self.inner.current_epoch,
                });
            }
        }
        // A per-device tombstone. If the presenting DEVICE key is epoch-
        // revoked, refuse the pairing even when the account is fine (one desk was
        // cut, its siblings live). Checked here so re-presenting a valid old grant
        // cannot mint a fresh witness for a revoked device — a newer epoch dropping
        // it is the only re-admit. `authenticated_peer_key == device_cert.device_key`
        // (asserted by `verify_device_cert` above), so this is the dialing identity.
        if self.inner.revoked_devices.contains(authenticated_peer_key) {
            return Err(PairingError::RevokedDeviceByEpoch {
                epoch: self.inner.current_epoch,
            });
        }

        // Least-privilege effective scope: intersect device cert + grant, and for
        // a delegated grant also intersect the delegator's parent scope so the
        // effective scope can never exceed the delegator's own.
        let mut scope = Scope::intersect(pres.device_cert.cert.scope, pres.grant.grant.scope);
        if let Some(del) = &pres.delegation {
            scope = Scope::intersect(scope, del.parent.grant.scope);
        }

        // Advance the anti-rollback floor to the observed time ONLY (never fold a
        // raw issued_at — a future-dated cert must not poison the floor; issuance
        // is already bounded to <= now + skew by the chain verifier). Advance only
        // on success — a rejected cert never moves the floor.
        self.inner.high_water_ns = self.inner.high_water_ns.max(now_ns);

        Ok(VerifiedPairing::new(
            subject,
            scope,
            pres.device_cert.cert.principal_kind,
            pres.device_cert.cert.device_key,
        ))
    }

    /// A verify-then-establish convenience so callers who intend to persist a
    /// pairing cannot accidentally skip verification. Equivalent to
    /// [`TrustStore::verify_new_pairing`] followed by
    /// [`TrustStore::establish_pairing`].
    pub fn verify_and_establish(
        &mut self,
        pres: &PairingPresentation,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
        name: Option<String>,
    ) -> Result<VerifiedPairing, PairingError> {
        let verified = self.verify_new_pairing(pres, authenticated_peer_key, now_ns)?;
        // A strong-path (cloud-issued grant) pairing is an established pairing — it
        // never rots offline (ongoing access is gated by revocation, not expiry).
        self.establish_pairing(&verified, PairingSource::StrongChain, name, now_ns, None)?;
        Ok(verified)
    }

    /// Claim an **unclaimed** robot from a **login-gated owner grant**
    /// — the primary owner-claim path. The installer logs in at install, the
    /// account service issues an `OWNER_FULL` [`crate::format::SignedGrant`] for the
    /// installer's account (`POST /v1/robots`), and this seam verifies that grant's
    /// offline chain against the root set, asserts it confers the OWNER role, records
    /// the owner, and writes the owner [`AccessRow`] — with **zero robot↔cloud
    /// contact** beyond the one-time install login (invariant I3).
    ///
    /// This is the counterpart to the chassis-secret [`TrustStore::claim`]: both take
    /// an unclaimed robot to `Claimed(owner)` with an `OWNER_FULL` row, but this one
    /// is authorized by an offline-verifiable owner grant rather than a
    /// physical-possession secret. Since A2, `claim()` is the factory-reset /
    /// recovery path; this login-gated seam is the primary claim.
    ///
    /// `authenticated_peer_key` is the transport-authenticated key the installer
    /// presents — on a robot install this is the robot's own transport key (shell
    /// access to the robot IS the authority proof, by design). The device cert's
    /// `device_key` must equal it (asserted by the chain verifier), so a device cert
    /// for a different key cannot claim this robot.
    ///
    /// Precondition order (each refuses before the next is examined):
    /// 1. [`PairingError::AlreadyClaimed`] on an already-claimed robot — a claim is
    ///    the FIRST ownership write; a re-claim never silently re-owns a live robot.
    /// 2. The full offline chain (the same verifier as
    ///    [`TrustStore::verify_new_pairing`]) — anti-rollback,
    ///    intermediate → device cert → grant, with the device key bound to the
    ///    authenticated peer and the grant bound to THIS robot + the device cert's
    ///    account. Any chain failure surfaces its precise [`PairingError`].
    /// 3. [`PairingError::NotOwnerGrant`] when the verified effective scope does not
    ///    confer the OWNER role — only an OWNER-scoped grant may claim ownership.
    ///
    /// On success the ownership flips to `Claimed(owner)`, the owner `AccessRow`
    /// (scope = the verified effective scope, `source = PairingSource::OwnerGrant`,
    /// `name = "owner"`) is written, and the account is returned. An unclaimed robot
    /// carries no rows (the pairing seams all refuse until claimed), so this is
    /// always a fresh insert.
    pub fn claim_by_owner_grant(
        &mut self,
        pres: &PairingPresentation,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
    ) -> Result<AccountId, PairingError> {
        if self.is_claimed() {
            return Err(PairingError::AlreadyClaimed);
        }
        let verified = self.verify_chain(pres, authenticated_peer_key, now_ns)?;
        // The login-gated claim requires an OWNER-scoped grant: the effective scope
        // (device cert ∩ grant) must confer the OWNER role. A narrower grant
        // (operator/viewer) can never mint an owner — refuse loudly.
        if verified.scope().role != Role::OWNER {
            return Err(PairingError::NotOwnerGrant);
        }
        let account = verified.account();
        // Defense-in-depth, symmetric with the chassis-secret `claim()` recovery
        // path (which carries the identical guard): an unclaimed robot must carry
        // ZERO access rows (every pairing seam refuses until claimed, and both
        // `provision` and `factory_reset` leave the list empty, so this branch is
        // unreachable via the current public API). The PRIMARY claim path must not
        // defend LESS than the recovery path — if a future code path ever left a row
        // on an unclaimed store, those rows are illegitimate by construction (no
        // owner authorized them and nobody could see or revoke them), so the
        // legitimate owner's claim wipes them rather than inheriting a hidden
        // backdoor. This also neutralizes the stale-sticky-`revoked` inheritance
        // vector through `upsert_active_row` (which preserves a prior row's
        // `revoked` bit): a leftover revoked row for `account` can never leave the
        // freshly-written owner row stuck revoked. Loud so the invariant violation
        // never passes silently.
        if !self.inner.access_rows.is_empty() {
            tracing::warn!(
                rows = self.inner.access_rows.len(),
                "claim_by_owner_grant() found access rows on an unclaimed robot; clearing \
                 them as illegitimate (no owner authorized them) before writing the owner row"
            );
            self.inner.access_rows.clear();
        }
        self.inner.ownership = OwnershipState::Claimed(account);
        self.upsert_active_row(AccessRow {
            account,
            scope: verified.scope(),
            principal_kind: verified.principal_kind(),
            source: PairingSource::OwnerGrant,
            name: Some("owner".to_string()),
            added_at_ns: now_ns,
            // The owner row never expires (only owner-signed guest grants can).
            expires_at_ns: None,
            revoked: false,
        });
        // verify_chain already advanced the anti-rollback floor to `now_ns`.
        tracing::info!("robot claimed by owner account via login-gated owner grant");
        Ok(account)
    }

    // -- owner-signed access grant -----------------------------------------

    /// Verify a **desk-carried, owner-signed access grant** OFFLINE against this
    /// robot's OWN owner chain. The robot's owner signs a
    /// [`crate::format::SignedAccessGrant`] for a subject account; the desk carries it
    /// (bundled with the owner's device cert + the intermediate) and presents it with
    /// its own subject device cert at dial time. This is the "desk-carried owner-signed
    /// grants" model — fully offline, NO synced ACL, NO cloud contact.
    ///
    /// On success advances the anti-rollback floor and returns the verified pairing
    /// (does NOT yet add an access row — call [`TrustStore::establish_by_owner_grant`]).
    ///
    /// Precondition/verification order (each refuses before the next is examined):
    /// 1. [`PairingError::RobotUnclaimed`] — an owner grant can only be honored on a
    ///    CLAIMED robot (there must be an owner to verify the signature against).
    /// 2. [`PairingError::RollbackDetected`] — anti-rollback floor (uniform with every
    ///    credential-acceptance path).
    /// 3. The intermediate (root-set M-of-N), then the OWNER device cert (issuer chain
    ///    only — its key is NOT the dialing peer) whose account must equal this robot's
    ///    claimed owner ([`PairingError::NotRobotOwner`]), then the SUBJECT device cert
    ///    (full chain + `device_key == authenticated_peer_key`).
    /// 4. The owner-signed access grant: this robot, the subject, the owner, the owner
    ///    device key, validity, signature, and the owner-scope attenuation (the crate
    ///    -internal `chain::verify_owner_access_grant`).
    /// 5. Revocation gates: the SUBJECT must not be epoch- or owner-revoked, and the
    ///    OWNER (the grant's issuer) must not be revoked
    ///    ([`PairingError::DelegatorRevoked`]) — a cut-off owner cannot admit guests.
    /// 6. [`PairingError::OwnerGrantForOwner`] — refuse a grant naming the owner itself
    ///    (the owner already has full access; establishing a guest row for it could only
    ///    downgrade the owner's own row).
    ///
    /// The effective scope is the least-privilege intersection of the subject cert's
    /// scope and the grant's scope (the grant is itself already ≤ the owner's cert).
    pub fn verify_owner_grant(
        &mut self,
        pres: &OwnerGrantPresentation,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
    ) -> Result<VerifiedPairing, PairingError> {
        // An owner grant is meaningless without a claimed owner to verify against.
        let owner_account = match self.inner.ownership {
            OwnershipState::Claimed(o) => o,
            OwnershipState::Unclaimed => return Err(PairingError::RobotUnclaimed),
        };
        // Anti-rollback first: reject time-travel behind the known-good floor.
        if now_ns < self.inner.high_water_ns {
            return Err(PairingError::RollbackDetected {
                now_ns,
                high_water_ns: self.inner.high_water_ns,
            });
        }

        let intermediate_key = pres.intermediate.cert.intermediate_key;
        let intermediate_max_scope = pres.intermediate.cert.max_scope;
        chain::verify_intermediate(&self.inner.root_set, &pres.intermediate, now_ns)?;
        // The OWNER cert: verify the issuer chain (NOT the peer binding — the owner is
        // not the dialing peer), then assert it binds THIS robot's claimed owner.
        chain::verify_device_cert_chain(
            &pres.owner_cert,
            &intermediate_key,
            &intermediate_max_scope,
            now_ns,
        )?;
        if pres.owner_cert.cert.account != owner_account {
            return Err(PairingError::NotRobotOwner);
        }
        // The SUBJECT cert: full chain + the device_key == authenticated peer binding.
        chain::verify_device_cert(
            &pres.subject_cert,
            &intermediate_key,
            &intermediate_max_scope,
            authenticated_peer_key,
            now_ns,
        )?;
        let subject = pres.subject_cert.cert.account;
        // The owner-signed grant, checked against the owner cert's device key + scope.
        chain::verify_owner_access_grant(
            &pres.access_grant,
            &pres.owner_cert.cert.device_key,
            &pres.owner_cert.cert.scope,
            &self.inner.robot,
            &subject,
            &owner_account,
            now_ns,
        )?;

        // Revocation gates for the SUBJECT (symmetric with `verify_chain`): a distinct
        // error keeps the operator signal accurate about WHICH revocation fired.
        if self.inner.revoked_accounts.contains(&subject) {
            return Err(PairingError::RevokedByEpoch {
                epoch: self.inner.current_epoch,
            });
        }
        if self.is_account_revoked(&subject) {
            return Err(PairingError::RevokedByOwner);
        }
        // The OWNER is the issuer here (the "delegator" of the strong path). A revoked
        // owner must not be able to admit new guests — refuse loudly.
        if self.is_account_revoked(&owner_account) {
            return Err(PairingError::DelegatorRevoked);
        }
        // Security: the OWNER's SIGNING device must not be epoch-revoked.
        // A stolen owner laptop that gets its device key revoked retains a valid,
        // unexpired owner cert + owner account, so WITHOUT this gate it could keep
        // SIGNING grants that verify — the revoked device would retain signing
        // authority. Refuse when the owner cert's device key is revoked; the owner
        // ACCOUNT stays valid (its OTHER devices can still sign), only this device's
        // authority dies. Checked BEFORE the subject-device gate because a grant minted
        // by a revoked signer is illegitimate regardless of the subject.
        if self
            .inner
            .revoked_devices
            .contains(&pres.owner_cert.cert.device_key)
        {
            return Err(PairingError::RevokedDeviceByEpoch {
                epoch: self.inner.current_epoch,
            });
        }
        // The per-device tombstone (symmetric with `verify_chain`). A
        // revoked SUBJECT device is refused even when its account is allowed, so a
        // desk-carried owner-signed grant cannot re-establish a cut desk. The subject
        // cert's `device_key == authenticated_peer_key` (asserted above).
        if self.inner.revoked_devices.contains(authenticated_peer_key) {
            return Err(PairingError::RevokedDeviceByEpoch {
                epoch: self.inner.current_epoch,
            });
        }
        // The owner needs no grant for its own access; refuse a self-grant that could
        // only downgrade the owner's own claim row.
        if subject == owner_account {
            return Err(PairingError::OwnerGrantForOwner);
        }

        // Least-privilege effective scope: subject cert ∩ owner-signed grant.
        let scope = Scope::intersect(pres.subject_cert.cert.scope, pres.access_grant.grant.scope);

        // Advance the anti-rollback floor to observed time only (never a raw future
        // issued_at — bounded to <= now + skew by the chain verifier). Success only.
        self.inner.high_water_ns = self.inner.high_water_ns.max(now_ns);

        Ok(VerifiedPairing::new(
            subject,
            scope,
            pres.subject_cert.cert.principal_kind,
            pres.subject_cert.cert.device_key,
        ))
    }

    /// Verify a desk-carried owner-signed grant ([`TrustStore::verify_owner_grant`])
    /// and, on success, persist a durable [`PairingSource::OwnerSignedGrant`] access
    /// row for the subject. The verify-then-establish convenience so callers who intend
    /// to persist cannot accidentally skip verification (mirrors
    /// [`TrustStore::verify_and_establish`]).
    pub fn establish_by_owner_grant(
        &mut self,
        pres: &OwnerGrantPresentation,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
        name: Option<String>,
    ) -> Result<VerifiedPairing, PairingError> {
        let verified = self.verify_owner_grant(pres, authenticated_peer_key, now_ns)?;
        // A desk-carried owner-signed grant carries its OWN finite validity: honor
        // it as the durable row's expiry so the owner's time-boxed guest access
        // DENIES offline once the clock passes it (the doc on
        // `AccessGrant::validity` promises an "offline-honorable expiry"). An
        // unbounded grant (`not_after_ns == u64::MAX`) maps to `None` (no expiry).
        let expires_at_ns = access_row_expiry(pres.access_grant.grant.validity.not_after_ns);
        self.establish_pairing(
            &verified,
            PairingSource::OwnerSignedGrant,
            name,
            now_ns,
            expires_at_ns,
        )?;
        Ok(verified)
    }

    /// Whether an account is currently revoked — either by the current epoch's
    /// revocation set OR by a sticky owner "revoke-now" on its access row.
    fn is_account_revoked(&self, account: &AccountId) -> bool {
        self.inner.revoked_accounts.contains(account)
            || self
                .inner
                .access_rows
                .iter()
                .any(|r| &r.account == account && r.revoked)
    }

    /// Establish (persist) a verified pairing as an access-list row.
    /// If the account already has a row, its scope/name/source/expiry are updated
    /// **but a prior owner-revocation sticks** (a re-presented grant does not
    /// silently un-revoke a locally revoked account — call
    /// [`TrustStore::owner_readmit`]).
    ///
    /// `expires_at_ns` is the row's finite access expiry: the strong
    /// path and code pairing pass `None` (an established pairing never rots), while
    /// a desk-carried owner-signed grant passes the grant's finite `not_after_ns` so
    /// [`TrustStore::is_allowed`] denies the row once the clock passes it. Because a
    /// re-presented grant REFRESHES the expiry (the update branch overwrites it), a
    /// renewed owner grant extends access and a shorter re-grant tightens it.
    ///
    /// Refuses with [`PairingError::RobotUnclaimed`] on an unclaimed robot: no
    /// access row may exist before the owner claims. This is checked
    /// independently of [`TrustStore::verify_new_pairing`] (which also refuses on
    /// unclaimed) because a [`VerifiedPairing`] is `Copy` and could be minted
    /// while claimed, then replayed after a `factory_reset` returned the robot to
    /// unclaimed — this gate blocks that stale-witness vector.
    pub fn establish_pairing(
        &mut self,
        verified: &VerifiedPairing,
        source: PairingSource,
        name: Option<String>,
        now_ns: u64,
        expires_at_ns: Option<u64>,
    ) -> Result<(), PairingError> {
        if !self.is_claimed() {
            return Err(PairingError::RobotUnclaimed);
        }
        if let Some(row) = self
            .inner
            .access_rows
            .iter_mut()
            .find(|r| r.account == verified.account)
        {
            row.scope = verified.scope;
            row.principal_kind = verified.principal_kind;
            row.source = source;
            row.name = name;
            row.added_at_ns = now_ns;
            // A re-presented grant refreshes the expiry (extend OR tighten).
            row.expires_at_ns = expires_at_ns;
            // row.revoked deliberately preserved (sticky owner-revocation).
        } else {
            self.inner.access_rows.push(AccessRow {
                account: verified.account,
                scope: verified.scope,
                principal_kind: verified.principal_kind,
                source,
                name,
                added_at_ns: now_ns,
                expires_at_ns,
                revoked: false,
            });
        }
        Ok(())
    }

    /// Establish a **code-paired** (CPace fallback) row: visible, named,
    /// revocable, with a limited default scope. REQUIRES a [`CpaceConfirmed`] — the
    /// unforgeable proof that the CPace ceremony actually succeeded — so a
    /// code-paired row can never be persisted without running the PAKE (the same
    /// proof-before-persist discipline the strong path enforces via
    /// [`VerifiedPairing`]). The witness binds the proven account: `account` must
    /// equal `confirmed.account()`, else [`PairingError::WitnessAccountMismatch`]
    /// (a confirmation for account A cannot persist account B). The principal kind
    /// comes from the witness; `name` and `scope` are the higher layer's choice.
    ///
    /// **Epoch-independence (deliberate):** code pairing does NOT touch the
    /// revocation epoch or the anti-rollback high-water mark. It is a LOCAL,
    /// owner-approved fallback add, not a cloud-issued credential with an issuance
    /// time — there is no cert/epoch that a clock rollback could replay, and
    /// nothing to anchor the floor to. Ongoing access still honors epoch + owner
    /// revocation through [`TrustStore::is_allowed`], exactly like the strong path.
    ///
    /// Precondition order (each refuses before the next is examined):
    /// 1. [`PairingError::RobotUnclaimed`] on an unclaimed robot — the FIRST
    ///    pairing must prove physical possession via `claim()`, and an unclaimed
    ///    robot must carry zero access rows (a code-paired guest row on an
    ///    ownerless robot would be invisible and unrevocable). Checked first so an
    ///    unclaimed robot rejects uniformly regardless of the presented witness.
    /// 2. [`PairingError::WitnessRobotMismatch`] when the witness's
    ///    `responder_key` is not this robot's own transport key — a `CpaceConfirmed`
    ///    proven against a DIFFERENT robot cannot be replayed here (cross-robot
    ///    witness replay). Checked before the account match: a witness for the
    ///    wrong robot is rejected as such even if its account happens to match.
    /// 3. [`PairingError::WitnessAccountMismatch`] when the witness's account is
    ///    not the account being established (a confirmation for account A cannot
    ///    persist account B).
    pub fn establish_code_pairing(
        &mut self,
        confirmed: &CpaceConfirmed,
        account: AccountId,
        name: String,
        scope: Scope,
        now_ns: u64,
    ) -> Result<(), PairingError> {
        if !self.is_claimed() {
            return Err(PairingError::RobotUnclaimed);
        }
        if confirmed.responder_key() != self.inner.robot_transport_key {
            return Err(PairingError::WitnessRobotMismatch);
        }
        if confirmed.account() != account {
            return Err(PairingError::WitnessAccountMismatch);
        }
        self.upsert_active_row(AccessRow {
            account,
            scope,
            principal_kind: confirmed.principal_kind(),
            source: PairingSource::CodePaired,
            name: Some(name),
            added_at_ns: now_ns,
            // A code-paired row is an established local pairing — it never rots
            // (only owner-signed guest grants carry a finite expiry).
            expires_at_ns: None,
            revoked: false,
        });
        Ok(())
    }

    // -- revocation ----------------------------------------------------------

    /// Apply a pushed access-list epoch (CRL-lite). The epoch must be signed by
    /// the intermediate (verified against the root set), be for this robot, and
    /// be strictly newer than the current epoch (monotonic). On apply, its
    /// revocation set becomes authoritative.
    pub fn apply_epoch(
        &mut self,
        signed_epoch: &SignedEpoch,
        intermediate: &SignedIntermediateCert,
        now_ns: u64,
    ) -> Result<EpochOutcome, PairingError> {
        // Anti-rollback floor is UNIFORM across every credential acceptance path:
        // a rolled-back clock must not be able to apply an epoch that revives an
        // expired intermediate.
        if now_ns < self.inner.high_water_ns {
            return Err(PairingError::RollbackDetected {
                now_ns,
                high_water_ns: self.inner.high_water_ns,
            });
        }
        chain::verify_intermediate(&self.inner.root_set, intermediate, now_ns)?;
        let intermediate_key = intermediate.cert.intermediate_key;
        let e = &signed_epoch.epoch_data;

        if e.issuer_key != intermediate_key {
            return Err(PairingError::IssuerMismatch(CertKind::Epoch));
        }
        match crypto::ed25519_verify(&e.issuer_key, &e.signing_payload(), &signed_epoch.signature) {
            VerifyResult::Ok => {}
            VerifyResult::BadKey => return Err(PairingError::BadKey),
            VerifyResult::BadSig => return Err(PairingError::BadSignature(CertKind::Epoch)),
        }
        chain::check_issued_at(e.issued_at_ns, now_ns, CertKind::Epoch)?;
        if e.robot != self.inner.robot {
            return Err(PairingError::WrongRobot);
        }
        if e.epoch <= self.inner.current_epoch {
            return Ok(EpochOutcome::NotNewer {
                current: self.inner.current_epoch,
            });
        }

        // Defensive OBSERVABILITY (NOT enforcement): the account service
        // guards against revoking a robot's own owner (`robot_revoke`), so the claimed
        // owner should never appear in a synced epoch's revoked-account set. If it
        // does, something upstream is wrong (a compromised/buggy issuer, or a future
        // owner-change flow that hasn't updated this robot's ownership yet) — warn
        // loudly so the operator sees it, but apply the epoch AS-IS. We deliberately do
        // NOT special-case it here: a robot-side enforcement carve-out could silently
        // break a legitimate future owner-account change (the wrong location per the
        // review). The epoch is authoritative; the guard belongs at the mint.
        if let OwnershipState::Claimed(owner) = self.inner.ownership {
            if e.revoked_accounts.contains(&owner) {
                tracing::warn!(
                    epoch = e.epoch,
                    "a synced access-list epoch revokes this robot's OWN claimed \
                     owner account — applying it as-is (the account service should have refused \
                     this at the mint). This will lock the owner out until a newer epoch drops it \
                     or a factory reset."
                );
            }
        }

        // Count rows that transition from allowed to epoch-revoked.
        let old = &self.inner.revoked_accounts;
        let newly_revoked = self
            .inner
            .access_rows
            .iter()
            .filter(|r| {
                !r.revoked && e.revoked_accounts.contains(&r.account) && !old.contains(&r.account)
            })
            .count();
        // Count the DEVICE keys newly appearing in this epoch's set (for
        // the operator breadcrumb only — devices are not access-list rows, so there
        // is nothing to fold into `EpochOutcome::Applied.newly_revoked`, which stays
        // account-scoped and unchanged).
        let old_devices = &self.inner.revoked_devices;
        let newly_revoked_devices = e
            .revoked_devices
            .iter()
            .filter(|d| !old_devices.contains(d))
            .count();

        self.inner.current_epoch = e.epoch;
        self.inner.revoked_accounts = e.revoked_accounts.clone();
        // The device-revocation set moves with the account set:
        // authoritative + monotonic (the epoch REPLACES it wholesale, so a newer
        // epoch dropping a device is the only re-admit — the tombstone).
        self.inner.revoked_devices = e.revoked_devices.clone();
        // Advance the anti-rollback floor to observed time only (never a raw
        // future issued_at — bounded to <= now + skew above).
        self.inner.high_water_ns = self.inner.high_water_ns.max(now_ns);
        tracing::info!(
            epoch = e.epoch,
            newly_revoked,
            newly_revoked_devices,
            "applied access-list epoch"
        );
        Ok(EpochOutcome::Applied { newly_revoked })
    }

    /// Owner "revoke-now": mark an account's row revoked over a live owner
    /// connection. The owner cannot revoke itself (self-lockout guard). Idempotent
    /// for accounts with no row.
    pub fn owner_revoke(
        &mut self,
        requesting_owner: &AccountId,
        target: &AccountId,
    ) -> Result<(), PairingError> {
        self.assert_owner(requesting_owner)?;
        if Some(*target) == self.inner.ownership.owner() {
            // Never let the owner lock itself out.
            return Ok(());
        }
        if let Some(row) = self
            .inner
            .access_rows
            .iter_mut()
            .find(|r| &r.account == target)
        {
            row.revoked = true;
            tracing::info!("owner revoked an account (revoke-now)");
        }
        Ok(())
    }

    /// Owner re-admit: clear a sticky owner-revocation on a row. (Does not affect
    /// epoch revocation.)
    pub fn owner_readmit(
        &mut self,
        requesting_owner: &AccountId,
        target: &AccountId,
    ) -> Result<(), PairingError> {
        self.assert_owner(requesting_owner)?;
        if let Some(row) = self
            .inner
            .access_rows
            .iter_mut()
            .find(|r| &r.account == target)
        {
            row.revoked = false;
        }
        Ok(())
    }

    fn assert_owner(&self, requesting_owner: &AccountId) -> Result<(), PairingError> {
        match self.inner.ownership.owner() {
            Some(o) if &o == requesting_owner => Ok(()),
            Some(_) => Err(PairingError::NotOwner),
            None => Err(PairingError::Unclaimed),
        }
    }

    /// Insert a row, or update an existing row for the same account. A prior
    /// **owner-revocation sticks** — a re-pairing (code pairing or claim) must NOT
    /// silently readmit a revoked account; only [`TrustStore::owner_readmit`]
    /// clears it. (After a factory reset the list is empty, so the owner claim is
    /// always a fresh, non-revoked insert.)
    fn upsert_active_row(&mut self, new_row: AccessRow) {
        if let Some(row) = self
            .inner
            .access_rows
            .iter_mut()
            .find(|r| r.account == new_row.account)
        {
            let was_revoked = row.revoked;
            *row = new_row;
            row.revoked = was_revoked;
            if was_revoked {
                tracing::warn!(
                    "pairing targets an owner-revoked account; the row stays revoked \
                     until an explicit owner_readmit"
                );
            }
        } else {
            self.inner.access_rows.push(new_row);
        }
    }
}

/// Map an owner-signed grant's validity upper bound (`not_after_ns`) to a durable
/// [`AccessRow`] expiry: a finite bound becomes the row's honorable expiry, while the
/// unbounded sentinel (`u64::MAX`, the [`crate::format::AccessGrant`] "no expiry"
/// encoding) becomes `None` so the row never rots. Pure so it is oracle-testable.
fn access_row_expiry(not_after_ns: u64) -> Option<u64> {
    (not_after_ns != u64::MAX).then_some(not_after_ns)
}

/// Sibling temp path for atomic writes: append `.tmp` to the file name.
fn tmp_path(path: &Path) -> PathBuf {
    let mut name = path.file_name().unwrap_or_default().to_os_string();
    name.push(".tmp");
    path.with_file_name(name)
}
