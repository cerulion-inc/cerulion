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
//! atomic and durable: the bytes go to a sibling `.tmp` that is fsynced, renamed
//! over the store, and then the directory is fsynced, so after a power loss the
//! store holds either the old bytes or the new ones. The MAC key is supplied by the
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
use super::{
    OwnerCertificatePresentationWire, OwnerGrantPresentation, PairingPresentation, VerifiedPairing,
};
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
// v4: access rows are keyed by `(account, device)` (`AccessRow::device`), and
// `TrustStoreInner` gained `allow_cloud`, the per-robot allow-from-cloud switch.
// A v3 file is MIGRATED, not refused (robots in the field carry v3 stores): every
// v3 row becomes an account-wide row (`device: None`), in the same order, and
// `allow_cloud` starts ON. The MAC is verified with the same key before the v3
// body is read, and the migration logs once at warn. The next save writes v4.
const STORE_FORMAT_VERSION: u16 = 4;
const STORE_FORMAT_VERSION_V3: u16 = 3;
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
    /// The per-robot allow-from-cloud switch. When `false`, every row whose
    /// principal is [`crate::format::PrincipalKind::Cloud`] is refused
    /// ([`TrustStore::cloud_allowed`]) and no new cloud row is written. ON at
    /// provision, after a factory reset, and for a migrated v3 store.
    allow_cloud: bool,
}

/// The v3 body, kept only to migrate a v3 file (field order is the wire).
#[derive(Deserialize)]
struct TrustStoreInnerV3 {
    store_version: u16,
    robot: RobotId,
    robot_transport_key: PublicKey,
    root_set: RootSet,
    ownership: OwnershipState,
    chassis_secret_hash: [u8; 32],
    access_rows: Vec<AccessRowV3>,
    current_epoch: u64,
    revoked_accounts: Vec<AccountId>,
    revoked_devices: Vec<PublicKey>,
    high_water_ns: u64,
}

/// A v3 access row: keyed by account only.
#[derive(Deserialize)]
struct AccessRowV3 {
    account: AccountId,
    scope: Scope,
    principal_kind: crate::format::PrincipalKind,
    source: PairingSource,
    name: Option<String>,
    added_at_ns: u64,
    expires_at_ns: Option<u64>,
    revoked: bool,
}

impl TrustStoreInnerV3 {
    /// The v4 state: each row account-wide (`device: None`), order kept,
    /// allow-from-cloud ON.
    fn migrate(self) -> TrustStoreInner {
        let _ = self.store_version;
        TrustStoreInner {
            store_version: STORE_FORMAT_VERSION,
            robot: self.robot,
            robot_transport_key: self.robot_transport_key,
            root_set: self.root_set,
            ownership: self.ownership,
            chassis_secret_hash: self.chassis_secret_hash,
            access_rows: self
                .access_rows
                .into_iter()
                .map(|row| AccessRow {
                    account: row.account,
                    device: None,
                    scope: row.scope,
                    principal_kind: row.principal_kind,
                    source: row.source,
                    name: row.name,
                    added_at_ns: row.added_at_ns,
                    expires_at_ns: row.expires_at_ns,
                    revoked: row.revoked,
                })
                .collect(),
            current_epoch: self.current_epoch,
            revoked_accounts: self.revoked_accounts,
            revoked_devices: self.revoked_devices,
            high_water_ns: self.high_water_ns,
            allow_cloud: true,
        }
    }
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
                allow_cloud: true,
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
    /// if the MAC does not verify. On Unix, when `path` itself is a regular file
    /// with a single link, its group and other permission bits are cleared before
    /// it is read; a symbolic link at `path` is read through unchanged.
    pub fn load(path: impl AsRef<Path>, mac_key: &[u8]) -> Result<Self, PairingError> {
        let bytes = read_private(path.as_ref())?;
        let inner = Self::decode(&bytes, mac_key)?;
        Ok(TrustStore {
            inner,
            path: Some(path.as_ref().to_path_buf()),
        })
    }

    /// Atomically and durably write the store to its backing path,
    /// authenticated with `mac_key`: create a fresh sibling temporary file
    /// (exclusively, mode 0600 on Unix), write and fsync it, rename it over the
    /// store, then fsync the directory. The store takes the temporary file's mode
    /// at the rename: 0600, less any bits the umask clears.
    ///
    /// A failure up to and including the rename leaves the old store in place and
    /// is returned. A failed directory fsync after the rename is logged at `warn`
    /// and the save returns `Ok`, since the new store is already in place; if that
    /// save recorded a revocation, a power loss can then bring back the previous
    /// store, in which the revoked device still has access, and the `warn` line is
    /// the only trace of it.
    ///
    /// Each save uses its own temporary file, so concurrent saves never share one
    /// and the last rename wins.
    pub fn save(&self, mac_key: &[u8]) -> Result<(), PairingError> {
        self.save_with(mac_key, &mut StdSaveFs)
    }

    /// [`Self::save`] over an injected [`SaveFs`].
    fn save_with(&self, mac_key: &[u8], fs: &mut impl SaveFs) -> Result<(), PairingError> {
        let path = self.path.as_ref().ok_or(PairingError::NoStorePath)?;
        let bytes = self.encode(mac_key)?;
        replace_durably(fs, &unique_tmp_path(path), path, &bytes)?;
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
        if ver != STORE_FORMAT_VERSION && ver != STORE_FORMAT_VERSION_V3 {
            return Err(PairingError::StoreCorrupt("unsupported format version"));
        }
        let body_len =
            u32::from_le_bytes([framed[10], framed[11], framed[12], framed[13]]) as usize;
        let body = &framed[HEADER_LEN..];
        if body.len() != body_len {
            return Err(PairingError::StoreCorrupt("body length mismatch"));
        }
        if ver == STORE_FORMAT_VERSION_V3 {
            let v3: TrustStoreInnerV3 = postcard::from_bytes(body).map_err(ser_err)?;
            if v3.store_version != STORE_FORMAT_VERSION_V3 {
                return Err(PairingError::StoreCorrupt("format version mismatch"));
            }
            let rows = v3.access_rows.len();
            tracing::warn!(
                rows,
                "trust store migrated from format v3 to v4: every access row is now an \
                 account-wide row and allow-from-cloud is on; the next save writes v4"
            );
            return Ok(v3.migrate());
        }
        let inner: TrustStoreInner = postcard::from_bytes(body).map_err(ser_err)?;
        if inner.store_version != STORE_FORMAT_VERSION {
            return Err(PairingError::StoreCorrupt("format version mismatch"));
        }
        Ok(inner)
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
    pub fn is_allowed(
        &self,
        account: &AccountId,
        device: &PublicKey,
        now_ns: u64,
    ) -> Option<&AccessRow> {
        if self.inner.revoked_accounts.contains(account) {
            return None;
        }
        // A device's own row decides for that device: a revoked or expired
        // device row does not fall back to the account-wide row.
        let row = self
            .row_for(account, Some(device))
            .or_else(|| self.row_for(account, None))?;
        row_active(row, now_ns).then_some(row)
    }

    /// Whether `account` has access at all: its account-wide row when that is
    /// active, else its first active device row, under the same revocation and
    /// expiry rules as [`TrustStore::is_allowed`]. For listings and diagnostics;
    /// admission of a connection always uses [`TrustStore::is_allowed`] with the
    /// connection's device key.
    pub fn account_access(&self, account: &AccountId, now_ns: u64) -> Option<&AccessRow> {
        if self.inner.revoked_accounts.contains(account) {
            return None;
        }
        let mut rows = self
            .inner
            .access_rows
            .iter()
            .filter(|r| &r.account == account);
        rows.clone()
            .find(|r| r.device.is_none())
            .filter(|r| row_active(r, now_ns))
            .or_else(|| rows.find(|r| r.device.is_some() && row_active(r, now_ns)))
    }

    /// The row keyed by exactly `(account, device)` (`None` = account-wide).
    fn row_for(&self, account: &AccountId, device: Option<&PublicKey>) -> Option<&AccessRow> {
        self.inner
            .access_rows
            .iter()
            .find(|r| &r.account == account && r.device.as_ref() == device)
    }

    /// The one allow-from-cloud predicate: `false` exactly when `row`'s
    /// principal is [`crate::format::PrincipalKind::Cloud`] and this robot's
    /// allow-from-cloud switch is OFF. Every plane's admission (ops, wire, and
    /// any later one) calls it on the row it is about to honor.
    pub fn cloud_allowed(&self, row: &AccessRow) -> bool {
        row.principal_kind != crate::format::PrincipalKind::Cloud || self.inner.allow_cloud
    }

    /// The allow-from-cloud switch (ON unless the owner turned it off).
    pub fn allow_cloud(&self) -> bool {
        self.inner.allow_cloud
    }

    /// Set the allow-from-cloud switch. Owner only ([`PairingError::NotOwner`]
    /// otherwise, [`PairingError::Unclaimed`] on an unclaimed robot).
    pub fn set_allow_cloud(
        &mut self,
        requesting_owner: &AccountId,
        allow: bool,
    ) -> Result<(), PairingError> {
        self.assert_owner(requesting_owner)?;
        self.inner.allow_cloud = allow;
        tracing::info!(allow, "allow-from-cloud switch set by the owner");
        Ok(())
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
        // The owner row replaces the account's account-wide row, if any.
        self.upsert_active_row(AccessRow {
            account,
            // The owner row is account-wide: each of the owner's devices that
            // presents a certificate adds its own row beside it.
            device: None,
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
        // A new owner starts with the switch at its default.
        self.inner.allow_cloud = true;
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

    /// Certificate presentation: admit one of the OWNER account's devices from
    /// its roots-verified device certificate chain, with no code and no user step.
    /// The chain must end in a certificate for `authenticated_peer_key` and the
    /// robot's owner account, its scope must cover the owner row's scope, and the
    /// owner row must be active. On success the device gets its own row
    /// `(owner, device)` with the owner row's scope and expiry and the
    /// certificate's principal (so a cloud device stays a cloud row); the owner
    /// row itself is never written. A cloud certificate is refused while the
    /// allow-from-cloud switch is OFF. A re-presentation refreshes the device's
    /// row and keeps a sticky owner revocation. The rollback floor advances.
    pub fn verify_owner_certificate(
        &mut self,
        pres: &OwnerCertificatePresentationWire,
        authenticated_peer_key: &PublicKey,
        now_ns: u64,
    ) -> Result<AccountId, PairingError> {
        pres.validate_version()?;
        let owner = self.owner().ok_or(PairingError::RobotUnclaimed)?;
        if now_ns < self.inner.high_water_ns {
            return Err(PairingError::RollbackDetected {
                now_ns,
                high_water_ns: self.inner.high_water_ns,
            });
        }
        chain::verify_intermediate(&self.inner.root_set, &pres.intermediate, now_ns)?;
        chain::verify_device_cert(
            &pres.device_cert,
            &pres.intermediate.cert.intermediate_key,
            &pres.intermediate.cert.max_scope,
            authenticated_peer_key,
            now_ns,
        )?;
        let cert = &pres.device_cert.cert;
        if cert.account != owner {
            return Err(PairingError::NotOwner);
        }
        if self.inner.revoked_accounts.contains(&owner) {
            return Err(PairingError::RevokedByEpoch {
                epoch: self.inner.current_epoch,
            });
        }
        if self.inner.revoked_devices.contains(authenticated_peer_key) {
            return Err(PairingError::RevokedDeviceByEpoch {
                epoch: self.inner.current_epoch,
            });
        }
        let owner_row = self
            .row_for(&owner, None)
            .ok_or(PairingError::StoreCorrupt(
                "claimed owner has no access row",
            ))?;
        if owner_row.revoked {
            return Err(PairingError::RevokedByOwner);
        }
        if owner_row
            .expires_at_ns
            .is_some_and(|expiry| now_ns >= expiry)
        {
            return Err(PairingError::OwnerAccessExpired);
        }
        if !owner_row.scope.is_attenuation_of(&cert.scope) {
            return Err(PairingError::OwnerCertificateScopeInsufficient);
        }
        if cert.principal_kind == crate::format::PrincipalKind::Cloud && !self.inner.allow_cloud {
            return Err(PairingError::CloudAccessDisabled);
        }
        let device_row = AccessRow {
            account: owner,
            device: Some(*authenticated_peer_key),
            scope: owner_row.scope,
            principal_kind: cert.principal_kind,
            source: PairingSource::CertificatePresentation,
            name: None,
            added_at_ns: now_ns,
            expires_at_ns: row_expiry(
                cert.principal_kind,
                cert.validity.not_after_ns,
                owner_row.expires_at_ns,
            ),
            revoked: false,
        };
        self.upsert_active_row(device_row);
        self.inner.high_water_ns = now_ns;
        Ok(owner)
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
            pres.device_cert.cert.validity.not_after_ns,
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
            device: None,
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
            pres.subject_cert.cert.validity.not_after_ns,
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

    /// Establish (persist) a verified pairing as the presenting device's own
    /// access-list row, keyed by `(account, device)`; other devices' rows of the
    /// same account, the owner row included, are never touched.
    /// If that device already has a row, its scope/name/source/expiry are updated
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
        if verified.principal_kind == crate::format::PrincipalKind::Cloud && !self.inner.allow_cloud
        {
            return Err(PairingError::CloudAccessDisabled);
        }
        let expires_at_ns = row_expiry(
            verified.principal_kind,
            verified.cert_not_after_ns,
            expires_at_ns,
        );
        let device = Some(verified.device_key);
        if let Some(row) = self
            .inner
            .access_rows
            .iter_mut()
            .find(|r| r.account == verified.account && r.device == device)
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
                device,
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
    /// 4. [`PairingError::CodePairOwnerAccount`] when the account is the robot's
    ///    owner: the owner row is never written by a code pairing.
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
        if self.owner() == Some(account) {
            return Err(PairingError::CodePairOwnerAccount);
        }
        self.upsert_active_row(AccessRow {
            account,
            device: Some(confirmed.initiator_key()),
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
        let mut revoked = 0usize;
        for row in self
            .inner
            .access_rows
            .iter_mut()
            .filter(|r| &r.account == target)
        {
            row.revoked = true;
            revoked += 1;
        }
        if revoked > 0 {
            tracing::info!(rows = revoked, "owner revoked an account (revoke-now)");
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
        for row in self
            .inner
            .access_rows
            .iter_mut()
            .filter(|r| &r.account == target)
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

    /// Insert a row, or update the existing row for the same `(account, device)`. A prior
    /// **owner-revocation sticks** — a re-pairing (code pairing or claim) must NOT
    /// silently readmit a revoked account; only [`TrustStore::owner_readmit`]
    /// clears it. (After a factory reset the list is empty, so the owner claim is
    /// always a fresh, non-revoked insert.)
    fn upsert_active_row(&mut self, new_row: AccessRow) {
        if let Some(row) = self
            .inner
            .access_rows
            .iter_mut()
            .find(|r| r.account == new_row.account && r.device == new_row.device)
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

/// The expiry of a row written from a certificate: `expires_at_ns` for a human
/// or machine principal; for a cloud principal the earlier of `expires_at_ns`
/// and the certificate's `not_after` (so the row lapses with the certificate,
/// and each renewed certificate presented extends it).
fn row_expiry(
    principal: crate::format::PrincipalKind,
    cert_not_after_ns: u64,
    expires_at_ns: Option<u64>,
) -> Option<u64> {
    if principal != crate::format::PrincipalKind::Cloud {
        return expires_at_ns;
    }
    match (expires_at_ns, access_row_expiry(cert_not_after_ns)) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    }
}

/// Whether `row` admits at `now_ns`: not revoked, and `now_ns` strictly before a
/// finite expiry (the `[not_before, not_after)` half-open window, so
/// `now_ns >= expiry` denies). `None` never expires. The one rule behind
/// [`TrustStore::is_allowed`] and [`TrustStore::account_access`].
fn row_active(row: &AccessRow, now_ns: u64) -> bool {
    !row.revoked && row.expires_at_ns.is_none_or(|exp| now_ns < exp)
}

/// Map an owner-signed grant's validity upper bound (`not_after_ns`) to a durable
/// [`AccessRow`] expiry: a finite bound becomes the row's honorable expiry, while the
/// unbounded sentinel (`u64::MAX`, the [`crate::format::AccessGrant`] "no expiry"
/// encoding) becomes `None` so the row never rots. Pure so it is oracle-testable.
fn access_row_expiry(not_after_ns: u64) -> Option<u64> {
    (not_after_ns != u64::MAX).then_some(not_after_ns)
}

/// A fresh sibling name for one save's temporary file, `.<name>.<16 hex>.tmp` in
/// the store's directory. The hex hashes the process id and a per-process
/// counter under std's OS-seeded `RandomState`, so two saves (in one process or
/// two) and a crashed save's leftover do not share a name. The name is unique,
/// not secret: the exclusive create in [`StdSaveFs`] is what refuses a path that
/// is already there.
fn unique_tmp_path(path: &Path) -> PathBuf {
    use std::hash::{BuildHasher, Hasher};
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
    hasher.write_u32(std::process::id());
    hasher.write_u64(NEXT.fetch_add(1, Ordering::Relaxed));
    let mut name = std::ffi::OsString::from(".");
    name.push(path.file_name().unwrap_or_default());
    name.push(format!(".{:016x}.tmp", hasher.finish()));
    path.with_file_name(name)
}

/// Read the store file at `path`. On Unix, when `path` is itself a regular file
/// with a single link that group or other users can access, its group and other
/// permission bits are cleared through the open handle before it is read, logged at
/// `warn` (see `narrow_to_owner`; a symbolic link at `path` is read through and its
/// target is not changed). When clearing them fails, that is logged at `warn` and
/// the read goes on.
fn read_private(path: &Path) -> std::io::Result<Vec<u8>> {
    use std::io::Read as _;
    #[cfg(unix)]
    let at_path = std::fs::symlink_metadata(path).ok();
    let mut file = std::fs::File::open(path)?;
    #[cfg(unix)]
    narrow_to_owner(&file, path, at_path.as_ref());
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)?;
    Ok(bytes)
}

/// Clear the group and other permission bits of the open store `file`.
///
/// `at_path` is the metadata of `path` itself, read before the open without
/// following a link. The bits are cleared only when the open handle is the same
/// device and inode as the regular file found at `path` when its metadata was
/// read, and that file has a single link. A path that is a symbolic link or not
/// a regular file, that shares its file with another name, that was replaced
/// between the two reads, or whose metadata could not be read is read as usual
/// and its mode is left alone, logged at `warn`.
#[cfg(unix)]
fn narrow_to_owner(file: &std::fs::File, path: &Path, at_path: Option<&std::fs::Metadata>) {
    use std::os::unix::fs::{MetadataExt as _, PermissionsExt as _};
    let Some(at_path) = at_path else {
        tracing::warn!(
            path = %path.display(),
            "the trust store's metadata could not be read before it was opened; its mode \
             was not checked"
        );
        return;
    };
    if at_path.file_type().is_symlink() {
        tracing::warn!(
            path = %path.display(),
            "the trust store path is a symbolic link; the mode of its target is left alone"
        );
        return;
    }
    if !at_path.is_file() {
        tracing::warn!(
            path = %path.display(),
            "the trust store path is not a regular file; its mode is left alone"
        );
        return;
    }
    let meta = match file.metadata() {
        Ok(meta) => meta,
        Err(e) => {
            tracing::warn!(
                path = %path.display(),
                error = %e,
                "could not read the trust store's mode"
            );
            return;
        }
    };
    if meta.dev() != at_path.dev() || meta.ino() != at_path.ino() {
        tracing::warn!(
            path = %path.display(),
            "the trust store path changed while it was opened; its mode is left alone"
        );
        return;
    }
    if meta.nlink() != 1 {
        tracing::warn!(
            path = %path.display(),
            links = meta.nlink(),
            "the trust store file has more than one link; its mode is left alone"
        );
        return;
    }
    let mode = meta.permissions().mode() & 0o7777;
    if mode & 0o077 == 0 {
        return;
    }
    let narrowed = mode & !0o077;
    match file.set_permissions(std::fs::Permissions::from_mode(narrowed)) {
        Ok(()) => tracing::warn!(
            path = %path.display(),
            mode = format_args!("{mode:o}"),
            narrowed = format_args!("{narrowed:o}"),
            "the trust store was accessible to other users; cleared its group and other bits"
        ),
        Err(e) => tracing::warn!(
            path = %path.display(),
            mode = format_args!("{mode:o}"),
            error = %e,
            "the trust store is accessible to other users and its group and other bits \
             could not be cleared"
        ),
    }
}

/// The file-system steps [`replace_durably`] runs, one method per step, so a
/// test can record their order. [`StdSaveFs`] is the production implementation.
trait SaveFs {
    /// Create `path` for writing; it must not exist.
    fn create(&mut self, path: &Path) -> std::io::Result<std::fs::File>;
    /// Write all of `bytes` to `file`, which was created at `path`.
    fn write(&mut self, file: &mut std::fs::File, path: &Path, bytes: &[u8])
        -> std::io::Result<()>;
    /// Flush `file` (created at `path`) and its metadata to stable storage.
    fn sync_file(&mut self, file: &std::fs::File, path: &Path) -> std::io::Result<()>;
    /// Rename `from` over `to`.
    fn rename(&mut self, from: &Path, to: &Path) -> std::io::Result<()>;
    /// Flush the directory `dir`, so a rename into it is on stable storage.
    fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()>;
}

/// [`SaveFs`] over `std::fs`.
struct StdSaveFs;

impl SaveFs for StdSaveFs {
    /// An exclusive create (`O_CREAT|O_EXCL`): it fails with `AlreadyExists` on
    /// any existing path, a symbolic link included, without following it (POSIX
    /// `open()` specifies `EEXIST` when `O_CREAT|O_EXCL` names a symbolic link,
    /// dangling or not; Linux and macOS do so). On Unix the file is created with
    /// mode 0600.
    fn create(&mut self, path: &Path) -> std::io::Result<std::fs::File> {
        let mut opts = std::fs::OpenOptions::new();
        opts.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt as _;
            opts.mode(0o600);
        }
        opts.open(path)
    }
    fn write(
        &mut self,
        file: &mut std::fs::File,
        _path: &Path,
        bytes: &[u8],
    ) -> std::io::Result<()> {
        std::io::Write::write_all(file, bytes)
    }
    fn sync_file(&mut self, file: &std::fs::File, _path: &Path) -> std::io::Result<()> {
        file.sync_all()
    }
    fn rename(&mut self, from: &Path, to: &Path) -> std::io::Result<()> {
        std::fs::rename(from, to)
    }
    /// Unix opens the directory and fsyncs it. Off Unix this is a no-op.
    fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()> {
        #[cfg(unix)]
        {
            std::fs::File::open(dir)?.sync_all()
        }
        #[cfg(not(unix))]
        {
            let _ = dir;
            Ok(())
        }
    }
}

/// Replace `path` with `bytes` so that a power loss at any point leaves either
/// the old file or all of the new one: write `tmp`, fsync it, rename it over
/// `path`, then fsync the directory holding `path` (the current directory for a
/// bare file name). Without the first fsync the rename can reach the disk
/// before the data does; without the second the rename itself can be lost.
///
/// A failure up to and including the rename is returned with `path` unchanged;
/// `tmp` is removed when this call created it. The rename is the commit point: a
/// failed directory fsync after it is logged at `warn` and the replace returns
/// `Ok`, because the new file is in place and a power loss can only bring back the
/// old one.
fn replace_durably(
    fs: &mut impl SaveFs,
    tmp: &Path,
    path: &Path,
    bytes: &[u8],
) -> std::io::Result<()> {
    // A failed create made nothing, so whatever is at `tmp` is not this save's
    // and is left alone.
    let mut file = fs.create(tmp)?;
    let staged = fs
        .write(&mut file, tmp, bytes)
        .and_then(|()| fs.sync_file(&file, tmp));
    drop(file);
    if let Err(e) = staged.and_then(|()| fs.rename(tmp, path)) {
        let _ = std::fs::remove_file(tmp);
        return Err(e);
    }
    let dir = match path.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir,
        _ => Path::new("."),
    };
    if let Err(e) = fs.sync_dir(dir) {
        tracing::warn!(
            dir = %dir.display(),
            error = %e,
            "the trust store's directory could not be flushed after the rename"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One recorded [`SaveFs`] step.
    #[derive(Debug, Clone, PartialEq, Eq)]
    enum Step {
        Create(PathBuf),
        Write(PathBuf),
        SyncFile(PathBuf),
        Rename(PathBuf, PathBuf),
        SyncDir(PathBuf),
    }

    /// Records each step, then performs it through [`StdSaveFs`].
    #[derive(Default)]
    struct Recorder(Vec<Step>);

    impl SaveFs for Recorder {
        fn create(&mut self, path: &Path) -> std::io::Result<std::fs::File> {
            self.0.push(Step::Create(path.to_path_buf()));
            StdSaveFs.create(path)
        }
        fn write(
            &mut self,
            file: &mut std::fs::File,
            path: &Path,
            bytes: &[u8],
        ) -> std::io::Result<()> {
            self.0.push(Step::Write(path.to_path_buf()));
            StdSaveFs.write(file, path, bytes)
        }
        fn sync_file(&mut self, file: &std::fs::File, path: &Path) -> std::io::Result<()> {
            self.0.push(Step::SyncFile(path.to_path_buf()));
            StdSaveFs.sync_file(file, path)
        }
        fn rename(&mut self, from: &Path, to: &Path) -> std::io::Result<()> {
            self.0
                .push(Step::Rename(from.to_path_buf(), to.to_path_buf()));
            StdSaveFs.rename(from, to)
        }
        fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()> {
            self.0.push(Step::SyncDir(dir.to_path_buf()));
            StdSaveFs.sync_dir(dir)
        }
    }

    #[test]
    fn a_durable_replace_syncs_the_file_before_the_rename_and_the_directory_after() {
        let tmp_dir = tempfile::tempdir().unwrap();
        let path = tmp_dir.path().join("trust.store");
        let tmp = unique_tmp_path(&path);
        std::fs::write(&path, b"old").unwrap();
        let mut fs = Recorder::default();
        replace_durably(&mut fs, &tmp, &path, b"new bytes").unwrap();
        assert_eq!(
            fs.0,
            vec![
                Step::Create(tmp.clone()),
                Step::Write(tmp.clone()),
                Step::SyncFile(tmp.clone()),
                Step::Rename(tmp.clone(), path.clone()),
                Step::SyncDir(tmp_dir.path().to_path_buf()),
            ]
        );
        assert_eq!(std::fs::read(&path).unwrap(), b"new bytes");
        assert!(!tmp.exists());
    }

    #[test]
    fn a_bare_file_name_syncs_the_current_directory() {
        /// Records only, so the test touches nothing on disk.
        #[derive(Default)]
        struct DirOnly(Vec<PathBuf>);
        impl SaveFs for DirOnly {
            fn create(&mut self, _: &Path) -> std::io::Result<std::fs::File> {
                tempfile::tempfile()
            }
            fn write(&mut self, _: &mut std::fs::File, _: &Path, _: &[u8]) -> std::io::Result<()> {
                Ok(())
            }
            fn sync_file(&mut self, _: &std::fs::File, _: &Path) -> std::io::Result<()> {
                Ok(())
            }
            fn rename(&mut self, _: &Path, _: &Path) -> std::io::Result<()> {
                Ok(())
            }
            fn sync_dir(&mut self, dir: &Path) -> std::io::Result<()> {
                self.0.push(dir.to_path_buf());
                Ok(())
            }
        }
        let mut fs = DirOnly::default();
        replace_durably(&mut fs, Path::new("s.tmp"), Path::new("s"), b"x").unwrap();
        assert_eq!(fs.0, vec![PathBuf::from(".")]);
    }

    #[test]
    fn a_failed_directory_sync_after_the_rename_still_saves() {
        /// Delegates to [`StdSaveFs`] but refuses the directory sync.
        struct DirSyncFails;
        impl SaveFs for DirSyncFails {
            fn create(&mut self, p: &Path) -> std::io::Result<std::fs::File> {
                StdSaveFs.create(p)
            }
            fn write(&mut self, f: &mut std::fs::File, p: &Path, b: &[u8]) -> std::io::Result<()> {
                StdSaveFs.write(f, p, b)
            }
            fn sync_file(&mut self, f: &std::fs::File, p: &Path) -> std::io::Result<()> {
                StdSaveFs.sync_file(f, p)
            }
            fn rename(&mut self, a: &Path, b: &Path) -> std::io::Result<()> {
                StdSaveFs.rename(a, b)
            }
            fn sync_dir(&mut self, _: &Path) -> std::io::Result<()> {
                Err(std::io::Error::other("injected"))
            }
        }
        let tmp_dir = tempfile::tempdir().unwrap();
        let path = tmp_dir.path().join("trust.store");
        replace_durably(&mut DirSyncFails, &unique_tmp_path(&path), &path, b"new").unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), b"new");
    }

    #[test]
    fn a_failed_file_sync_keeps_the_old_store_and_removes_the_tmp() {
        /// Delegates to [`StdSaveFs`] but refuses the file sync.
        struct FileSyncFails;
        impl SaveFs for FileSyncFails {
            fn create(&mut self, p: &Path) -> std::io::Result<std::fs::File> {
                StdSaveFs.create(p)
            }
            fn write(&mut self, f: &mut std::fs::File, p: &Path, b: &[u8]) -> std::io::Result<()> {
                StdSaveFs.write(f, p, b)
            }
            fn sync_file(&mut self, _: &std::fs::File, _: &Path) -> std::io::Result<()> {
                Err(std::io::Error::other("injected"))
            }
            fn rename(&mut self, a: &Path, b: &Path) -> std::io::Result<()> {
                StdSaveFs.rename(a, b)
            }
            fn sync_dir(&mut self, d: &Path) -> std::io::Result<()> {
                StdSaveFs.sync_dir(d)
            }
        }
        let tmp_dir = tempfile::tempdir().unwrap();
        let path = tmp_dir.path().join("trust.store");
        let tmp = unique_tmp_path(&path);
        std::fs::write(&path, b"old").unwrap();
        let err = replace_durably(&mut FileSyncFails, &tmp, &path, b"new").unwrap_err();
        assert_eq!(err.to_string(), "injected");
        assert_eq!(std::fs::read(&path).unwrap(), b"old");
        assert!(!tmp.exists(), "the tmp is removed");
    }

    #[test]
    fn save_runs_the_durable_replace_on_the_store_path() {
        let tmp_dir = tempfile::tempdir().unwrap();
        let path = tmp_dir.path().join("trust.store");
        let roots = RootSet::new(vec![PublicKey([1; 32]), PublicKey([2; 32])], 2).unwrap();
        let store = TrustStore::provision(RobotId([0xB0; 32]), PublicKey([3; 32]), roots, b"c", 0)
            .unwrap()
            .with_path(&path);
        let mut fs = Recorder::default();
        store
            .save_with(b"0123456789abcdef0123456789abcdef", &mut fs)
            .unwrap();
        let Some(Step::Create(tmp)) = fs.0.first().cloned() else {
            panic!("the first step is the create: {:?}", fs.0);
        };
        assert_tmp_name(&tmp, &path);
        assert_eq!(
            fs.0,
            vec![
                Step::Create(tmp.clone()),
                Step::Write(tmp.clone()),
                Step::SyncFile(tmp.clone()),
                Step::Rename(tmp, path.clone()),
                Step::SyncDir(tmp_dir.path().to_path_buf()),
            ]
        );
        TrustStore::load(&path, KEY).expect("the saved store loads");
    }

    const KEY: &[u8] = b"0123456789abcdef0123456789abcdef";

    fn test_store(path: &Path) -> TrustStore {
        let roots = RootSet::new(vec![PublicKey([1; 32]), PublicKey([2; 32])], 2).unwrap();
        TrustStore::provision(RobotId([0xB0; 32]), PublicKey([3; 32]), roots, b"c", 0)
            .unwrap()
            .with_path(path)
    }

    /// `tmp` is `.<store name>.<16 lowercase hex>.tmp` beside `store`.
    fn assert_tmp_name(tmp: &Path, store: &Path) {
        assert_eq!(tmp.parent(), store.parent(), "{}", tmp.display());
        let name = tmp.file_name().unwrap().to_str().unwrap();
        let prefix = format!(".{}.", store.file_name().unwrap().to_str().unwrap());
        let hex = name
            .strip_prefix(&prefix)
            .and_then(|rest| rest.strip_suffix(".tmp"))
            .unwrap_or_else(|| panic!("unexpected tmp name {name}"));
        assert_eq!(hex.len(), 16, "{name}");
        assert!(
            hex.bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)),
            "{name}"
        );
    }

    #[test]
    fn each_save_gets_its_own_temporary_name() {
        let path = Path::new("/r/trust.store");
        let a = unique_tmp_path(path);
        let b = unique_tmp_path(path);
        assert_tmp_name(&a, path);
        assert_tmp_name(&b, path);
        assert_ne!(a, b);
    }

    #[cfg(unix)]
    fn mode_of(path: &Path) -> u32 {
        use std::os::unix::fs::PermissionsExt as _;
        std::fs::metadata(path).unwrap().permissions().mode() & 0o7777
    }

    #[cfg(unix)]
    #[test]
    fn a_save_over_a_0644_store_leaves_it_0600() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        std::fs::write(&path, b"old").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
        // Control: a file created without an explicit mode keeps group or other
        // bits under this umask, so the 0600 below comes from the save.
        let probe = dir.path().join("probe");
        std::fs::File::create(&probe).unwrap();
        assert_ne!(
            mode_of(&probe) & 0o077,
            0,
            "the umask clears group and other bits, which would hide a dropped mode"
        );
        std::fs::remove_file(&probe).unwrap();
        test_store(&path).save(KEY).unwrap();
        assert_eq!(mode_of(&path), 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn loading_a_0644_store_narrows_it_to_0600() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        test_store(&path).save(KEY).unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
        TrustStore::load(&path, KEY).expect("a narrowed store still loads");
        assert_eq!(mode_of(&path), 0o600);
    }

    #[cfg(unix)]
    #[test]
    fn a_planted_symlink_at_a_temporary_name_is_not_followed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        let victim = dir.path().join("victim");
        std::fs::write(&victim, b"keep").unwrap();
        // The fixed name saves used before, and a name shaped like a fresh one.
        let fixed = dir.path().join("trust.store.tmp");
        let shaped = dir.path().join(".trust.store.0123456789abcdef.tmp");
        for link in [&fixed, &shaped] {
            std::os::unix::fs::symlink(&victim, link).unwrap();
            let err = StdSaveFs
                .create(link)
                .expect_err("an existing path is refused");
            assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists, "{err}");
            let err = replace_durably(&mut StdSaveFs, link, &path, b"new").unwrap_err();
            assert_eq!(err.kind(), std::io::ErrorKind::AlreadyExists, "{err}");
            assert!(
                link.symlink_metadata().is_ok(),
                "the planted path is left in place"
            );
        }
        assert_eq!(
            std::fs::read(&victim).unwrap(),
            b"keep",
            "the target is unchanged"
        );
        assert!(!path.exists(), "nothing was renamed into place");
        // A save beside the planted links goes through its own fresh name.
        test_store(&path).save(KEY).unwrap();
        TrustStore::load(&path, KEY).expect("the saved store loads");
        assert_eq!(std::fs::read(&victim).unwrap(), b"keep");
    }

    #[cfg(unix)]
    #[test]
    fn a_symlinked_store_is_read_through_without_changing_its_target() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let target = dir.path().join("elsewhere.store");
        test_store(&target).save(KEY).unwrap();
        std::fs::set_permissions(&target, std::fs::Permissions::from_mode(0o644)).unwrap();
        let link = dir.path().join("trust.store");
        std::os::unix::fs::symlink(&target, &link).unwrap();
        TrustStore::load(&link, KEY).expect("a symlinked store still loads");
        assert_eq!(mode_of(&target), 0o644, "the link's target keeps its mode");
        assert!(link.symlink_metadata().unwrap().file_type().is_symlink());
    }

    #[cfg(unix)]
    #[test]
    fn a_store_with_another_hard_link_is_left_alone() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let first = dir.path().join("first.store");
        test_store(&first).save(KEY).unwrap();
        std::fs::set_permissions(&first, std::fs::Permissions::from_mode(0o644)).unwrap();
        let path = dir.path().join("trust.store");
        std::fs::hard_link(&first, &path).unwrap();
        TrustStore::load(&path, KEY).expect("a hard-linked store still loads");
        assert_eq!(
            mode_of(&first),
            0o644,
            "a file with two links keeps its mode"
        );
    }

    #[cfg(unix)]
    /// A FIFO at `path` with `mode`, made with the system `mkfifo`, and a handle
    /// on it opened read-write so the open does not wait for a peer.
    fn fifo_with_mode(path: &Path, mode: u32) -> std::fs::File {
        let made = std::process::Command::new("mkfifo")
            .arg("-m")
            .arg(format!("{mode:o}"))
            .arg(path)
            .status()
            .expect("run mkfifo");
        assert!(made.success(), "mkfifo failed: {made}");
        std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .unwrap()
    }

    #[cfg(unix)]
    #[test]
    fn a_fifo_at_the_store_path_is_not_changed() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        let file = fifo_with_mode(&path, 0o644);
        let at_path = std::fs::symlink_metadata(&path).unwrap();
        narrow_to_owner(&file, &path, Some(&at_path));
        assert_eq!(mode_of(&path), 0o644, "a non-regular file keeps its mode");
    }

    #[cfg(unix)]
    #[test]
    fn a_directory_at_the_store_path_is_not_changed() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let sub = dir.path().join("trust.store");
        std::fs::create_dir(&sub).unwrap();
        std::fs::set_permissions(&sub, std::fs::Permissions::from_mode(0o755)).unwrap();
        assert!(
            TrustStore::load(&sub, KEY).is_err(),
            "a directory is not a store"
        );
        assert_eq!(mode_of(&sub), 0o755);
    }

    #[cfg(unix)]
    #[test]
    fn a_handle_that_is_not_the_file_at_the_path_is_left_alone() {
        use std::os::unix::fs::PermissionsExt as _;
        let dir = tempfile::tempdir().unwrap();
        let opened = dir.path().join("opened");
        let other = dir.path().join("other");
        std::fs::write(&opened, b"x").unwrap();
        std::fs::write(&other, b"y").unwrap();
        std::fs::set_permissions(&opened, std::fs::Permissions::from_mode(0o644)).unwrap();
        let file = std::fs::File::open(&opened).unwrap();
        let other_meta = std::fs::symlink_metadata(&other).unwrap();
        narrow_to_owner(&file, &opened, Some(&other_meta));
        assert_eq!(mode_of(&opened), 0o644, "a different inode is not changed");
        let own_meta = std::fs::symlink_metadata(&opened).unwrap();
        narrow_to_owner(&file, &opened, Some(&own_meta));
        assert_eq!(mode_of(&opened), 0o600, "the same inode is narrowed");
    }

    #[test]
    fn concurrent_saves_both_succeed_and_leave_a_valid_store() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        let store = test_store(&path);
        let threads: Vec<_> = (0..2)
            .map(|_| {
                let store = store.clone();
                std::thread::spawn(move || {
                    for _ in 0..10 {
                        store.save(KEY).expect("a concurrent save succeeds");
                    }
                })
            })
            .collect();
        for t in threads {
            t.join().unwrap();
        }
        TrustStore::load(&path, KEY).expect("the final store has a valid MAC");
        let leftovers: Vec<_> = std::fs::read_dir(dir.path())
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .filter(|n| n != "trust.store")
            .collect();
        assert!(
            leftovers.is_empty(),
            "no temporary file is left: {leftovers:?}"
        );
    }
}

#[cfg(test)]
mod owner_certificate_state_tests {
    use super::*;
    use crate::format::{DeviceCert, IntermediateCert, PrincipalKind, Validity, FORMAT_VERSION};
    use ed25519_dalek::SigningKey;

    const OWNER: AccountId = AccountId([4; 32]);

    fn key(seed: u8) -> PublicKey {
        PublicKey(
            SigningKey::from_bytes(&[seed; 32])
                .verifying_key()
                .to_bytes(),
        )
    }

    /// A roots-verified chain for the owner account's device `seed`.
    fn proof(seed: u8, principal_kind: PrincipalKind) -> OwnerCertificatePresentationWire {
        proof_until(seed, principal_kind, 100)
    }

    /// [`proof`] with the device certificate valid until `not_after_ns`.
    fn proof_until(
        seed: u8,
        principal_kind: PrincipalKind,
        not_after_ns: u64,
    ) -> OwnerCertificatePresentationWire {
        let root = SigningKey::from_bytes(&[1; 32]);
        let issuer = SigningKey::from_bytes(&[2; 32]);
        let issuer_key = PublicKey(issuer.verifying_key().to_bytes());
        let validity = Validity {
            not_before_ns: 0,
            not_after_ns: 100,
        };
        OwnerCertificatePresentationWire::new(
            IntermediateCert {
                version: FORMAT_VERSION,
                intermediate_key: issuer_key,
                validity,
                issued_at_ns: 0,
                max_scope: Scope::OWNER_FULL,
            }
            .sign_by_roots(&[&root]),
            DeviceCert {
                version: FORMAT_VERSION,
                device_key: key(seed),
                account: OWNER,
                principal_kind,
                scope: Scope::OWNER_FULL,
                validity: Validity {
                    not_before_ns: 0,
                    not_after_ns,
                },
                issued_at_ns: 0,
                issuer_key,
            }
            .sign(&issuer),
        )
    }

    fn fixture() -> (TrustStore, OwnerCertificatePresentationWire) {
        let root = SigningKey::from_bytes(&[1; 32]);
        let mut store = TrustStore::provision(
            RobotId([5; 32]),
            key(3),
            RootSet::new(vec![PublicKey(root.verifying_key().to_bytes())], 1).unwrap(),
            b"unit recovery",
            1,
        )
        .unwrap();
        store
            .claim(OWNER, b"unit recovery", PrincipalKind::Human, 1)
            .unwrap();
        (store, proof(3, PrincipalKind::Human))
    }

    #[test]
    fn owner_certificate_cannot_clear_a_sticky_revoked_owner_row() {
        let (mut store, proof) = fixture();
        // owner_revoke intentionally prevents self-lockout. A persisted revoked
        // owner row must still fail closed if imported or loaded from older state.
        store.inner.access_rows[0].revoked = true;
        assert!(matches!(
            store.verify_owner_certificate(&proof, &proof.device_cert.cert.device_key, 2),
            Err(PairingError::RevokedByOwner)
        ));
        assert_eq!(store.access_rows().len(), 1);
        assert!(store.access_rows()[0].revoked);
        assert_eq!(store.high_water_ns(), 1);
    }

    #[test]
    fn owner_certificate_never_synthesizes_a_missing_owner_row() {
        let (mut store, proof) = fixture();
        store.inner.access_rows.clear();
        assert!(matches!(
            store.verify_owner_certificate(&proof, &proof.device_cert.cert.device_key, 2),
            Err(PairingError::StoreCorrupt(
                "claimed owner has no access row"
            ))
        ));
        assert!(store.access_rows().is_empty());
        assert_eq!(store.high_water_ns(), 1);
    }

    #[test]
    fn owner_certificate_adds_a_device_row_and_never_rewrites_the_owner_row() {
        let (mut store, proof) = fixture();
        store.inner.access_rows[0].expires_at_ns = Some(3);
        let owner_row = store.access_rows()[0].clone();
        store.verify_owner_certificate(&proof, &key(3), 2).unwrap();
        assert_eq!(store.access_rows()[0], owner_row);
        assert_eq!(
            store.access_rows()[1],
            AccessRow {
                account: OWNER,
                device: Some(key(3)),
                scope: Scope::OWNER_FULL,
                principal_kind: PrincipalKind::Human,
                source: PairingSource::CertificatePresentation,
                name: None,
                added_at_ns: 2,
                expires_at_ns: Some(3),
                revoked: false,
            }
        );
        assert!(matches!(
            store.verify_owner_certificate(&proof, &key(3), 3),
            Err(PairingError::OwnerAccessExpired)
        ));
        assert_eq!(store.access_rows().len(), 2);
        assert_eq!(store.high_water_ns(), 2);
    }

    #[test]
    fn a_cloud_certificate_gets_its_own_cloud_row_beside_the_owner_rows() {
        let (mut store, desk) = fixture();
        store.verify_owner_certificate(&desk, &key(3), 2).unwrap();
        let before: Vec<AccessRow> = store.access_rows().to_vec();
        let cloud = proof(9, PrincipalKind::Cloud);
        store.verify_owner_certificate(&cloud, &key(9), 3).unwrap();
        // The owner row and the desk row are byte-for-byte unchanged.
        assert_eq!(&store.access_rows()[..2], before.as_slice());
        let row = store.is_allowed(&OWNER, &key(9), 3).unwrap();
        assert_eq!(row.device, Some(key(9)));
        assert_eq!(row.principal_kind, PrincipalKind::Cloud);
        assert_eq!(
            store.is_allowed(&OWNER, &key(3), 3).unwrap().principal_kind,
            PrincipalKind::Human
        );
        // A device with no row of its own falls back to the owner row.
        assert_eq!(store.is_allowed(&OWNER, &key(7), 3).unwrap().device, None);
    }

    #[test]
    fn the_switch_off_refuses_cloud_rows_and_new_cloud_certificates_only() {
        let (mut store, desk) = fixture();
        store.verify_owner_certificate(&desk, &key(3), 2).unwrap();
        store
            .verify_owner_certificate(&proof(9, PrincipalKind::Cloud), &key(9), 3)
            .unwrap();
        assert!(store.allow_cloud());
        let cloud_row = store.is_allowed(&OWNER, &key(9), 3).unwrap().clone();
        assert!(store.cloud_allowed(&cloud_row));
        assert!(matches!(
            store.set_allow_cloud(&AccountId([6; 32]), false),
            Err(PairingError::NotOwner)
        ));
        store.set_allow_cloud(&OWNER, false).unwrap();
        assert!(!store.cloud_allowed(&cloud_row));
        let desk_row = store.is_allowed(&OWNER, &key(3), 3).unwrap().clone();
        assert!(store.cloud_allowed(&desk_row));
        let rows = store.access_rows().to_vec();
        assert!(matches!(
            store.verify_owner_certificate(&proof(8, PrincipalKind::Cloud), &key(8), 4),
            Err(PairingError::CloudAccessDisabled)
        ));
        assert_eq!(store.access_rows(), rows.as_slice());
        store.set_allow_cloud(&OWNER, true).unwrap();
        assert!(store.cloud_allowed(&cloud_row));
    }

    #[test]
    fn a_revoked_device_row_never_falls_back_and_account_revoke_covers_every_row() {
        let (mut store, desk) = fixture();
        let guest = AccountId([6; 32]);
        store.verify_owner_certificate(&desk, &key(3), 2).unwrap();
        store.inner.access_rows.push(AccessRow {
            account: guest,
            device: Some(key(10)),
            scope: Scope::CODE_PAIR_DEFAULT,
            principal_kind: PrincipalKind::Human,
            source: PairingSource::CodePaired,
            name: None,
            added_at_ns: 2,
            expires_at_ns: None,
            revoked: false,
        });
        let guest_row = store.inner.access_rows[2].clone();
        store.inner.access_rows.push(AccessRow {
            device: Some(key(11)),
            ..guest_row
        });
        store.inner.access_rows[1].revoked = true;
        assert!(store.is_allowed(&OWNER, &key(3), 3).is_none());
        assert!(store.is_allowed(&OWNER, &key(7), 3).is_some());
        store.owner_revoke(&OWNER, &guest).unwrap();
        assert!(store.is_allowed(&guest, &key(10), 3).is_none());
        assert!(store.is_allowed(&guest, &key(11), 3).is_none());
        store.owner_readmit(&OWNER, &guest).unwrap();
        assert!(store.is_allowed(&guest, &key(10), 3).is_some());
        assert!(store.is_allowed(&guest, &key(11), 3).is_some());
    }

    #[test]
    fn strong_chain_and_owner_grant_paths_write_no_cloud_row_while_the_switch_is_off() {
        let (mut store, _) = fixture();
        store.set_allow_cloud(&OWNER, false).unwrap();
        let rows = store.access_rows().to_vec();
        // `verify_and_establish` and `establish_by_owner_grant` both persist
        // through `establish_pairing`.
        for source in [PairingSource::StrongChain, PairingSource::OwnerSignedGrant] {
            let cloud = VerifiedPairing::new(
                AccountId([6; 32]),
                Scope::CODE_PAIR_DEFAULT,
                PrincipalKind::Cloud,
                key(12),
                u64::MAX,
            );
            assert!(matches!(
                store.establish_pairing(&cloud, source, None, 2, None),
                Err(PairingError::CloudAccessDisabled)
            ));
            assert_eq!(store.access_rows(), rows.as_slice());
        }
        let human = VerifiedPairing::new(
            AccountId([6; 32]),
            Scope::CODE_PAIR_DEFAULT,
            PrincipalKind::Human,
            key(13),
            u64::MAX,
        );
        store
            .establish_pairing(&human, PairingSource::StrongChain, None, 2, None)
            .unwrap();
        assert_eq!(store.access_rows().len(), rows.len() + 1);
    }

    #[test]
    fn an_expired_device_row_denies_that_device_while_the_account_wide_row_admits_others() {
        let (mut store, desk) = fixture();
        store.verify_owner_certificate(&desk, &key(3), 2).unwrap();
        store.inner.access_rows[1].expires_at_ns = Some(5);
        assert!(store.is_allowed(&OWNER, &key(3), 4).is_some());
        assert!(store.is_allowed(&OWNER, &key(3), 5).is_none());
        assert!(store.is_allowed(&OWNER, &key(3), 6).is_none());
        assert_eq!(store.is_allowed(&OWNER, &key(7), 6).unwrap().device, None);
        // An expired account-wide row denies a device with no row of its own.
        store.inner.access_rows[0].expires_at_ns = Some(6);
        assert!(store.is_allowed(&OWNER, &key(7), 6).is_none());
        assert!(store.is_allowed(&OWNER, &key(7), 5).is_some());
    }

    #[test]
    fn a_cloud_row_lapses_with_its_certificate_and_a_renewal_extends_it() {
        let (mut store, _) = fixture();
        store
            .verify_owner_certificate(&proof_until(9, PrincipalKind::Cloud, 50), &key(9), 2)
            .unwrap();
        assert_eq!(store.access_rows()[1].expires_at_ns, Some(50));
        assert!(store.is_allowed(&OWNER, &key(9), 49).is_some());
        assert!(store.is_allowed(&OWNER, &key(9), 50).is_none());
        // A certificate whose not_after has passed is refused and stores nothing.
        let rows = store.access_rows().to_vec();
        assert!(store
            .verify_owner_certificate(&proof_until(9, PrincipalKind::Cloud, 50), &key(9), 60)
            .is_err());
        assert_eq!(store.access_rows(), rows.as_slice());
        // A renewed certificate extends the row.
        store
            .verify_owner_certificate(&proof_until(9, PrincipalKind::Cloud, 90), &key(9), 61)
            .unwrap();
        assert_eq!(store.access_rows()[1].expires_at_ns, Some(90));
        assert!(store.is_allowed(&OWNER, &key(9), 70).is_some());
        // A human certificate's row keeps the owner row's expiry (none).
        store
            .verify_owner_certificate(&proof_until(3, PrincipalKind::Human, 80), &key(3), 62)
            .unwrap();
        assert_eq!(store.access_rows()[2].expires_at_ns, None);
        assert!(store.is_allowed(&OWNER, &key(3), 95).is_some());
    }

    #[test]
    fn a_cloud_row_takes_the_earlier_of_the_certificate_and_grant_expiry() {
        let (mut store, _) = fixture();
        let cloud = |seed: u8| {
            VerifiedPairing::new(
                AccountId([6; 32]),
                Scope::CODE_PAIR_DEFAULT,
                PrincipalKind::Cloud,
                key(seed),
                40,
            )
        };
        let source = PairingSource::OwnerSignedGrant;
        store
            .establish_pairing(&cloud(20), source, None, 2, Some(30))
            .unwrap();
        store
            .establish_pairing(&cloud(21), source, None, 2, Some(90))
            .unwrap();
        store
            .establish_pairing(&cloud(22), source, None, 2, None)
            .unwrap();
        let human = VerifiedPairing::new(
            AccountId([6; 32]),
            Scope::CODE_PAIR_DEFAULT,
            PrincipalKind::Human,
            key(23),
            40,
        );
        store
            .establish_pairing(&human, source, None, 2, Some(90))
            .unwrap();
        let expiries: Vec<Option<u64>> = store.access_rows()[1..]
            .iter()
            .map(|r| r.expires_at_ns)
            .collect();
        assert_eq!(expiries, vec![Some(30), Some(40), Some(40), Some(90)]);
    }

    /// The v3 body as a v3 writer laid it out (field order is the wire).
    #[derive(Serialize)]
    struct V3Body {
        store_version: u16,
        robot: RobotId,
        robot_transport_key: PublicKey,
        root_set: RootSet,
        ownership: OwnershipState,
        chassis_secret_hash: [u8; 32],
        access_rows: Vec<V3Row>,
        current_epoch: u64,
        revoked_accounts: Vec<AccountId>,
        revoked_devices: Vec<PublicKey>,
        high_water_ns: u64,
    }

    #[derive(Serialize)]
    struct V3Row {
        account: AccountId,
        scope: Scope,
        principal_kind: PrincipalKind,
        source: PairingSource,
        name: Option<String>,
        added_at_ns: u64,
        expires_at_ns: Option<u64>,
        revoked: bool,
    }

    fn v3_file(mac_key: &[u8], version_in_body: u16) -> Vec<u8> {
        let root = SigningKey::from_bytes(&[1; 32]);
        let row = |account: [u8; 32], scope, source, revoked| V3Row {
            account: AccountId(account),
            scope,
            principal_kind: PrincipalKind::Human,
            source,
            name: Some("v3".into()),
            added_at_ns: 7,
            expires_at_ns: None,
            revoked,
        };
        let body = postcard::to_stdvec(&V3Body {
            store_version: version_in_body,
            robot: RobotId([5; 32]),
            robot_transport_key: key(3),
            root_set: RootSet::new(vec![PublicKey(root.verifying_key().to_bytes())], 1).unwrap(),
            ownership: OwnershipState::Claimed(OWNER),
            chassis_secret_hash: [0x5a; 32],
            access_rows: vec![
                row([4; 32], Scope::OWNER_FULL, PairingSource::Claim, false),
                row(
                    [6; 32],
                    Scope::CODE_PAIR_DEFAULT,
                    PairingSource::CodePaired,
                    true,
                ),
            ],
            current_epoch: 2,
            revoked_accounts: vec![AccountId([8; 32])],
            revoked_devices: vec![key(12)],
            high_water_ns: 40,
        })
        .unwrap();
        let mut framed = Vec::new();
        framed.extend_from_slice(b"CERPAIR\x01");
        framed.extend_from_slice(&3u16.to_le_bytes());
        framed.extend_from_slice(&(body.len() as u32).to_le_bytes());
        framed.extend_from_slice(&body);
        let tag = crypto::hmac_sha256(mac_key, &framed);
        framed.extend_from_slice(&tag);
        framed
    }

    #[test]
    fn a_v3_store_migrates_every_row_account_wide_in_order_and_saves_as_v4() {
        let mac = [0x33; 32];
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        std::fs::write(&path, v3_file(&mac, 3)).unwrap();
        let store = TrustStore::load(&path, &mac).unwrap();
        assert_eq!(store.owner(), Some(OWNER));
        assert_eq!(store.current_epoch(), 2);
        assert_eq!(store.high_water_ns(), 40);
        assert_eq!(store.revoked_devices(), &[key(12)]);
        assert!(store.allow_cloud());
        let rows: Vec<(AccountId, Option<PublicKey>, Scope, bool)> = store
            .access_rows()
            .iter()
            .map(|r| (r.account, r.device, r.scope, r.revoked))
            .collect();
        assert_eq!(
            rows,
            vec![
                (OWNER, None, Scope::OWNER_FULL, false),
                (AccountId([6; 32]), None, Scope::CODE_PAIR_DEFAULT, true),
            ]
        );
        store.save(&mac).unwrap();
        let saved = std::fs::read(&path).unwrap();
        assert_eq!(&saved[8..10], &4u16.to_le_bytes());
        let reloaded = TrustStore::load(&path, &mac).unwrap();
        assert_eq!(reloaded.access_rows(), store.access_rows());
        // The migration is deterministic: two loads of the same v3 file agree.
        std::fs::write(&path, v3_file(&mac, 3)).unwrap();
        assert_eq!(
            TrustStore::load(&path, &mac).unwrap().encode(&mac).unwrap(),
            store.encode(&mac).unwrap()
        );
    }

    #[test]
    fn a_v3_store_with_a_bad_mac_or_mismatched_body_version_is_refused() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trust.store");
        std::fs::write(&path, v3_file(&[0x33; 32], 3)).unwrap();
        assert!(matches!(
            TrustStore::load(&path, &[0x34; 32]),
            Err(PairingError::TamperDetected)
        ));
        std::fs::write(&path, v3_file(&[0x33; 32], 4)).unwrap();
        assert!(matches!(
            TrustStore::load(&path, &[0x33; 32]),
            Err(PairingError::StoreCorrupt("format version mismatch"))
        ));
    }
}
