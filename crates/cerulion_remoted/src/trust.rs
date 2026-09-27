// SPDX-License-Identifier: AGPL-3.0-only
//! [`SharedTrust`] — the LIVE, shared trust state the accept gate reads and the
//! pairing bootstrap verbs mutate.
//!
//! The [`crate::authorizer::PairingAuthorizer`] does not read
//! an immutable boot-time snapshot of the trust store + side-map. This module makes
//! that state LIVE: a `claim` / `pair` / `code-pair` bootstrap verb mutates the
//! store + the `device_key → account` side-map, and the accept gate must see the
//! change WITHOUT a daemon restart (an always-on robot that is claimed — or a
//! guest that pairs — takes effect immediately). Both the authorizer (read side)
//! and the pairing verbs (write side) therefore share ONE
//! `Arc<Mutex<TrustInner>>` behind this handle.
//!
//! ## Persistence (fail-closed, lock-free disk I/O)
//!
//! A mutation is applied to the live store under the lock, then the two MAC'd
//! artifacts (the trust store + the `device_key → account` side-map, both
//! authenticated with the firmware secure-storage MAC key) are snapshotted to
//! OWNED buffers and the lock is RELEASED — so the blocking disk writes never run
//! while holding the mutex the async accept gate reads.
//!
//! **Store-BEFORE-index is the deny-closed ordering.** The two files are written
//! store-first, index-second. A partial failure therefore never leaves a durable
//! key binding without its account row: a store-ok/index-fail leaves a row with NO
//! binding (`snapshot_for_key` → `account_for` = `None` → DENIED — the multi-device
//! ghost is impossible), and a store-fail leaves neither. (The naive index-first
//! order was fail-OPEN: an index-ok/store-fail durably wrote a `key → account`
//! binding whose account row already existed on disk, so the key read `Allowed` on
//! reload despite the pairing verb returning `Err`.)
//!
//! **The live binding is published only AFTER the durable write succeeds**, so on
//! any disk-write failure the key never became `Allowed` — fail-closed by
//! construction, with no rollback and no transient-Allowed window. The store row
//! the mutation wrote lingers in the live store on a persist failure (not easily
//! reversible) but is unreachable without a binding; a reboot reloads the
//! deny-closed disk state. A persist error is surfaced LOUDLY as
//! [`TrustError::Persist`] — a client whose verb returned `Err` never retains
//! access.
//!
//! **Limitation:** the two-file write is not a single
//! transaction, so the orphaned in-memory store row persists until reboot.

use std::sync::{Arc, Mutex};

use cerulion_pairing::format::{
    AccountId, PrincipalKind, PublicKey, Scope, SignedEpoch, SignedIntermediateCert,
};
use cerulion_pairing::pake::CpaceConfirmed;
use cerulion_pairing::verify::{
    EpochOutcome, OwnerCertificatePresentationWire, OwnerGrantPresentation, PairingPresentation,
    TrustStore,
};
use cerulion_pairing::PairingError;

use crate::clock::RemotedClock;
use crate::device_index::DeviceAccountIndex;

/// The three states a device key can be in against the live access list, with
/// the effective scope when allowed. The accept gate branches on this to produce
/// a precise deny reason (an *unpaired* key is distinct from a *revoked/removed*
/// account — same outcome, different signal).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyAccess {
    /// No `device_key → account` binding exists (the key has never paired).
    Unpaired,
    /// A binding exists, but the account is not currently on the access list
    /// (revoked by epoch, owner-revoked, its row removed, OR its owner-signed
    /// grant has passed its expiry).
    NotAllowed,
    /// The DEVICE key itself is revoked by the current access-list epoch,
    /// regardless of its account row. Kept distinct from [`KeyAccess::NotAllowed`]
    /// so the deny reason is precise: this desk was cut, but the account (and its OTHER
    /// devices) may still be allowed. Denied at the accept gate exactly like
    /// `NotAllowed`; the mid-session sweep evicts a live session once it flips.
    DeviceRevoked,
    /// The key maps to an account currently on the access list; the granted scope.
    Allowed(Scope),
}

/// A mutation of the shared trust state failed.
#[derive(Debug, thiserror::Error)]
pub enum TrustError {
    /// The pairing library rejected the operation (a bad chassis secret, a cert
    /// chain that does not verify, a CPace witness mismatch, an already-claimed
    /// robot, …). Surfaced to the caller as a verb error, never fail-open.
    #[error("pairing rejected: {0}")]
    Pairing(#[from] PairingError),

    /// The in-memory mutation succeeded but persisting it to disk failed. Loud so
    /// a lost durable write is never mistaken for a committed pairing.
    #[error("persist failed: {0}")]
    Persist(String),
}

/// The live, mutex-guarded trust state: the MAC'd trust store + the
/// `device_key → account` side-map.
struct TrustInner {
    store: TrustStore,
    index: DeviceAccountIndex,
}

/// A cloneable handle to the LIVE trust state. The authorizer holds one (read
/// side); every pairing bootstrap verb holds one (write side); both observe the
/// same `Arc<Mutex<TrustInner>>`.
#[derive(Clone)]
pub struct SharedTrust {
    inner: Arc<Mutex<TrustInner>>,
    /// Serializes the WHOLE commit critical section (in-memory mutation →
    /// snapshot → disk writes → publish). Two concurrent commits each snapshot the
    /// index with only their OWN unpublished binding; without this a second write
    /// would clobber the first's binding on disk (a lost pairing on reboot). It is
    /// a SEPARATE lock from `inner` (the accept-gate read lock), so commits
    /// serialize across their blocking disk I/O WITHOUT ever holding — or blocking
    /// readers on — the `inner` lock during I/O. Always acquired BEFORE `inner`
    /// (one consistent lock order → no deadlock; readers take only `inner`).
    persist_lock: Arc<Mutex<()>>,
    /// The firmware secure-storage MAC key that authenticates BOTH persisted
    /// artifacts. `Arc<Vec<u8>>` so clones share it; never logged.
    mac_key: Arc<Vec<u8>>,
    /// The robot's OWN trusted clock, read at every ACCEPT-time access check so an
    /// expiring owner-signed grant denies once its bound passes. Reads
    /// use the robot's clock (never a client-supplied time — a peer that could set
    /// it would trivially outlive an expiry); mutations still take their `now_ns`
    /// from the ops verb's `RemotedClock`, so read + write share one trusted source.
    /// Defaults to [`RemotedClock::wall`]; tests inject a [`RemotedClock::fixed`].
    clock: RemotedClock,
    /// Test-only seam: a sleep injected between the commit snapshot and the disk
    /// write to WIDEN the concurrent-commit race window, so the serialization pin
    /// deterministically fails when the `persist_lock` is removed.
    #[cfg(test)]
    race_delay: Option<std::time::Duration>,
}

impl std::fmt::Debug for SharedTrust {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Never print the MAC key or the store internals (chassis-secret hash,
        // keys). The claimed flag + bound-device count is a safe operator signal.
        //
        // TRY_LOCK, never lock: `Debug` must never block/deadlock. `std::sync::Mutex`
        // is non-reentrant, so a `tracing!(?shared)` inside a section already holding
        // the lock would self-deadlock on a blocking `lock()`. On contention (or a
        // poisoned mutex) render a placeholder instead.
        match self.inner.try_lock() {
            Ok(g) => f
                .debug_struct("SharedTrust")
                .field("claimed", &g.store.is_claimed())
                .field("bound_devices", &g.index.len())
                .finish_non_exhaustive(),
            Err(_) => f
                .debug_struct("SharedTrust")
                .field("state", &"<locked>")
                .finish_non_exhaustive(),
        }
    }
}

impl SharedTrust {
    /// Build a shared-trust handle over a loaded store + side-map, authenticated
    /// with `mac_key` for persistence. An EMPTY `mac_key` (and/or a path-less
    /// store) yields a read-only handle whose mutations cannot persist — the shape
    /// used by [`crate::authorizer::PairingAuthorizer::new`] in pure-authz tests
    /// that never mutate.
    pub fn new(store: TrustStore, index: DeviceAccountIndex, mac_key: Vec<u8>) -> Self {
        // The accept gate reads the robot's WALL clock for expiry checks — the same
        // trusted time source the ops-plane clock uses in production.
        Self::new_with_clock(store, index, mac_key, RemotedClock::wall())
    }

    /// Build a shared-trust handle with an explicit access-check [`RemotedClock`]
    /// for grant expiry. Production uses [`SharedTrust::new`] (wall); this seam lets a
    /// test inject a [`RemotedClock::fixed`] and advance it past an owner-signed
    /// grant's expiry to prove the accept gate then DENIES.
    pub fn new_with_clock(
        store: TrustStore,
        index: DeviceAccountIndex,
        mac_key: Vec<u8>,
        clock: RemotedClock,
    ) -> Self {
        SharedTrust {
            inner: Arc::new(Mutex::new(TrustInner { store, index })),
            persist_lock: Arc::new(Mutex::new(())),
            mac_key: Arc::new(mac_key),
            clock,
            #[cfg(test)]
            race_delay: None,
        }
    }

    /// Lock the inner state, recovering the guard even across a poisoned mutex
    /// (a panic while mutating must not permanently wedge the always-on gate — a
    /// poisoned read still reflects the last consistent state). The `inner` lock
    /// is deny-closed under a partial mutation (store-before-index ordering + the
    /// deferred live-bind mean a torn commit reads `Unpaired`), so recovering it is
    /// safe — unlike an integrity-critical hash-chained sink (see `ops.rs`).
    fn lock(&self) -> std::sync::MutexGuard<'_, TrustInner> {
        self.inner.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Acquire the writer serialization lock (held across a whole commit's
    /// critical section, INCLUDING its disk I/O). Recovers a poisoned guard: the
    /// `()` it guards has no state, and the store-before-index ordering keeps a
    /// panic-mid-commit disk state deny-closed, so a serial retry is safe.
    fn writer_lock(&self) -> std::sync::MutexGuard<'_, ()> {
        self.persist_lock.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Test-only: inject a sleep between the commit snapshot and the disk write to
    /// widen the concurrent-commit race window.
    #[cfg(test)]
    fn with_race_delay(mut self, delay: std::time::Duration) -> Self {
        self.race_delay = Some(delay);
        self
    }

    // -- reads (accept gate) -------------------------------------------------

    /// The claimed flag AND the device key's access state, read under ONE lock so
    /// the accept gate never observes a torn (claim mid-decision) view. The access
    /// state is evaluated at the robot's own trusted `now_ns`, so a
    /// device key whose only row is an EXPIRED owner-signed grant reads
    /// [`KeyAccess::NotAllowed`] — the accept gate denies it without an explicit
    /// revoke.
    pub fn snapshot_for_key(&self, device_key: &[u8; 32]) -> (bool, KeyAccess) {
        // Read the trusted time OUTSIDE the store lock (the wall clock is a syscall).
        let now_ns = self.clock.now_ns();
        let key = PublicKey(*device_key);
        let guard = self.lock();
        let claimed = guard.store.is_claimed();
        // A per-device revocation denies this desk REGARDLESS of its
        // account row (the account's other devices stay allowed). Checked FIRST so a
        // revoked device is cut even while its account is fine, and so the
        // mid-session sweep evicts it the moment a synced epoch flips this bit.
        let access = if guard.store.is_device_revoked(&key) {
            KeyAccess::DeviceRevoked
        } else {
            match guard.index.account_for(&key) {
                None => KeyAccess::Unpaired,
                Some(account) => match guard.store.is_allowed(&account, now_ns) {
                    None => KeyAccess::NotAllowed,
                    Some(row) => KeyAccess::Allowed(row.scope),
                },
            }
        };
        (claimed, access)
    }

    /// The robot's own transport key (its iroh `EndpointId`), recorded at
    /// provision time — the key a code-pairing witness must be proven against.
    pub fn robot_transport_key(&self) -> PublicKey {
        self.lock().store.robot_transport_key()
    }

    // -- mutations (pairing bootstrap verbs) ---------------------------------

    /// Claim an unclaimed robot by proving physical possession (the chassis
    /// secret), writing the owner row AND binding the claiming device key to the
    /// owner account. Persists both artifacts (store first — deny-closed on a
    /// partial failure). The ONLY admissible mutation on an unclaimed robot.
    pub fn claim(
        &self,
        device_key: &[u8; 32],
        account: AccountId,
        chassis_secret: &[u8],
        principal_kind: PrincipalKind,
        now_ns: u64,
    ) -> Result<(), TrustError> {
        // Serialize the WHOLE commit (mutation → snapshot → I/O → publish) so a
        // concurrent commit cannot clobber the on-disk binding. Acquired BEFORE the
        // inner lock (consistent order → no deadlock with the accept-gate readers).
        let _writer = self.writer_lock();
        let mut guard = self.lock();
        guard
            .store
            .claim(account, chassis_secret, principal_kind, now_ns)?;
        self.commit(guard, device_key, account)
    }

    /// Verify a strong-path [`PairingPresentation`] offline against the trust
    /// anchor (asserting the cert's device key equals the transport-authenticated
    /// peer key), establish the access row, and bind the device key to the proven
    /// account. Returns the established account. Persists both artifacts.
    pub fn verify_and_establish(
        &self,
        presentation: &PairingPresentation,
        authenticated_peer_key: &[u8; 32],
        name: Option<String>,
        now_ns: u64,
    ) -> Result<AccountId, TrustError> {
        let peer = PublicKey(*authenticated_peer_key);
        let _writer = self.writer_lock();
        let mut guard = self.lock();
        let verified = guard
            .store
            .verify_and_establish(presentation, &peer, now_ns, name)?;
        let account = verified.account();
        self.commit(guard, authenticated_peer_key, account)?;
        Ok(account)
    }

    /// Admit a certified device of the current owner without rewriting its access
    /// row. The verified binding is published only after store and index persist.
    pub fn verify_owner_certificate_and_establish(
        &self,
        presentation: &OwnerCertificatePresentationWire,
        authenticated_peer_key: &[u8; 32],
        now_ns: u64,
    ) -> Result<AccountId, TrustError> {
        let peer = PublicKey(*authenticated_peer_key);
        let _writer = self.writer_lock();
        let mut guard = self.lock();
        let account = guard
            .store
            .verify_owner_certificate(presentation, &peer, now_ns)?;
        self.commit(guard, authenticated_peer_key, account)?;
        Ok(account)
    }

    /// Verify a desk-carried, owner-signed [`OwnerGrantPresentation`] OFFLINE against
    /// the robot's OWN owner chain, establish the subject's access row,
    /// and bind the authenticated device key to the proven subject account. Returns
    /// the established account. Persists both artifacts (store-before-index,
    /// fail-closed — the SAME commit path the strong `pair` path uses).
    ///
    /// The proof is the owner's signature over the grant, verified against the robot's
    /// claimed owner ([`TrustStore::establish_by_owner_grant`]) — no cloud contact, no
    /// synced ACL. A verification failure surfaces as [`TrustError::Pairing`]; the
    /// device key never becomes `Allowed` on any error.
    pub fn establish_owner_grant(
        &self,
        presentation: &OwnerGrantPresentation,
        authenticated_peer_key: &[u8; 32],
        name: Option<String>,
        now_ns: u64,
    ) -> Result<AccountId, TrustError> {
        let peer = PublicKey(*authenticated_peer_key);
        let _writer = self.writer_lock();
        let mut guard = self.lock();
        let verified = guard
            .store
            .establish_by_owner_grant(presentation, &peer, now_ns, name)?;
        let account = verified.account();
        self.commit(guard, authenticated_peer_key, account)?;
        Ok(account)
    }

    /// Establish a code-paired (CPace fallback) access row from an unforgeable
    /// [`CpaceConfirmed`] witness (which the CPace ceremony minted for `account`),
    /// binding the authenticated device key to that account. The witness — never a
    /// bare account id — is the proof-before-persist gate; `scope` is the caller's
    /// conservative default. Persists both artifacts.
    pub fn establish_code_pairing(
        &self,
        confirmed: &CpaceConfirmed,
        device_key: &[u8; 32],
        account: AccountId,
        name: String,
        scope: Scope,
        now_ns: u64,
    ) -> Result<(), TrustError> {
        let _writer = self.writer_lock();
        let mut guard = self.lock();
        guard
            .store
            .establish_code_pairing(confirmed, account, name, scope, now_ns)?;
        self.commit(guard, device_key, account)
    }

    /// Apply a signed access-list epoch to the LIVE store and persist
    /// it. This is the robot-side revocation SYNC seam: an owner revokes an account
    /// OR a device via the account service, which issues a fresh
    /// [`SignedEpoch`]; applying it here flips
    /// [`TrustStore::is_allowed`] / [`TrustStore::is_device_revoked`], so the accept
    /// gate ([`SharedTrust::snapshot_for_key`]) refuses NEW connections AND the
    /// mid-session sweep evicts LIVE sessions on the next tick.
    ///
    /// Only the trust STORE is mutated (the `device_key → account` index is
    /// untouched), so only the store is persisted — and only when the epoch was
    /// actually [`EpochOutcome::Applied`] (a stale [`EpochOutcome::NotNewer`] epoch
    /// changes nothing, so it never touches the disk). A rejected epoch (bad
    /// signature / wrong robot / rollback) surfaces as [`TrustError::Pairing`] with
    /// no state change; a persist failure is a loud [`TrustError::Persist`] (the
    /// in-memory epoch lingers but a reboot reloads the last durable state).
    ///
    /// Serialized under the same `writer_lock` as the pairing commits so an epoch
    /// apply never races a concurrent pairing's snapshot→write.
    pub fn apply_epoch(
        &self,
        signed_epoch: &SignedEpoch,
        intermediate: &SignedIntermediateCert,
        now_ns: u64,
    ) -> Result<EpochOutcome, TrustError> {
        let _writer = self.writer_lock();
        let mut guard = self.lock();
        let outcome = guard
            .store
            .apply_epoch(signed_epoch, intermediate, now_ns)?;
        // A stale epoch changed nothing → nothing to persist.
        if matches!(outcome, EpochOutcome::NotNewer { .. }) {
            return Ok(outcome);
        }
        // Read-only handle (no MAC key — the pure-authz test shape): applied in memory
        // only; nothing to persist.
        if self.mac_key.is_empty() {
            return Ok(outcome);
        }
        // Snapshot the store to an OWNED buffer, release the lock BEFORE the blocking
        // disk write (the accept gate never blocks on our I/O), then persist. Only the
        // store changed (the index is untouched by an epoch).
        let store_snapshot = guard.store.clone();
        drop(guard);
        store_snapshot
            .save(&self.mac_key)
            .map_err(|e| TrustError::Persist(format!("trust store (epoch): {e}")))?;
        Ok(outcome)
    }

    /// Commit a mutation whose STORE half is already applied to the live store
    /// under `guard`: bind `device_key → account` DURABLY, then publish the live
    /// binding — FAIL-CLOSED, with the blocking disk writes OUTSIDE the `inner`
    /// lock. Called with the `persist_lock` (writer serialization) HELD by the
    /// caller, so the snapshot → I/O → publish sequence is atomic w.r.t. other
    /// commits (no concurrent-write clobber).
    ///
    /// Steps (the strict f0/f1 fix):
    /// 1. Snapshot the two artifacts to OWNED buffers under the lock — the store
    ///    (a deep clone) and a clone of the index WITH the new binding added — then
    ///    RELEASE the guard. The live index is NOT mutated yet.
    /// 2. Write to disk OUTSIDE the lock (so the blocking I/O never runs under the
    ///    mutex the async accept gate reads — f1), ordered STORE-BEFORE-INDEX: a
    ///    partial failure is then DENY-CLOSED — a durable key binding never outlives
    ///    its account row, so a store-ok/index-fail leaves a row with no binding
    ///    (`snapshot_for_key` → account_for=None → denied) and a store-fail leaves
    ///    neither.
    /// 3. Only after the durable write succeeds, publish the binding to the LIVE
    ///    index. On ANY disk-write failure the live index is left untouched (the
    ///    key never became `Allowed`), so there is nothing to roll back and no
    ///    transient-Allowed window — fail-closed by construction (f0).
    ///
    /// The store row / ownership the caller mutated in the live store lingers on a
    /// persist failure (not easily reversible), but it is unreachable without a
    /// binding, and a reboot reloads the deny-closed disk state. A persist error is
    /// surfaced LOUDLY — a client whose verb returned `Err` never retains access.
    fn commit(
        &self,
        mut guard: std::sync::MutexGuard<'_, TrustInner>,
        device_key: &[u8; 32],
        account: AccountId,
    ) -> Result<(), TrustError> {
        let key = PublicKey(*device_key);

        // Read-only handle (no MAC key — the pure-authz test shape): apply the
        // binding in memory only; nothing to persist.
        if self.mac_key.is_empty() {
            tracing::debug!(
                "SharedTrust: mutation applied in memory only (no MAC key configured — \
                 read-only handle); not persisting"
            );
            guard.index.bind(key, account);
            return Ok(());
        }

        // Owned snapshots to write outside the lock; the live index is untouched.
        let store_snapshot = guard.store.clone();
        let mut index_snapshot = guard.index.clone();
        index_snapshot.bind(key, account);
        drop(guard); // release BEFORE the blocking disk writes (f1).

        // Test-only: widen the concurrent-commit race window so the persist_lock
        // serialization is deterministically load-bearing (see the concurrency pin).
        #[cfg(test)]
        if let Some(d) = self.race_delay {
            std::thread::sleep(d);
        }

        // STORE first, INDEX second → a partial failure is deny-closed (f0).
        self.persist_snapshots(&store_snapshot, &index_snapshot)?;

        // Durable write succeeded → publish the binding to the LIVE index.
        self.lock().index.bind(key, account);
        Ok(())
    }

    /// Persist owned snapshots STORE-FIRST then INDEX, authenticated with the MAC
    /// key. Store-before-index is the deny-closed ordering (a durable key binding
    /// never outlives its account row on a partial failure).
    fn persist_snapshots(
        &self,
        store: &TrustStore,
        index: &DeviceAccountIndex,
    ) -> Result<(), TrustError> {
        store
            .save(&self.mac_key)
            .map_err(|e| TrustError::Persist(format!("trust store: {e}")))?;
        index
            .save(&self.mac_key)
            .map_err(|e| TrustError::Persist(format!("device index: {e}")))?;
        Ok(())
    }
}

/// The default conservative scope a code-paired (fallback) row receives: observe
/// only, no teleop, no delegation — mirrors [`Scope::CODE_PAIR_DEFAULT`].
pub const CODE_PAIR_SCOPE: Scope = Scope::CODE_PAIR_DEFAULT;

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_pairing::format::{
        AccessListEpoch, DeviceCert, Grant, IntermediateCert, RobotId, RootSet,
        SignedIntermediateCert, Validity, FORMAT_VERSION,
    };
    use cerulion_pairing::verify::PairingSource;
    use ed25519_dalek::SigningKey;

    const CHASSIS: &[u8] = b"trust-unit-chassis-secret";
    const MAC: &[u8] = b"trust-unit-mac-key";
    const T_NOW: u64 = 1_000_000_000_000;
    const ISSUED: u64 = 500_000_000_000;
    const ROBOT: RobotId = RobotId([5; 32]);
    const OWNER: AccountId = AccountId([10; 32]);

    fn root_sk() -> SigningKey {
        SigningKey::from_bytes(&[1; 32])
    }
    fn int_sk() -> SigningKey {
        SigningKey::from_bytes(&[2; 32])
    }
    fn pubkey(sk: &SigningKey) -> PublicKey {
        PublicKey(sk.verifying_key().to_bytes())
    }
    fn wide() -> Validity {
        Validity {
            not_before_ns: 0,
            not_after_ns: 100_000_000_000_000,
        }
    }

    /// A provisioned (unclaimed) store + index at `store_path` / `index_path`,
    /// with the REAL root pubkey so a cert chain built with [`pair_chain`] verifies.
    fn provision_at(store_path: &std::path::Path, index_path: &std::path::Path) -> SharedTrust {
        let root_set = RootSet::new(vec![pubkey(&root_sk())], 1).unwrap();
        let store = TrustStore::provision(ROBOT, PublicKey([6; 32]), root_set, CHASSIS, T_NOW)
            .unwrap()
            .with_path(store_path);
        let index = DeviceAccountIndex::new().with_path(index_path);
        SharedTrust::new(store, index, MAC.to_vec())
    }

    /// A store + index colocated under one `dir` (the simple happy-path shape).
    fn provision(dir: &std::path::Path) -> SharedTrust {
        provision_at(&dir.join("trust_store"), &dir.join("device_index.json"))
    }

    /// A real offline cert chain (root → intermediate → device cert for
    /// `device_key` → grant) for `account` at `scope`.
    fn pair_chain(account: AccountId, device_key: [u8; 32], scope: Scope) -> PairingPresentation {
        let int_pk = pubkey(&int_sk());
        let intermediate = IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: int_pk,
            validity: wide(),
            issued_at_ns: ISSUED,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&root_sk()]);
        let device_cert = DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(device_key),
            account,
            principal_kind: PrincipalKind::Human,
            scope,
            validity: wide(),
            issued_at_ns: ISSUED,
            issuer_key: int_pk,
        }
        .sign(&int_sk());
        let grant = Grant {
            version: FORMAT_VERSION,
            subject: account,
            robot: ROBOT,
            scope,
            principal_kind: PrincipalKind::Human,
            delegation_depth: 0,
            validity: wide(),
            issued_at_ns: ISSUED,
            issuer: AccountId([2; 32]),
            issuer_key: int_pk,
        }
        .sign(&int_sk());
        PairingPresentation {
            intermediate,
            device_cert,
            grant,
            delegation: None,
        }
    }

    const VIEWER_SCOPE: Scope = Scope {
        role: cerulion_pairing::format::Role::VIEWER,
        caps: Scope::CAP_OBSERVE,
    };

    /// Reload a fresh SharedTrust from disk — the exact reboot path.
    fn reload(store_path: &std::path::Path, index_path: &std::path::Path) -> SharedTrust {
        let store = TrustStore::load(store_path, MAC).expect("store reloads");
        let index = DeviceAccountIndex::load(index_path, MAC).expect("index reloads");
        SharedTrust::new(store, index, MAC.to_vec())
    }

    #[cfg(unix)]
    fn set_mode(dir: &std::path::Path, mode: u32) {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(dir, std::fs::Permissions::from_mode(mode)).unwrap();
    }

    fn owner_certificate(key: [u8; 32]) -> OwnerCertificatePresentationWire {
        let chain = pair_chain(OWNER, key, Scope::OWNER_FULL);
        OwnerCertificatePresentationWire::new(chain.intermediate, chain.device_cert)
    }

    #[test]
    fn owner_certificate_adds_durable_binding_without_changing_owner_row() {
        let dir = tempfile::tempdir().unwrap();
        let shared = provision(dir.path());
        shared
            .claim(&[7; 32], OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        let before = postcard::to_stdvec(shared.lock().store.access_rows()).unwrap();
        let key = pubkey(&SigningKey::from_bytes(&[8; 32])).0;
        assert_eq!(shared.snapshot_for_key(&key).1, KeyAccess::Unpaired);
        assert_eq!(
            shared
                .verify_owner_certificate_and_establish(&owner_certificate(key), &key, T_NOW + 1)
                .unwrap(),
            OWNER
        );
        assert_eq!(
            shared.snapshot_for_key(&key).1,
            KeyAccess::Allowed(Scope::OWNER_FULL)
        );
        assert_eq!(
            postcard::to_stdvec(shared.lock().store.access_rows()).unwrap(),
            before
        );
        let loaded = reload(
            &dir.path().join("trust_store"),
            &dir.path().join("device_index.json"),
        );
        assert_eq!(
            loaded.snapshot_for_key(&key).1,
            KeyAccess::Allowed(Scope::OWNER_FULL)
        );
        assert_eq!(
            postcard::to_stdvec(loaded.lock().store.access_rows()).unwrap(),
            before
        );
        assert_eq!(loaded.lock().store.high_water_ns(), T_NOW + 1);
    }

    #[test]
    fn owner_certificate_store_and_index_failures_never_publish_new_binding() {
        for fail_store in [true, false] {
            let dir = tempfile::tempdir().unwrap();
            let shared = provision(dir.path());
            shared
                .claim(&[7; 32], OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
                .unwrap();
            // A directory at the serializer's temp path deterministically fails
            // writes without relying on uid-dependent permission behavior.
            let blocker = if fail_store {
                "trust_store.tmp"
            } else {
                "device_index.json.tmp"
            };
            std::fs::create_dir(dir.path().join(blocker)).unwrap();
            let key = pubkey(&SigningKey::from_bytes(&[8; 32])).0;
            assert!(matches!(
                shared.verify_owner_certificate_and_establish(
                    &owner_certificate(key),
                    &key,
                    T_NOW + 1
                ),
                Err(TrustError::Persist(_))
            ));
            assert_eq!(shared.snapshot_for_key(&key).1, KeyAccess::Unpaired);
            let loaded = reload(
                &dir.path().join("trust_store"),
                &dir.path().join("device_index.json"),
            );
            assert_eq!(loaded.snapshot_for_key(&key).1, KeyAccess::Unpaired);
            assert_eq!(
                loaded.snapshot_for_key(&[7; 32]).1,
                KeyAccess::Allowed(Scope::OWNER_FULL)
            );
        }
    }

    #[test]
    fn a_successful_claim_binds_the_key_anti_tautology() {
        // Control: a claim whose persist SUCCEEDS (a writable temp dir) binds the
        // key → Allowed(OWNER_FULL). Proves the fail-closed tests are not vacuous
        // (the apparatus really binds on success, over a disk reload).
        let dir = tempfile::tempdir().unwrap();
        let shared = provision(dir.path());
        let key = [7u8; 32];
        shared
            .claim(&key, OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .expect("a claim with a writable state dir succeeds");
        let (claimed, access) = shared.snapshot_for_key(&key);
        assert!(claimed);
        assert_eq!(access, KeyAccess::Allowed(Scope::OWNER_FULL));
        // ...and it is durable: a fresh reload from disk still Allows the key.
        let reloaded = reload(
            &dir.path().join("trust_store"),
            &dir.path().join("device_index.json"),
        );
        assert_eq!(
            reloaded.snapshot_for_key(&key).1,
            KeyAccess::Allowed(Scope::OWNER_FULL)
        );
    }

    #[test]
    fn a_store_save_failure_never_publishes_the_binding_fail_closed() {
        // Both saves fail (a nonexistent backing dir → store.save fails FIRST). The
        // in-memory store mutation lingers, but the binding is NEVER published, so
        // the key reads Unpaired (DENIED) — fail-closed, no rollback needed.
        let missing = std::path::Path::new("/nonexistent-trust/does/not/exist");
        let shared = provision_at(
            &missing.join("trust_store"),
            &missing.join("device_index.json"),
        );
        let key = [7u8; 32];
        let err = shared
            .claim(&key, OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .expect_err("a claim whose persist fails must surface an error");
        assert!(matches!(err, TrustError::Persist(_)), "got {err:?}");
        assert_eq!(
            shared.snapshot_for_key(&key).1,
            KeyAccess::Unpaired,
            "a persist failure must leave the key Unpaired (fail-closed)"
        );
    }

    /// Headline: an index-ok / store-fail partial with a
    /// pre-existing account row is deny-closed on a disk reload. The store-before-
    /// index ordering makes store.save fail FIRST, so the K2→A binding is NEVER
    /// durably written — no multi-device ghost. Reverting `persist_snapshots` to
    /// index-first writes the K2 binding durably while
    /// store.save fails, and the reload then reads K2 as Allowed → this test FAILS.
    #[cfg(unix)]
    #[test]
    fn multi_device_partial_persist_is_deny_closed_on_reload() {
        let dir = tempfile::tempdir().unwrap();
        let store_dir = dir.path().join("s");
        let index_dir = dir.path().join("i");
        std::fs::create_dir(&store_dir).unwrap();
        std::fs::create_dir(&index_dir).unwrap();
        let store_path = store_dir.join("trust_store");
        let index_path = index_dir.join("device_index.json");

        let account_a = AccountId([20; 32]);
        let k0_owner = [1u8; 32];
        let k1 = [2u8; 32]; // Alice's phone → account A (paired cleanly)
        let k2 = [3u8; 32]; // Alice's laptop → account A (partial persist failure)

        // Seed cleanly (both dirs writable): claim OWNER, then pair K1 → A.
        let shared = provision_at(&store_path, &index_path);
        shared
            .claim(&k0_owner, OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        shared
            .verify_and_establish(&pair_chain(account_a, k1, VIEWER_SCOPE), &k1, None, T_NOW)
            .expect("K1 pairs cleanly");

        // Now make the STORE dir read-only so the NEXT store.save fails (its old
        // file stays intact), while the index dir stays writable.
        set_mode(&store_dir, 0o500);
        let err = shared
            .verify_and_establish(&pair_chain(account_a, k2, VIEWER_SCOPE), &k2, None, T_NOW)
            .expect_err("K2's pairing must fail when store.save fails");
        assert!(matches!(err, TrustError::Persist(_)), "got {err:?}");
        // Restore write perms so the tempdir can be cleaned up + reloaded.
        set_mode(&store_dir, 0o700);

        // The RELOAD (reboot) is deny-closed: K2 never got a durable binding, so it
        // is DENIED; K1 (and the owner) keep their access — no ghost, no collateral.
        let reloaded = reload(&store_path, &index_path);
        assert_eq!(
            reloaded.snapshot_for_key(&k2).1,
            KeyAccess::Unpaired,
            "K2's partial-failed pairing must be DENIED on reload (no ghost binding)"
        );
        assert!(
            matches!(reloaded.snapshot_for_key(&k1).1, KeyAccess::Allowed(_)),
            "K1's clean pairing survives K2's failure"
        );
        assert!(matches!(
            reloaded.snapshot_for_key(&k0_owner).1,
            KeyAccess::Allowed(_)
        ));
    }

    /// Race pin: two concurrent commits (K1→A, K2→A)
    /// via real threads must both persist — after both return, a fresh reload from
    /// disk Allows BOTH keys. The `persist_lock` serializes the whole
    /// snapshot→write→publish, so the second commit's index snapshot includes the
    /// first's published binding (no clobber). An injected race-window sleep makes
    /// the clobber deterministic when the lock is removed: dropping the
    /// serialization writes two overlapping snapshots and one key is lost on reload.
    #[test]
    fn concurrent_commits_both_persist_no_clobber_on_reload() {
        let dir = tempfile::tempdir().unwrap();
        let store_path = dir.path().join("trust_store");
        let index_path = dir.path().join("device_index.json");

        // Claim the owner first (a single, uncontended seed), then run TWO
        // concurrent pairings to distinct accounts through the shared handle.
        let shared = provision(dir.path()).with_race_delay(std::time::Duration::from_millis(150));
        shared
            .claim(&[1u8; 32], OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();

        let account_a = AccountId([20; 32]);
        let account_b = AccountId([30; 32]);
        let k1 = [2u8; 32];
        let k2 = [3u8; 32];

        let s1 = shared.clone();
        let s2 = shared.clone();
        let t1 = std::thread::spawn(move || {
            s1.verify_and_establish(&pair_chain(account_a, k1, VIEWER_SCOPE), &k1, None, T_NOW)
        });
        let t2 = std::thread::spawn(move || {
            s2.verify_and_establish(&pair_chain(account_b, k2, VIEWER_SCOPE), &k2, None, T_NOW)
        });
        t1.join().unwrap().expect("K1 pairs");
        t2.join().unwrap().expect("K2 pairs");

        // BOTH bindings are durable: a fresh reload Allows both keys. Reverting the
        // `persist_lock` serialization makes the two overlapping snapshots clobber
        // one binding → that key is Unpaired on reload → this assertion FAILS.
        let reloaded = reload(&store_path, &index_path);
        assert!(
            matches!(reloaded.snapshot_for_key(&k1).1, KeyAccess::Allowed(_)),
            "K1 must survive a concurrent commit"
        );
        assert!(
            matches!(reloaded.snapshot_for_key(&k2).1, KeyAccess::Allowed(_)),
            "K2 must survive a concurrent commit (no clobber)"
        );
    }

    /// The OTHER partial direction: STORE-ok / INDEX-fail is also deny-closed on
    /// reload — the account row is durable but the key binding is not.
    #[cfg(unix)]
    #[test]
    fn store_ok_index_fail_partial_is_deny_closed_on_reload() {
        let dir = tempfile::tempdir().unwrap();
        let store_dir = dir.path().join("s");
        let index_dir = dir.path().join("i");
        std::fs::create_dir(&store_dir).unwrap();
        std::fs::create_dir(&index_dir).unwrap();
        let store_path = store_dir.join("trust_store");
        let index_path = index_dir.join("device_index.json");

        let account_b = AccountId([30; 32]);
        let k0_owner = [1u8; 32];
        let k = [4u8; 32];

        let shared = provision_at(&store_path, &index_path);
        shared
            .claim(&k0_owner, OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();

        // Make the INDEX dir read-only: store.save succeeds (writing B's row), then
        // index.save fails → the binding is not durable.
        set_mode(&index_dir, 0o500);
        let err = shared
            .verify_and_establish(&pair_chain(account_b, k, VIEWER_SCOPE), &k, None, T_NOW)
            .expect_err("index.save failure must fail the pairing");
        assert!(matches!(err, TrustError::Persist(_)), "got {err:?}");
        set_mode(&index_dir, 0o700);

        // On reload the key is DENIED: B's row is on disk but no binding maps to it.
        let reloaded = reload(&store_path, &index_path);
        assert_eq!(reloaded.snapshot_for_key(&k).1, KeyAccess::Unpaired);
    }

    #[test]
    fn pair_refuses_a_presentation_whose_device_key_is_not_the_authed_peer() {
        // f2 (auth-plane): the strong path asserts device_cert.device_key ==
        // authenticated_peer_key. A presentation built for K_cert presented over a
        // connection authed as K_other → Err, and NEITHER key gets a row/binding.
        let dir = tempfile::tempdir().unwrap();
        let shared = provision(dir.path());
        let k_owner = [1u8; 32];
        shared
            .claim(&k_owner, OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();

        let account_a = AccountId([20; 32]);
        let k_cert = [8u8; 32]; // the cert is issued for this device key
        let k_other = [9u8; 32]; // ...but the connection is authenticated as this one
        let pres = pair_chain(account_a, k_cert, VIEWER_SCOPE);

        let err = shared
            .verify_and_establish(&pres, &k_other, None, T_NOW)
            .expect_err("a device-key ≠ authed-peer presentation must be refused");
        assert!(matches!(err, TrustError::Pairing(_)), "got {err:?}");
        // No row, no binding for EITHER key — the refusal wrote nothing.
        assert_eq!(shared.snapshot_for_key(&k_cert).1, KeyAccess::Unpaired);
        assert_eq!(shared.snapshot_for_key(&k_other).1, KeyAccess::Unpaired);
    }

    #[test]
    fn a_clean_strong_pairing_binds_and_reloads_control() {
        // Anti-tautology for the f2 refusal: the SAME chain presented over the
        // MATCHING authed key pairs cleanly and Allows the key on reload.
        let dir = tempfile::tempdir().unwrap();
        let shared = provision(dir.path());
        let k_owner = [1u8; 32];
        shared
            .claim(&k_owner, OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        let account_a = AccountId([20; 32]);
        let k = [8u8; 32];
        let account = shared
            .verify_and_establish(
                &pair_chain(account_a, k, VIEWER_SCOPE),
                &k,
                Some("v".into()),
                T_NOW,
            )
            .expect("a matching device-key presentation pairs");
        assert_eq!(account, account_a);
        let reloaded = reload(
            &dir.path().join("trust_store"),
            &dir.path().join("device_index.json"),
        );
        match reloaded.snapshot_for_key(&k).1 {
            KeyAccess::Allowed(scope) => assert_eq!(scope, VIEWER_SCOPE),
            other => panic!("expected Allowed(VIEWER_SCOPE), got {other:?}"),
        }
        // The row is a real strong-chain row on disk.
        let store = TrustStore::load(dir.path().join("trust_store"), MAC).unwrap();
        assert_eq!(
            store.is_allowed(&account_a, T_NOW).unwrap().source,
            PairingSource::StrongChain
        );
    }

    // ── SharedTrust::apply_epoch device revocation ───────────────────────────

    /// The intermediate cert the store trusts (`int_sk` signed by the single root),
    /// needed to apply an epoch.
    fn good_intermediate_cert() -> SignedIntermediateCert {
        IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: pubkey(&int_sk()),
            validity: wide(),
            issued_at_ns: ISSUED,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&root_sk()])
    }

    /// A signed epoch (number `n`) revoking `devices`, signed by the intermediate.
    fn device_revoke_epoch(
        n: u64,
        devices: Vec<PublicKey>,
    ) -> cerulion_pairing::format::SignedEpoch {
        AccessListEpoch {
            version: FORMAT_VERSION,
            robot: ROBOT,
            epoch: n,
            revoked_accounts: vec![],
            revoked_devices: devices,
            issued_at_ns: ISSUED,
            issuer_key: pubkey(&int_sk()),
        }
        .sign(&int_sk())
    }

    /// Headline (SharedTrust wrapper): applying an epoch that revokes ONE
    /// device key flips the accept gate to `DeviceRevoked` for that key while a
    /// SIBLING device of the SAME account stays `Allowed`, persists, and is
    /// re-admitted by a newer epoch dropping it (the monotonic tombstone).
    #[test]
    fn apply_epoch_device_revocation_flips_the_accept_gate_persists_and_leaves_siblings() {
        let dir = tempfile::tempdir().unwrap();
        let shared = provision(dir.path());
        shared
            .claim(&[1u8; 32], OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();

        // Two devices of the SAME account A.
        let account_a = AccountId([20; 32]);
        let k_a = [2u8; 32]; // desk A
        let k_b = [3u8; 32]; // desk B (same account, different key)
        shared
            .verify_and_establish(&pair_chain(account_a, k_a, VIEWER_SCOPE), &k_a, None, T_NOW)
            .expect("desk A pairs");
        shared
            .verify_and_establish(&pair_chain(account_a, k_b, VIEWER_SCOPE), &k_b, None, T_NOW)
            .expect("desk B pairs");
        assert!(matches!(
            shared.snapshot_for_key(&k_a).1,
            KeyAccess::Allowed(_)
        ));
        assert!(matches!(
            shared.snapshot_for_key(&k_b).1,
            KeyAccess::Allowed(_)
        ));

        // Apply an epoch revoking ONLY desk A's device key.
        let outcome = shared
            .apply_epoch(
                &device_revoke_epoch(2, vec![PublicKey(k_a)]),
                &good_intermediate_cert(),
                T_NOW,
            )
            .expect("the epoch applies");
        assert!(matches!(outcome, EpochOutcome::Applied { .. }));

        // Desk A reads DeviceRevoked (the distinct signal); desk B stays Allowed.
        assert_eq!(shared.snapshot_for_key(&k_a).1, KeyAccess::DeviceRevoked);
        assert!(matches!(
            shared.snapshot_for_key(&k_b).1,
            KeyAccess::Allowed(_)
        ));

        // Persisted: a fresh reload from disk keeps desk A revoked + desk B allowed.
        let reloaded = reload(
            &dir.path().join("trust_store"),
            &dir.path().join("device_index.json"),
        );
        assert_eq!(reloaded.snapshot_for_key(&k_a).1, KeyAccess::DeviceRevoked);
        assert!(matches!(
            reloaded.snapshot_for_key(&k_b).1,
            KeyAccess::Allowed(_)
        ));

        // A newer epoch dropping the device re-admits desk A (monotonic replace).
        shared
            .apply_epoch(
                &device_revoke_epoch(3, vec![]),
                &good_intermediate_cert(),
                T_NOW,
            )
            .expect("the re-admit epoch applies");
        assert!(matches!(
            shared.snapshot_for_key(&k_a).1,
            KeyAccess::Allowed(_)
        ));
    }

    /// A stale epoch (not newer) is a true no-op: `NotNewer`, no state change, and it
    /// never touches the disk (the persist is skipped on `NotNewer`).
    #[test]
    fn apply_epoch_stale_is_a_noop_and_a_bad_signature_is_a_loud_error() {
        let dir = tempfile::tempdir().unwrap();
        let shared = provision(dir.path());
        shared
            .claim(&[1u8; 32], OWNER, CHASSIS, PrincipalKind::Human, T_NOW)
            .unwrap();
        let k_a = [2u8; 32];
        shared
            .apply_epoch(
                &device_revoke_epoch(5, vec![PublicKey(k_a)]),
                &good_intermediate_cert(),
                T_NOW,
            )
            .unwrap();
        assert_eq!(shared.snapshot_for_key(&k_a).1, KeyAccess::DeviceRevoked);

        // An OLDER epoch (2 <= 5) is NotNewer — no change (desk A stays revoked).
        let outcome = shared
            .apply_epoch(
                &device_revoke_epoch(2, vec![]),
                &good_intermediate_cert(),
                T_NOW,
            )
            .unwrap();
        assert!(matches!(outcome, EpochOutcome::NotNewer { current: 5 }));
        assert_eq!(shared.snapshot_for_key(&k_a).1, KeyAccess::DeviceRevoked);

        // A bad-signature epoch is a loud Pairing error (never silently applied).
        let mut bad = device_revoke_epoch(9, vec![]);
        bad.signature = cerulion_pairing::format::Signature([0xAB; 64]);
        let err = shared
            .apply_epoch(&bad, &good_intermediate_cert(), T_NOW)
            .expect_err("a forged epoch is refused");
        assert!(matches!(err, TrustError::Pairing(_)), "got {err:?}");
    }
}
