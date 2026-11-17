// SPDX-License-Identifier: AGPL-3.0-only
//! Locked local identity reads shared by desk transports.
//!
//! Identity-only reads never open certificate files. Full reads preserve the
//! account, key, leaf and chain as one transaction without checking expiry or
//! contacting the account service. Callers supply their platform's read-only,
//! nonblocking file opener; this module validates the opened descriptors.
//! Proof checks cover structure and identity consistency. The receiving robot
//! remains responsible for signatures, trust roots, expiry and authorization.

use std::fs::{File, TryLockError};
use std::io;
use std::path::Path;

use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PublicKey};
use cerulion_pairing::verify::OwnerCertificatePresentationWire;

use crate::identity_cache::{self, CachedOwnerChain};

mod login;
pub use login::{decode_login_identity, read_identity_at, read_regular_bounded, LoginProblem};

const MAX_CERT_BYTES: u64 = 64 * 1024;

/// A typed local-state failure containing no token, seed or untrusted file text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapshotError {
    /// Configuration home cannot be resolved by the caller.
    NoConfigHome,
    /// The existing sibling store lock could not be opened or validated.
    Lock(LoginProblem),
    /// A login writer owns the transaction lock; retry outside other state locks.
    Busy,
    /// The durable login is absent or malformed.
    Auth(LoginProblem),
    /// The saved account is neither a hosted UUID nor a legacy pairing id.
    Account,
    /// The existing device key could not be read safely.
    Key(LoginProblem),
    /// The existing device key does not contain exactly 32 bytes.
    KeySize,
    /// The present public leaf could not be read safely.
    Certificate(LoginProblem),
    /// The present public chain cache could not be read safely.
    Chain(LoginProblem),
    /// A cache exists without its current leaf.
    IncompleteChain,
    /// Present certificate material is malformed or names another identity.
    InvalidCertificate,
}

/// Account and actual transport key from one locked read; no proof is required.
/// Deliberately has no Debug or serialization implementation.
pub struct LocalIdentity {
    auth_account_id: String,
    session_present: bool,
    account: AccountId,
    seed: [u8; 32],
    device_key: PublicKey,
}

impl LocalIdentity {
    /// Original account-service identity retained in auth.json.
    pub fn auth_account_id(&self) -> &str {
        &self.auth_account_id
    }
    /// Whether the stored session token is nonempty, without checking expiry.
    /// This exposes no credential bytes and lets account actions detect logout.
    pub fn session_present(&self) -> bool {
        self.session_present
    }
    /// Pairing identity mapped from the original account-service id.
    pub fn account(&self) -> AccountId {
        self.account
    }
    /// Public half derived from the actual persisted seed.
    pub fn device_key(&self) -> PublicKey {
        self.device_key
    }
    /// Existing transport seed. Never log, format, or serialize this value.
    pub fn seed(&self) -> &[u8; 32] {
        &self.seed
    }
}

/// Exact public identity and certificate content used to fence publication.
/// Bearer refreshes and elapsed time do not change this value.
#[derive(Clone, PartialEq, Eq)]
pub struct IdentityStamp {
    auth_account_id: String,
    device_key: PublicKey,
    leaf: Option<Vec<u8>>,
    chain: Option<Vec<u8>>,
}

/// Identity and optional proof read in one transaction. No Debug or serialization.
pub struct IdentitySnapshot {
    identity: LocalIdentity,
    owner_chain: Option<CachedOwnerChain>,
    stamp: IdentityStamp,
}

impl IdentitySnapshot {
    /// Original account-service identity retained in auth.json.
    pub fn auth_account_id(&self) -> &str {
        self.identity.auth_account_id()
    }
    /// Whether the stored session token is nonempty, without checking expiry.
    pub fn session_present(&self) -> bool {
        self.identity.session_present()
    }
    /// Pairing identity mapped from the original account-service id.
    pub fn account(&self) -> AccountId {
        self.identity.account()
    }
    /// Public half derived from the actual persisted seed.
    pub fn device_key(&self) -> PublicKey {
        self.identity.device_key()
    }
    /// Existing transport seed. Never log, format, or serialize this value.
    pub fn seed(&self) -> &[u8; 32] {
        self.identity.seed()
    }
    /// Structurally consistent public proof, including a possibly expired proof.
    pub fn owner_chain(&self) -> Option<&OwnerCertificatePresentationWire> {
        self.owner_chain.as_ref().map(|chain| &chain.presentation)
    }
    /// Canonical public envelope bytes, matching the CLI's snapshot encoding.
    pub fn owner_chain_wire(&self) -> Option<&[u8]> {
        self.owner_chain
            .as_ref()
            .map(|chain| chain.postcard.as_slice())
    }
    /// Comparison stamp for an in-flight local identity operation.
    pub fn stamp(&self) -> &IdentityStamp {
        &self.stamp
    }
}

/// Read only auth and the existing machine key under the existing store lock.
///
/// `open` must open read-only without creating files and must not block on a
/// special file (for example, use O_NONBLOCK on Unix). `lock_path` is the same
/// never-renamed sibling lock used by the account writer. No certificate path
/// is opened, so proof errors cannot disable an established durable pairing.
pub fn load_identity(
    home: &Path,
    lock_path: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<LocalIdentity, SnapshotError> {
    let _lock = lock_existing(lock_path, open)?;
    read_identity_locked(home, open)
}

/// Read auth, key, leaf and chain under one existing shared store lock.
///
/// The opener and lock requirements are identical to [`load_identity`]. Valid
/// leaf-only and certificate-free logins return no proof. Present malformed or
/// mismatched material refuses the entire snapshot. No expiry check is performed.
pub fn load_with_proof(
    home: &Path,
    lock_path: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<IdentitySnapshot, SnapshotError> {
    let _lock = lock_existing(lock_path, open)?;
    let identity = read_identity_locked(home, open)?;
    let leaf =
        optional_file(&home.join("device.cert"), open).map_err(SnapshotError::Certificate)?;
    let chain =
        optional_file(&home.join("device-chain.json"), open).map_err(SnapshotError::Chain)?;
    let owner_chain = match (leaf.as_deref(), chain.as_deref()) {
        (None, None) => None,
        (None, Some(_)) => return Err(SnapshotError::IncompleteChain),
        (Some(leaf), chain) => identity_cache::decode_owner_chain(
            leaf,
            chain,
            identity.account(),
            identity.device_key(),
        )
        .map_err(|_| SnapshotError::InvalidCertificate)?,
    };
    let stamp = IdentityStamp {
        auth_account_id: identity.auth_account_id().to_owned(),
        device_key: identity.device_key(),
        leaf,
        chain,
    };
    Ok(IdentitySnapshot {
        identity,
        owner_chain,
        stamp,
    })
}

fn read_identity_locked(
    home: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<LocalIdentity, SnapshotError> {
    let (auth_account_id, session_present) =
        login::read_login_state(&home.join("auth.json"), open).map_err(SnapshotError::Auth)?;
    let account =
        identity_cache::pairing_account_id(&auth_account_id).map_err(|_| SnapshotError::Account)?;
    let seed: [u8; 32] = read_regular_bounded(
        &cerulion_pairing::identity::IdentityLayout::at(home).machine_key_read_path(),
        32,
        open,
    )
    .map_err(SnapshotError::Key)?
    .try_into()
    .map_err(|_| SnapshotError::KeySize)?;
    let device_key = DeviceIdentity::from_seed(&seed).public_key();
    Ok(LocalIdentity {
        auth_account_id,
        session_present,
        account,
        seed,
        device_key,
    })
}

fn optional_file(
    path: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<Option<Vec<u8>>, LoginProblem> {
    match read_regular_bounded(path, MAX_CERT_BYTES, open) {
        Ok(bytes) => Ok(Some(bytes)),
        Err(LoginProblem::Missing) => match std::fs::symlink_metadata(path) {
            Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(None),
            _ => Err(LoginProblem::Unreadable),
        },
        Err(error) => Err(error),
    }
}

fn lock_existing(
    path: &Path,
    open: &impl Fn(&Path) -> io::Result<File>,
) -> Result<File, SnapshotError> {
    let file = open(path).map_err(|error| {
        SnapshotError::Lock(if error.kind() == io::ErrorKind::NotFound {
            LoginProblem::Missing
        } else {
            LoginProblem::Unreadable
        })
    })?;
    if !file
        .metadata()
        .map_err(|_| SnapshotError::Lock(LoginProblem::Unreadable))?
        .is_file()
    {
        return Err(SnapshotError::Lock(LoginProblem::NotRegular));
    }
    file.try_lock_shared().map_err(|error| match error {
        TryLockError::WouldBlock => SnapshotError::Busy,
        TryLockError::Error(_) => SnapshotError::Lock(LoginProblem::Unreadable),
    })?;
    Ok(file)
}

#[cfg(test)]
mod tests;
