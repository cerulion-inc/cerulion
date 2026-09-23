// SPDX-License-Identifier: AGPL-3.0-only
//! Consistent local identity for the account controller, without network or writes.
//!
//! A shared lock on the same never-renamed file as login writers covers every
//! read. Busy writers refuse immediately. No token or certificate expiry clock
//! participates in identity: expiry gates new enrollment at the robot, not an
//! already established durable binding. Certificate checks here are structural
//! and identity consistency checks, not root-trust or signature verification.

use crate::serving_login::{self, LoginProblem};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PublicKey};
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use cerulion_wireclient::identity_cache::{self, CachedOwnerChain};
use std::fs::{File, OpenOptions, TryLockError};
use std::path::Path;

use cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE as STORE_LOCK_FILE;
const MAX_CERT_BYTES: u64 = 64 * 1024;

/// A typed local-state failure containing no token, seed or untrusted file text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapshotError {
    /// Configuration home cannot be resolved.
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

/// Exact public identity and certificate content used to fence publication.
/// Bearer refreshes and the passage of time do not change this value. A proof
/// refresh changes the stamp but need not tear down readers for the same account/key.
#[derive(Clone, PartialEq, Eq)]
pub struct IdentityStamp {
    auth_account_id: String,
    device_key: PublicKey,
    leaf: Option<Vec<u8>>,
    chain: Option<Vec<u8>>,
}

/// One complete read transaction. Deliberately has no Debug or serialization.
pub struct IdentitySnapshot {
    account: AccountId,
    seed: [u8; 32],
    owner_chain: Option<CachedOwnerChain>,
    stamp: IdentityStamp,
}

impl IdentitySnapshot {
    /// Original account-service identity retained in auth.json.
    pub fn auth_account_id(&self) -> &str {
        &self.stamp.auth_account_id
    }
    /// Pairing identity mapped from the original account-service id.
    pub fn account(&self) -> AccountId {
        self.account
    }
    /// Public half derived from the actual persisted seed.
    pub fn device_key(&self) -> PublicKey {
        self.stamp.device_key
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
    /// Build account-bound dial configuration without exposing the seed or opening
    /// an endpoint. The controller owns later plane creation and identity fencing.
    pub fn wan_registry(
        &self,
        robots: std::collections::HashMap<String, crate::wan::WanRobot>,
        relay: cerulion_link::RelayConfig,
        epoch_dir: Option<std::path::PathBuf>,
    ) -> Result<crate::wan::WanRegistry, String> {
        let registry = crate::wan::WanRegistry::new(robots, self.seed, relay)
            .with_account(Some(self.account.0))
            .with_epoch_dir(epoch_dir);
        registry.replace_owner_certificate(self.owner_chain().cloned().map(std::sync::Arc::new))?;
        Ok(registry)
    }
}

/// Load from the shared login configuration home. Never creates local state.
pub fn load() -> Result<IdentitySnapshot, SnapshotError> {
    let home = cerulion_discovery::robot_state::config_dir().ok_or(SnapshotError::NoConfigHome)?;
    load_at(&home)
}

/// Read auth, key, leaf and chain under the login writer's shared store lock.
/// Valid leaf-only legacy logins and certificate-free logins return no proof.
/// Present corrupt/mismatched material always refuses the entire snapshot.
pub fn load_at(home: &Path) -> Result<IdentitySnapshot, SnapshotError> {
    let _lock = lock_existing(&home.join(STORE_LOCK_FILE))?;
    let auth_account_id =
        serving_login::read_identity_at(&home.join("auth.json")).map_err(SnapshotError::Auth)?;
    let account =
        identity_cache::pairing_account_id(&auth_account_id).map_err(|_| SnapshotError::Account)?;
    let seed: [u8; 32] = serving_login::read_regular_bounded(&home.join("desk.key"), 32)
        .map_err(SnapshotError::Key)?
        .try_into()
        .map_err(|_| SnapshotError::KeySize)?;
    let device_key = DeviceIdentity::from_seed(&seed).public_key();
    let leaf = optional_file(&home.join("device.cert")).map_err(SnapshotError::Certificate)?;
    let chain = optional_file(&home.join("device-chain.json")).map_err(SnapshotError::Chain)?;
    let owner_chain = match (leaf.as_deref(), chain.as_deref()) {
        (None, None) => None,
        (None, Some(_)) => return Err(SnapshotError::IncompleteChain),
        (Some(leaf), chain) => identity_cache::decode_owner_chain(leaf, chain, account, device_key)
            .map_err(|_| SnapshotError::InvalidCertificate)?,
    };
    Ok(IdentitySnapshot {
        account,
        seed,
        owner_chain,
        stamp: IdentityStamp {
            auth_account_id,
            device_key,
            leaf,
            chain,
        },
    })
}

fn optional_file(path: &Path) -> Result<Option<Vec<u8>>, LoginProblem> {
    match serving_login::read_regular_bounded(path, MAX_CERT_BYTES) {
        Ok(bytes) => Ok(Some(bytes)),
        Err(LoginProblem::Missing) => match std::fs::symlink_metadata(path) {
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
            _ => Err(LoginProblem::Unreadable),
        },
        Err(error) => Err(error),
    }
}

fn lock_existing(path: &Path) -> Result<File, SnapshotError> {
    let mut options = OpenOptions::new();
    options.read(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags(libc::O_NONBLOCK);
    }
    let file = options.open(path).map_err(|error| {
        SnapshotError::Lock(if error.kind() == std::io::ErrorKind::NotFound {
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
