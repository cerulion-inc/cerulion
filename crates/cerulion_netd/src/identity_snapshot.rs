// SPDX-License-Identifier: AGPL-3.0-only
//! Consistent local identity for the account controller, without network or writes.
//!
//! The shared reader holds the login writer's existing lock across auth, key and
//! proof reads. Netd retains its full-snapshot behavior, including refusing
//! malformed present proofs; expiry never invalidates a durable identity.

use crate::serving_login;
use cerulion_pairing::format::{AccountId, PublicKey};
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use cerulion_wireclient::local_identity;
use std::path::Path;

use cerulion_discovery::robot_state::AUTH_STORE_LOCK_FILE as STORE_LOCK_FILE;
pub use cerulion_wireclient::local_identity::{IdentityStamp, SnapshotError};

#[cfg(test)]
use crate::serving_login::LoginProblem;
#[cfg(test)]
use std::fs::{File, OpenOptions};
#[cfg(test)]
const MAX_CERT_BYTES: u64 = 64 * 1024;

/// One complete read transaction. Deliberately has no Debug or serialization.
pub struct IdentitySnapshot(local_identity::IdentitySnapshot);

impl IdentitySnapshot {
    /// Original account-service identity retained in auth.json.
    pub fn auth_account_id(&self) -> &str {
        self.0.auth_account_id()
    }
    /// Pairing identity mapped from the original account-service id.
    pub fn account(&self) -> AccountId {
        self.0.account()
    }
    /// Public half derived from the actual persisted seed.
    pub fn device_key(&self) -> PublicKey {
        self.0.device_key()
    }
    /// Structurally consistent public proof, including a possibly expired proof.
    pub fn owner_chain(&self) -> Option<&OwnerCertificatePresentationWire> {
        self.0.owner_chain()
    }
    /// Canonical public envelope bytes, matching the CLI's snapshot encoding.
    pub fn owner_chain_wire(&self) -> Option<&[u8]> {
        self.0.owner_chain_wire()
    }
    /// Comparison stamp for an in-flight local identity operation.
    pub fn stamp(&self) -> &IdentityStamp {
        self.0.stamp()
    }
    /// Build account-bound dial configuration without exposing the seed or opening
    /// an endpoint. The controller owns later plane creation and identity fencing.
    pub fn wan_registry(
        &self,
        robots: std::collections::HashMap<String, crate::wan::WanRobot>,
        relay: cerulion_link::RelayConfig,
        epoch_dir: Option<std::path::PathBuf>,
    ) -> Result<crate::wan::WanRegistry, String> {
        let registry = crate::wan::WanRegistry::new(robots, *self.0.seed(), relay)
            .with_account(Some(self.account().0))
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
    local_identity::load_with_proof(
        home,
        &home.join(STORE_LOCK_FILE),
        &serving_login::open_read_only,
    )
    .map(IdentitySnapshot)
}

#[cfg(test)]
mod tests;
