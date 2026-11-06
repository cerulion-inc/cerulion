// SPDX-License-Identifier: MIT OR Apache-2.0
//! A development team issuer (feature `dev-issuer`): a local root set, an
//! intermediate, and one team's issuing CAs for the current and the next
//! generation, kept in `<config>/dev-issuer/state` (0600), from which this
//! machine's leaf and the team trust bundle are issued.
//!
//! Every key in the state file is a secret. A second machine joins the same
//! development team by copying that file into its own `<config>/dev-issuer/`
//! (or [`DevIssuer::import`]) and enrolling.
//!
//! [`trusted_root_sets`] returns the development root set from the state file,
//! so a build with this feature accepts team credentials that chain to it. A
//! build without the feature compiles none of this module, and its
//! [`trusted_root_sets`] returns the release root sets only.
//!
//! The intermediate and the bundle are issued at the Unix epoch and the
//! intermediate is valid until `u64::MAX` nanoseconds (exclusive), so a
//! machine clock that is behind, or moves back after enrollment, still loads
//! the credentials. Leaves carry the role lifetimes of `team::issue_leaf`.

use std::path::{Path, PathBuf};

use ed25519_dalek::SigningKey;
use serde::{Deserialize, Serialize};
use zeroize::Zeroize;

use super::{
    issue_leaf, issue_team_ca, LeafParams, SignedTeamTrustBundle, TeamId, TeamRole, TeamShort,
    TeamTrustBundle,
};
use crate::format::{IntermediateCert, PublicKey, RootSet, Scope, Validity, FORMAT_VERSION};
use crate::identity::{create_secret_dir, io_err, write_new_private, IdentityLayout, TeamDirName};
use crate::PairingError;

/// The directory under the config directory that holds the state file.
pub const DEV_ISSUER_DIR: &str = "dev-issuer";
/// The state file name.
pub const DEV_ISSUER_STATE_FILENAME: &str = "state";
const STATE_VERSION: u16 = 1;

#[derive(Serialize, Deserialize)]
struct State {
    version: u16,
    team: TeamShort,
    team_id: TeamId,
    generation: u32,
    created_at_ns: u64,
    root_seed: [u8; 32],
    intermediate_seed: [u8; 32],
    ca_seed: [u8; 32],
    next_ca_seed: [u8; 32],
}

impl Drop for State {
    fn drop(&mut self) {
        self.root_seed.zeroize();
        self.intermediate_seed.zeroize();
        self.ca_seed.zeroize();
        self.next_ca_seed.zeroize();
    }
}

/// A loaded development issuer.
pub struct DevIssuer {
    state: State,
}

impl core::fmt::Debug for DevIssuer {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DevIssuer")
            .field("team", &self.state.team)
            .field("generation", &self.state.generation)
            .finish_non_exhaustive()
    }
}

fn random_seed(path: &Path) -> Result<[u8; 32], PairingError> {
    let mut seed = [0u8; 32];
    getrandom::fill(&mut seed).map_err(|e| {
        io_err(
            "generating a key for",
            path,
            std::io::Error::other(e.to_string()),
        )
    })?;
    Ok(seed)
}

fn key(seed: &[u8; 32]) -> SigningKey {
    SigningKey::from_bytes(seed)
}

fn public(key: &SigningKey) -> PublicKey {
    PublicKey(key.verifying_key().to_bytes())
}

/// Decode and version-check state file bytes read from `path`.
fn decode_state(path: &Path, bytes: &[u8]) -> Result<State, PairingError> {
    let state =
        postcard::from_bytes::<State>(bytes).map_err(|_| PairingError::TeamCredentials {
            path: path.to_path_buf(),
            reason: "does not decode as a development issuer state file".into(),
        })?;
    if state.version != STATE_VERSION {
        return Err(PairingError::TeamCredentials {
            path: path.to_path_buf(),
            reason: format!(
                "is development issuer state version {}, this build reads {STATE_VERSION}",
                state.version
            ),
        });
    }
    Ok(state)
}

/// Refuse a state file that is a symbolic link, and clear the group and
/// other permission bits of one that has them, on Unix.
fn owner_only(path: &Path) -> Result<(), PairingError> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let meta = std::fs::symlink_metadata(path).map_err(|e| io_err("reading", path, e))?;
        if meta.file_type().is_symlink() {
            return Err(PairingError::TeamCredentials {
                path: path.to_path_buf(),
                reason: "is a symbolic link; copy the state file itself".into(),
            });
        }
        if meta.permissions().mode() & 0o077 != 0 {
            std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))
                .map_err(|e| io_err("restricting", path, e))?;
        }
    }
    #[cfg(not(unix))]
    let _ = path;
    Ok(())
}

/// `<config>/dev-issuer`.
pub fn dev_issuer_dir(layout: &IdentityLayout) -> PathBuf {
    layout.config_dir().join(DEV_ISSUER_DIR)
}

fn state_path(layout: &IdentityLayout) -> PathBuf {
    dev_issuer_dir(layout).join(DEV_ISSUER_STATE_FILENAME)
}

impl DevIssuer {
    /// Create a development issuer for `team` at generation 1, refusing an
    /// existing state file.
    ///
    /// # Errors
    ///
    /// [`PairingError::IdentityIo`] (an existing state file is `AlreadyExists`).
    pub fn create(
        layout: &IdentityLayout,
        team: TeamShort,
        now_ns: u64,
    ) -> Result<DevIssuer, PairingError> {
        let path = state_path(layout);
        create_secret_dir(&dev_issuer_dir(layout))?;
        let state = State {
            version: STATE_VERSION,
            team,
            team_id: TeamId(random_seed(&path)?),
            generation: 1,
            created_at_ns: now_ns,
            root_seed: random_seed(&path)?,
            intermediate_seed: random_seed(&path)?,
            ca_seed: random_seed(&path)?,
            next_ca_seed: random_seed(&path)?,
        };
        let mut bytes =
            postcard::to_allocvec(&state).map_err(|_| PairingError::TeamCredentials {
                path: path.clone(),
                reason: "the development issuer state does not encode".into(),
            })?;
        let written = write_new_private(&path, &bytes);
        bytes.zeroize();
        written?;
        Ok(DevIssuer { state })
    }

    /// Load the development issuer, or `None` when there is no state file.
    /// A state file readable by group or others is restricted to its owner
    /// (0600) first.
    ///
    /// # Errors
    ///
    /// [`PairingError::IdentityIo`] or [`PairingError::TeamCredentials`] for a
    /// state file that does not decode.
    pub fn load(layout: &IdentityLayout) -> Result<Option<DevIssuer>, PairingError> {
        let path = state_path(layout);
        let mut bytes = match std::fs::read(&path) {
            Ok(b) => b,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(e) => return Err(io_err("reading", &path, e)),
        };
        let decoded = owner_only(&path).and_then(|()| decode_state(&path, &bytes));
        bytes.zeroize();
        Ok(Some(DevIssuer { state: decoded? }))
    }

    /// Copy another machine's state file (`from`, the file itself) into this
    /// layout, refusing an existing one, and load it.
    ///
    /// # Errors
    ///
    /// As [`DevIssuer::load`], plus `AlreadyExists` for an existing state file.
    pub fn import(layout: &IdentityLayout, from: &Path) -> Result<DevIssuer, PairingError> {
        let mut bytes = std::fs::read(from).map_err(|e| io_err("reading", from, e))?;
        let written = decode_state(from, &bytes).and_then(|state| {
            create_secret_dir(&dev_issuer_dir(layout))?;
            write_new_private(&state_path(layout), &bytes)?;
            Ok(state)
        });
        bytes.zeroize();
        Ok(DevIssuer { state: written? })
    }

    /// The team named in a state file at `from`, read without importing it.
    ///
    /// # Errors
    ///
    /// As [`DevIssuer::import`] for a file that does not decode.
    pub fn team_of_state_file(from: &Path) -> Result<TeamShort, PairingError> {
        let mut bytes = std::fs::read(from).map_err(|e| io_err("reading", from, e))?;
        let decoded = decode_state(from, &bytes);
        bytes.zeroize();
        Ok(decoded?.team.clone())
    }

    /// The team short name.
    pub fn team(&self) -> &TeamShort {
        &self.state.team
    }

    /// The current generation.
    pub fn generation(&self) -> u32 {
        self.state.generation
    }

    /// The development root set: one root key, threshold one.
    pub fn root_set(&self) -> RootSet {
        RootSet {
            version: FORMAT_VERSION,
            keys: vec![public(&key(&self.state.root_seed))],
            threshold: 1,
        }
    }

    fn ca_der(&self, generation: u32, seed: &[u8; 32]) -> Result<Vec<u8>, PairingError> {
        issue_team_ca(
            &self.state.team,
            generation,
            &key(seed),
            self.state.created_at_ns,
        )
    }

    /// The team trust bundle: the current issuing CA, the next generation's
    /// CA, no retired generations and no robot leaves, issued at the epoch.
    ///
    /// # Errors
    ///
    /// A certificate encoding error from [`issue_team_ca`].
    pub fn bundle(&self) -> Result<SignedTeamTrustBundle, PairingError> {
        let int = key(&self.state.intermediate_seed);
        let intermediate = IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: public(&int),
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: u64::MAX,
            },
            issued_at_ns: 0,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&key(&self.state.root_seed)]);
        let bundle = TeamTrustBundle {
            version: 1,
            team_id: self.state.team_id,
            team_short: self.state.team.clone(),
            generation: self.state.generation,
            issued_at_ns: 0,
            issuing_ca_der: self.ca_der(self.state.generation, &self.state.ca_seed)?,
            next_ca_der: self.ca_der(self.state.generation + 1, &self.state.next_ca_seed)?,
            retired: Vec::new(),
            robot_leaves: Vec::new(),
            issuer_key: public(&int),
        };
        Ok(bundle.sign(&int, intermediate))
    }

    /// A leaf for `device_key` in `role`, issued at `now_ns`.
    ///
    /// # Errors
    ///
    /// A certificate encoding error from [`issue_leaf`].
    pub fn issue_leaf(
        &self,
        role: TeamRole,
        device_key: PublicKey,
        now_ns: u64,
    ) -> Result<Vec<u8>, PairingError> {
        issue_leaf(
            &LeafParams {
                team: self.state.team.clone(),
                generation: self.state.generation,
                role,
                device_key,
                account: None,
                issued_at_ns: now_ns,
            },
            &key(&self.state.ca_seed),
        )
    }

    /// Issue this machine a leaf in `role` (creating the machine key when
    /// there is none), write the leaf and the bundle into its team directory
    /// and make the team active. Returns the team directory name.
    ///
    /// # Errors
    ///
    /// The machine key, certificate and file errors of the steps.
    pub fn enroll(
        &self,
        layout: &IdentityLayout,
        role: TeamRole,
        now_ns: u64,
    ) -> Result<TeamDirName, PairingError> {
        let (machine, _) = layout.load_or_create_machine_key()?;
        let leaf = self.issue_leaf(role, PublicKey(machine.public_key()), now_ns)?;
        let team = TeamDirName::new(self.state.team.as_str())?;
        layout.write_team_credentials(&team, &leaf, &self.bundle()?)?;
        layout.set_active_team(&team)?;
        Ok(team)
    }
}

/// The root sets team credentials are checked against: the release root
/// sets, plus the development root set when this layout holds a development
/// issuer.
///
/// # Errors
///
/// As [`DevIssuer::load`].
pub fn trusted_root_sets(layout: &IdentityLayout) -> Result<Vec<RootSet>, PairingError> {
    let mut sets = super::release_root_sets();
    if let Some(dev) = DevIssuer::load(layout)? {
        tracing::warn!(
            dir = %dev_issuer_dir(layout).display(),
            team = %dev.team(),
            "this build trusts team credentials from a development issuer; release builds do not"
        );
        sets.push(dev.root_set());
    }
    Ok(sets)
}
