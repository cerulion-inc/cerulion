// SPDX-License-Identifier: AGPL-3.0-only
//! Automatic account-backed provisioning from a private registration bundle.
//!
//! The bundle is an internal process boundary, never an operator command. It
//! contains signed account-service artifacts and a path to the login device key.
//! Existing claimed state is checked offline; partial state is never replaced.

use std::path::PathBuf;

use cerulion_pairing::format::{AccountId, PublicKey, RobotId};

use crate::RemotedError;

/// Private writer inputs. Secret key bytes never enter command arguments.
pub struct ProvisionConfig {
    pub state_root: PathBuf,
    pub registration_bundle: PathBuf,
}

/// Public identity facts returned to the automatic bootstrap worker.
#[derive(Debug, PartialEq, Eq)]
pub struct Provisioned {
    pub robot_id: RobotId,
    pub endpoint_id: PublicKey,
    pub owner_account: AccountId,
}

/// Provision a claimed robot, or validate an already complete matching install.
/// A Tokio I/O runtime is needed only to read the effective UID through a local
/// anonymous socket pair. This opens no listening or remote network endpoint.
#[cfg(unix)]
pub fn provision(config: &ProvisionConfig, now_ns: u64) -> Result<Provisioned, RemotedError> {
    unix::provision(config, now_ns)
}

/// Provisioning currently requires Unix ownership and permission semantics.
#[cfg(not(unix))]
pub fn provision(_config: &ProvisionConfig, _now_ns: u64) -> Result<Provisioned, RemotedError> {
    Err(RemotedError::Provision("provisioning requires Unix".into()))
}

#[cfg(unix)]
mod unix {
    use super::*;
    use std::fs::{self, File, Metadata, OpenOptions};
    use std::io::{Read, Write};
    use std::os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt};
    use std::path::{Component, Path};

    use cerulion_pairing::client::DeviceIdentity;
    use cerulion_pairing::format::{
        RootSet, SignedDeviceCert, SignedGrant, SignedIntermediateCert, FORMAT_VERSION,
    };
    use cerulion_pairing::verify::{PairingPresentation, TrustStore};
    use serde::{Deserialize, Serialize};

    use crate::config::{
        DEVICE_INDEX_FILENAME, DEVICE_KEY_FILENAME, REMOTED_SUBDIR, TRUST_STORE_FILENAME,
        TRUST_STORE_MAC_KEY_FILENAME,
    };
    use crate::DeviceAccountIndex;

    const CHASSIS_SECRET_FILENAME: &str = "chassis_secret";
    const MAX_BUNDLE_BYTES: usize = 256 * 1024;
    const MAX_STATE_BYTES: usize = 16 * 1024 * 1024;

    // The hidden CLI worker emits this exact private JSON shape. Certificate
    // signatures use byte-oriented serde, so their wire containers are postcard.
    // No Debug implementation: this boundary includes private filesystem paths.
    #[derive(Serialize, Deserialize)]
    #[serde(deny_unknown_fields)]
    struct RegistrationBundle {
        version: u16,
        robot_id: String,
        owner_account: String,
        device_key_file: PathBuf,
        root_set_postcard: String,
        intermediate_postcard: String,
        device_cert_postcard: String,
        owner_grant_postcard: String,
    }

    struct Registration {
        robot: RobotId,
        owner: AccountId,
        seed: [u8; 32],
        roots: RootSet,
        presentation: PairingPresentation,
    }

    fn refused(message: &'static str) -> RemotedError {
        RemotedError::Provision(message.into())
    }

    fn random32() -> Result<[u8; 32], RemotedError> {
        let mut bytes = [0; 32];
        getrandom::getrandom(&mut bytes)
            .map_err(|_| refused("operating system entropy unavailable"))?;
        Ok(bytes)
    }

    fn current_uid() -> Result<u32, RemotedError> {
        tokio::runtime::Handle::try_current()
            .map_err(|_| refused("provisioning requires a Tokio I/O runtime"))?;
        let (socket, _peer) = tokio::net::UnixStream::pair()?;
        Ok(socket.peer_cred()?.uid())
    }

    // Reject links and writable ancestry before using path-based filesystem APIs.
    // The threat boundary excludes processes already controlling this UID or root.
    fn trusted_ancestry(path: &Path, uid: u32) -> Result<(), RemotedError> {
        if !path.is_absolute()
            || path
                .components()
                .any(|part| matches!(part, Component::ParentDir | Component::CurDir))
        {
            return Err(refused(
                "provisioning paths must be absolute without traversal",
            ));
        }
        for ancestor in path.ancestors() {
            match fs::symlink_metadata(ancestor) {
                Ok(meta) => {
                    let sticky_system_dir = meta.uid() == 0 && meta.mode() & 0o1000 != 0;
                    if !meta.is_dir()
                        || (meta.uid() != uid && meta.uid() != 0)
                        || (meta.mode() & 0o022 != 0 && !sticky_system_dir)
                    {
                        return Err(refused("provisioning requires trusted directory ancestry"));
                    }
                }
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => return Err(error.into()),
            }
        }
        Ok(())
    }

    fn private_directory(path: &Path, uid: u32) -> Result<(), RemotedError> {
        let meta = fs::symlink_metadata(path)?;
        if !meta.is_dir() || meta.uid() != uid || meta.mode() & 0o077 != 0 {
            return Err(refused("state directories must be owner-only"));
        }
        Ok(())
    }

    fn create_private_root(path: &Path, uid: u32) -> Result<(), RemotedError> {
        trusted_ancestry(path, uid)?;
        let mut missing = Vec::new();
        for ancestor in path.ancestors() {
            match fs::symlink_metadata(ancestor) {
                Ok(_) => break,
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                    missing.push(ancestor)
                }
                Err(error) => return Err(error.into()),
            }
        }
        for directory in missing.into_iter().rev() {
            fs::DirBuilder::new().mode(0o700).create(directory)?;
            sync_directory(
                directory
                    .parent()
                    .ok_or_else(|| refused("state root has no parent"))?,
            )?;
        }
        private_directory(path, uid)
    }

    fn checked_file(path: &Path, uid: u32, secret: bool) -> Result<(File, Metadata), RemotedError> {
        let parent = path
            .parent()
            .ok_or_else(|| refused("input file has no parent"))?;
        trusted_ancestry(parent, uid)?;
        let mut options = OpenOptions::new();
        options
            .read(true)
            .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK);
        let file = options.open(path)?;
        let meta = file.metadata()?;
        if !meta.is_file() || (meta.uid() != uid && meta.uid() != 0) {
            return Err(refused("input must be a trusted regular file"));
        }
        if (secret && meta.mode() & 0o777 != 0o600) || (!secret && meta.mode() & 0o022 != 0) {
            return Err(refused(
                "secret inputs require mode 0600; public inputs must not be writable by others",
            ));
        }
        Ok((file, meta))
    }

    fn read_bounded(
        path: &Path,
        uid: u32,
        secret: bool,
        limit: usize,
    ) -> Result<Vec<u8>, RemotedError> {
        let (file, meta) = checked_file(path, uid, secret)?;
        if meta.len() > limit as u64 {
            return Err(refused("input file exceeds its size limit"));
        }
        let mut bytes = Vec::new();
        file.take(limit as u64 + 1).read_to_end(&mut bytes)?;
        if bytes.len() > limit {
            return Err(refused("input file exceeds its size limit"));
        }
        Ok(bytes)
    }

    fn write_new(path: &Path, bytes: &[u8]) -> Result<(), RemotedError> {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(path)?;
        file.write_all(bytes)?;
        file.sync_all()?;
        Ok(())
    }

    fn sync_directory(path: &Path) -> Result<(), RemotedError> {
        File::open(path)?.sync_all()?;
        Ok(())
    }

    // The existing serializers use this sibling temporary filename. Reserving it
    // exclusively in our fresh private directory prevents link following and
    // preserves mode 0600 when those serializers write and rename it.
    fn reserve_save_temp(path: &Path) -> Result<(), RemotedError> {
        let mut filename = path
            .file_name()
            .ok_or_else(|| refused("state file has no name"))?
            .to_os_string();
        filename.push(".tmp");
        write_new(&path.with_file_name(filename), &[])
    }

    fn decode<T: serde::de::DeserializeOwned>(encoded: &str) -> Result<T, RemotedError> {
        let bytes =
            hex::decode(encoded).map_err(|_| refused("registration artifact is not hex"))?;
        let (value, rest) = postcard::take_from_bytes(&bytes)
            .map_err(|_| refused("registration artifact is invalid postcard"))?;
        if !rest.is_empty() {
            return Err(refused("registration artifact has trailing bytes"));
        }
        Ok(value)
    }

    fn key32(encoded: &str) -> Result<[u8; 32], RemotedError> {
        let mut key = [0; 32];
        hex::decode_to_slice(encoded, &mut key)
            .map_err(|_| refused("registration identity must contain exactly 64 hex characters"))?;
        Ok(key)
    }

    fn read_registration(config: &ProvisionConfig, uid: u32) -> Result<Registration, RemotedError> {
        if !config.registration_bundle.starts_with(&config.state_root) {
            return Err(refused("registration bundle must be inside the state root"));
        }
        let bytes = read_bounded(&config.registration_bundle, uid, true, MAX_BUNDLE_BYTES)?;
        let bundle: RegistrationBundle =
            serde_json::from_slice(&bytes).map_err(|_| refused("invalid registration bundle"))?;
        if bundle.version != 1 {
            return Err(refused("unsupported registration bundle version"));
        }
        let seed = read_bounded(&bundle.device_key_file, uid, true, 32)?
            .try_into()
            .map_err(|_| refused("login device key must contain exactly 32 raw bytes"))?;
        let roots: RootSet = decode(&bundle.root_set_postcard)?;
        if roots.version != FORMAT_VERSION {
            return Err(refused("unsupported trusted root set version"));
        }
        roots.validate().map_err(RemotedError::Store)?;
        let intermediate: SignedIntermediateCert = decode(&bundle.intermediate_postcard)?;
        let device_cert: SignedDeviceCert = decode(&bundle.device_cert_postcard)?;
        let grant: SignedGrant = decode(&bundle.owner_grant_postcard)?;
        Ok(Registration {
            robot: RobotId(key32(&bundle.robot_id)?),
            owner: AccountId(key32(&bundle.owner_account)?),
            seed,
            roots,
            presentation: PairingPresentation {
                intermediate,
                device_cert,
                grant,
                delegation: None,
            },
        })
    }

    fn identity(registration: &Registration) -> Provisioned {
        Provisioned {
            robot_id: registration.robot,
            endpoint_id: DeviceIdentity::from_seed(&registration.seed).public_key(),
            owner_account: registration.owner,
        }
    }

    fn verify_identity(store: &TrustStore, expected: &Provisioned) -> Result<(), RemotedError> {
        if store.robot_id() != expected.robot_id
            || store.robot_transport_key() != expected.endpoint_id
            || store.owner() != Some(expected.owner_account)
        {
            return Err(refused(
                "robot state does not match the registered identity and owner",
            ));
        }
        Ok(())
    }

    fn existing_state(dir: &Path, uid: u32, expected: &Provisioned) -> Result<(), RemotedError> {
        private_directory(dir, uid)?;
        let seed: [u8; 32] = read_bounded(&dir.join(DEVICE_KEY_FILENAME), uid, true, 32)?
            .try_into()
            .map_err(|_| refused("stored device key must contain exactly 32 raw bytes"))?;
        if DeviceIdentity::from_seed(&seed).public_key() != expected.endpoint_id {
            return Err(refused(
                "registered login key does not match the existing robot",
            ));
        }
        let mac = read_bounded(&dir.join(TRUST_STORE_MAC_KEY_FILENAME), uid, true, 32)?;
        if mac.len() != 32
            || read_bounded(&dir.join(CHASSIS_SECRET_FILENAME), uid, true, 32)?.len() != 32
        {
            return Err(refused("existing robot secret files are incomplete"));
        }
        let store_path = dir.join(TRUST_STORE_FILENAME);
        let index_path = dir.join(DEVICE_INDEX_FILENAME);
        read_bounded(&store_path, uid, true, MAX_STATE_BYTES)?;
        read_bounded(&index_path, uid, true, MAX_STATE_BYTES)?;
        let store = TrustStore::load(store_path, &mac).map_err(RemotedError::Store)?;
        verify_identity(&store, expected)?;
        let index = DeviceAccountIndex::load(index_path, &mac)?;
        if index.account_for(&expected.endpoint_id) != Some(expected.owner_account) {
            return Err(refused(
                "stored owner device binding is incomplete or mismatched",
            ));
        }
        Ok(())
    }

    fn write_state(
        dir: &Path,
        registration: &Registration,
        secret: &[u8; 32],
        mac: &[u8; 32],
        store: TrustStore,
    ) -> Result<(), RemotedError> {
        let expected = identity(registration);
        verify_identity(&store, &expected)?;
        for name in [
            DEVICE_KEY_FILENAME,
            TRUST_STORE_MAC_KEY_FILENAME,
            CHASSIS_SECRET_FILENAME,
            DEVICE_INDEX_FILENAME,
            "device_index.json.tmp",
            TRUST_STORE_FILENAME,
            ".trust_store.provision",
            ".trust_store.provision.tmp",
        ] {
            match fs::symlink_metadata(dir.join(name)) {
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => return Err(error.into()),
                Ok(_) => return Err(refused("partial robot state is never replaced")),
            }
        }
        write_new(&dir.join(DEVICE_KEY_FILENAME), &registration.seed)?;
        write_new(&dir.join(TRUST_STORE_MAC_KEY_FILENAME), mac)?;
        write_new(&dir.join(CHASSIS_SECRET_FILENAME), secret)?;
        // Persist and verify the CLAIMED store before the device index. The final
        // trust_store filename is the commit marker and does not exist yet.
        let pending = dir.join(".trust_store.provision");
        reserve_save_temp(&pending)?;
        store
            .with_path(&pending)
            .save(mac)
            .map_err(RemotedError::Store)?;
        File::open(&pending)?.sync_all()?;
        let loaded = TrustStore::load(&pending, mac).map_err(RemotedError::Store)?;
        verify_identity(&loaded, &expected)?;
        let index_path = dir.join(DEVICE_INDEX_FILENAME);
        reserve_save_temp(&index_path)?;
        let mut index = DeviceAccountIndex::new().with_path(&index_path);
        index.bind(expected.endpoint_id, expected.owner_account);
        index.save(mac)?;
        File::open(&index_path)?.sync_all()?;
        sync_directory(dir)?;
        // An exclusive same-directory link publishes the complete marker without
        // rename's overwrite behavior. Only then remove the staging name.
        fs::hard_link(&pending, dir.join(TRUST_STORE_FILENAME))?;
        fs::remove_file(&pending)?;
        sync_directory(dir)
    }

    fn claimed_store(
        registration: &Registration,
        secret: &[u8; 32],
        now_ns: u64,
    ) -> Result<TrustStore, RemotedError> {
        let endpoint = DeviceIdentity::from_seed(&registration.seed).public_key();
        let mut store = TrustStore::provision(
            registration.robot,
            endpoint,
            registration.roots.clone(),
            secret,
            now_ns,
        )
        .map_err(RemotedError::Store)?;
        let owner = store
            .claim_by_owner_grant(&registration.presentation, &endpoint, now_ns)
            .map_err(RemotedError::Store)?;
        if owner != registration.owner {
            return Err(refused("registration grant names a different login owner"));
        }
        Ok(store)
    }

    pub(super) fn provision(
        config: &ProvisionConfig,
        now_ns: u64,
    ) -> Result<Provisioned, RemotedError> {
        let uid = current_uid()?;
        trusted_ancestry(&config.state_root, uid)?;
        private_directory(&config.state_root, uid)?;
        let registration = read_registration(config, uid)?;
        let expected = identity(&registration);
        let dir = config.state_root.join(REMOTED_SUBDIR);
        match fs::symlink_metadata(&dir) {
            Ok(_) => {
                // Durable registration survives certificate expiry and cloud
                // outages. This branch validates local MACs/identity only.
                existing_state(&dir, uid, &expected)?;
                return Ok(expected);
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
        let secret = random32()?;
        let mac = random32()?;
        let store = claimed_store(&registration, &secret, now_ns)?;
        create_private_root(&config.state_root, uid)?;
        fs::DirBuilder::new().mode(0o700).create(&dir)
            .map_err(|_| refused("robot state already exists or cannot be created; partial state is never replaced"))?;
        sync_directory(&config.state_root)?;
        write_state(&dir, &registration, &secret, &mac, store)?;
        Ok(expected)
    }

    #[cfg(test)]
    mod tests;
}
