// SPDX-License-Identifier: AGPL-3.0-only
//! Private first-serve registration worker. Existing registration is checked offline.

use std::fs::{self, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt};
use std::path::{Component, Path, PathBuf};
use std::time::Duration;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::{
    PublicKey, RobotId, RootSet, SignedDeviceCert, SignedGrant, SignedIntermediateCert,
    FORMAT_VERSION,
};
use cerulion_pairing::verify::{PairingPresentation, TrustStore};
use serde::{Deserialize, Serialize};

use crate::error::{CliError, CliResult};
use crate::{account_cmd, account_identity, auth, login_cmd, owner_certificate, robot_cmd};

const BUNDLE_FILE: &str = "robot-registration.json";
const MAX_BUNDLE_BYTES: usize = 256 * 1024;

/// Public identity facts and the private handoff location; never contains a key seed.
#[derive(Serialize, Deserialize, PartialEq, Eq, Debug)]
#[serde(deny_unknown_fields)]
pub struct BootstrapPrepared {
    pub version: u16,
    pub registration_bundle: PathBuf,
    pub robot_id: String,
    pub endpoint_id: String,
    pub owner_account: String,
}

// Exact versioned process boundary consumed by cerulion-remoted's private writer.
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

struct LoginSnapshot {
    state: auth::AuthState,
    key_file: PathBuf,
    key: [u8; 32],
}

/// Run the private worker and write its versioned machine result.
pub fn run(state_root: &Path, out: &mut impl Write) -> CliResult<()> {
    let prepared = prepare(state_root)?;
    serde_json::to_writer(&mut *out, &prepared)
        .map_err(|_| refused("could not encode the worker result"))?;
    writeln!(out)?;
    Ok(())
}

fn refused(message: &str) -> CliError {
    CliError::Login(format!("automatic robot registration: {message}"))
}

fn login_snapshot() -> CliResult<LoginSnapshot> {
    let auth_path = auth::auth_json_path().ok_or_else(|| refused("no account directory"))?;
    auth::with_store_lock(&auth_path, || {
        Ok((|| {
            let loaded = auth::load_from(&auth_path);
            let state = loaded
                .state()
                .filter(|s| s.logged_in_ever)
                .ok_or_else(|| refused("run `cerulion login` before serving a robot"))?
                .clone();
            let key_file = auth::device_key_path().ok_or_else(|| refused("no login key path"))?;
            let seed: [u8; 32] = read_private(&key_file, 32)?
                .try_into()
                .map_err(|_| refused("the login key must contain exactly 32 bytes"))?;
            let key = ed25519_dalek::SigningKey::from_bytes(&seed)
                .verifying_key()
                .to_bytes();
            Ok(LoginSnapshot {
                state,
                key_file,
                key,
            })
        })())
    })?
}

fn same_login(before: &LoginSnapshot, after: &LoginSnapshot) -> bool {
    before.state == after.state && before.key_file == after.key_file && before.key == after.key
}

/// Prepare a verified account-service registration for the robot writer.
/// This worker never opens a listener or prompts in a background process.
pub fn prepare(state_root: &Path) -> CliResult<BootstrapPrepared> {
    let initial = login_snapshot()?;
    trusted_root(state_root)?;
    create_state_root(state_root)?;
    let metadata = fs::symlink_metadata(state_root)?;
    if metadata.uid() != cerulion_hygiene::euid() || metadata.mode() & 0o077 != 0 {
        return Err(refused("the robot state directory must be owner-only"));
    }
    let bundle_path = state_root.join(BUNDLE_FILE);
    auth::with_store_lock(&bundle_path, || Ok(prepare_locked(&bundle_path, initial)))?
}

fn prepare_locked(bundle_path: &Path, initial: LoginSnapshot) -> CliResult<BootstrapPrepared> {
    match fs::symlink_metadata(bundle_path) {
        Ok(_) => {
            let bundle: RegistrationBundle =
                serde_json::from_slice(&read_private(bundle_path, MAX_BUNDLE_BYTES)?).map_err(
                    |_| refused("cached registration is malformed; it was left untouched"),
                )?;
            let current = login_snapshot()?;
            if !same_login(&initial, &current) {
                return Err(refused(
                    "the login changed while reading robot registration",
                ));
            }
            return describe(&bundle, bundle_path, &current);
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => return Err(e.into()),
    }
    // A cloud request is necessary only for the first registration. Established
    // robots never refresh a session or fetch trust roots on this path.
    let service = login_cmd::account_service_base();
    let authority = trusted_service_url(&service)?;
    let session = account_cmd::require_session()?;
    let before = login_snapshot()?;
    if before.state.account_id != initial.state.account_id || before.key != initial.key {
        return Err(refused("the login changed before robot registration"));
    }
    if session != before.state.session_token {
        return Err(refused("the session changed before robot registration"));
    }
    let proof = owner_certificate::load()?;
    let hostname = cerulion_core::graph::robot_identity_from_env();
    let registration = robot_cmd::register_robot(&session, &hostname)?;
    if decode_id(&registration.account_id)?
        != account_identity::pairing_account_id(&before.state.account_id)?.0
    {
        return Err(refused("the service returned a different robot owner"));
    }
    let roots = fetch_roots(&authority)?;
    let intermediate: SignedIntermediateCert = decode_credential(&registration.intermediate)?;
    if intermediate.cert.intermediate_key != proof.device_cert.cert.issuer_key {
        return Err(refused("the issuer changed since login; run `cerulion login` again before registering this robot"));
    }
    let grant: SignedGrant = decode_credential(&registration.owner_grant)?;
    let robot = RobotId(decode_id(&registration.robot_id)?);
    let key = PublicKey(before.key);
    let now = auth::now_unix_ns();
    // This verifier-only store never escapes or touches disk. Its recovery hash
    // is unused; the robot writer generates the real private recovery secret.
    let mut verifier = TrustStore::provision(robot, key, roots.clone(), &[0; 32], now)
        .map_err(|e| refused(&format!("trusted registration cannot be verified: {e}")))?;
    let verified_owner = verifier.claim_by_owner_grant(&PairingPresentation {
        intermediate,
        device_cert: proof.device_cert.clone(),
        grant,
        delegation: None,
    }, &key, now).map_err(|e| refused(&format!("owner credentials were refused before caching: {e}; run `cerulion login` to renew them")))?;
    if verified_owner.0 != decode_id(&registration.account_id)? {
        return Err(refused(
            "verified robot ownership differs from the login account",
        ));
    }
    let bundle = RegistrationBundle {
        version: 1,
        robot_id: hex::encode(decode_id(&registration.robot_id)?),
        owner_account: hex::encode(decode_id(&registration.account_id)?),
        device_key_file: before.key_file.clone(),
        root_set_postcard: hex::encode(
            postcard::to_stdvec(&roots).map_err(|_| refused("could not encode trusted roots"))?,
        ),
        intermediate_postcard: decode_blob(&registration.intermediate)?,
        device_cert_postcard: hex::encode(
            postcard::to_stdvec(&proof.device_cert)
                .map_err(|_| refused("could not encode the device certificate"))?,
        ),
        owner_grant_postcard: decode_blob(&registration.owner_grant)?,
    };
    let auth_path = auth::auth_json_path().ok_or_else(|| refused("no account directory"))?;
    auth::with_store_lock(&auth_path, || {
        Ok((|| {
            let after = login_snapshot()?;
            if !same_login(&before, &after) || owner_certificate::load()? != proof {
                return Err(refused(
                    "the login changed during robot registration; no local ownership was published",
                ));
            }
            let prepared = describe(&bundle, bundle_path, &after)?;
            let bytes = serde_json::to_vec(&bundle)
                .map_err(|_| refused("could not encode robot registration"))?;
            if bytes.len() > MAX_BUNDLE_BYTES {
                return Err(refused("robot registration exceeds its size limit"));
            }
            auth::stage_secret(bundle_path, &bytes)?.commit_new()?;
            Ok(prepared)
        })())
    })?
}

fn describe(
    bundle: &RegistrationBundle,
    path: &Path,
    login: &LoginSnapshot,
) -> CliResult<BootstrapPrepared> {
    if bundle.version != 1
        || bundle.device_key_file != login.key_file
        || bundle.owner_account
            != hex::encode(account_identity::pairing_account_id(&login.state.account_id)?.0)
    {
        return Err(refused(
            "cached robot ownership does not match this login; existing state was left untouched",
        ));
    }
    let _: [u8; 32] = hex::decode(&bundle.robot_id)
        .ok()
        .and_then(|v| v.try_into().ok())
        .ok_or_else(|| refused("cached robot id is malformed"))?;
    let cert_bytes = hex::decode(&bundle.device_cert_postcard)
        .map_err(|_| refused("cached device certificate is malformed"))?;
    let (cert, rest): (SignedDeviceCert, _) = postcard::take_from_bytes(&cert_bytes)
        .map_err(|_| refused("cached device certificate is malformed"))?;
    if !rest.is_empty()
        || cert.cert.device_key.0 != login.key
        || hex::encode(cert.cert.account.0) != bundle.owner_account
    {
        return Err(refused(
            "cached robot key or account does not match this login",
        ));
    }
    Ok(BootstrapPrepared {
        version: 1,
        registration_bundle: path.to_path_buf(),
        robot_id: bundle.robot_id.clone(),
        endpoint_id: hex::encode(login.key),
        owner_account: bundle.owner_account.clone(),
    })
}

fn decode_id(encoded: &str) -> CliResult<[u8; 32]> {
    URL_SAFE_NO_PAD
        .decode(encoded)
        .ok()
        .and_then(|b| b.try_into().ok())
        .ok_or_else(|| refused("the account service returned an invalid identity"))
}

fn decode_blob(encoded: &str) -> CliResult<String> {
    URL_SAFE_NO_PAD
        .decode(encoded)
        .map(hex::encode)
        .map_err(|_| refused("the account service returned invalid credential encoding"))
}

fn decode_credential<T: serde::de::DeserializeOwned>(encoded: &str) -> CliResult<T> {
    if encoded.len() > MAX_BUNDLE_BYTES {
        return Err(refused("credential exceeds its size limit"));
    }
    let bytes = URL_SAFE_NO_PAD
        .decode(encoded)
        .map_err(|_| refused("credential encoding is invalid"))?;
    let (value, rest) =
        postcard::take_from_bytes(&bytes).map_err(|_| refused("credential is malformed"))?;
    if !rest.is_empty() {
        return Err(refused("credential contains trailing data"));
    }
    Ok(value)
}

fn trusted_service_url(service: &str) -> CliResult<reqwest::Url> {
    let url =
        reqwest::Url::parse(service).map_err(|_| refused("the account service URL is invalid"))?;
    let loopback = url
        .host_str()
        .and_then(|s| s.trim_matches(['[', ']']).parse::<std::net::IpAddr>().ok())
        .is_some_and(|ip| ip.is_loopback());
    if (url.scheme() != "https" && !(url.scheme() == "http" && loopback))
        || !url.username().is_empty()
        || url.password().is_some()
        || url.query().is_some()
        || url.fragment().is_some()
    {
        return Err(refused(
            "trusted roots require HTTPS, or HTTP on a loopback address",
        ));
    }
    Ok(url)
}

fn fetch_roots(authority: &reqwest::Url) -> CliResult<RootSet> {
    #[derive(Deserialize)]
    struct RootsResponse {
        root_set: String,
        format_version: u16,
    }
    let url = format!(
        "{}/.well-known/cerulion-roots",
        authority.as_str().trim_end_matches('/')
    );
    let response = login_cmd::http_client_without_redirects()?
        .get(&url)
        .timeout(Duration::from_secs(15))
        .send()
        .map_err(|_| refused("trusted roots could not be fetched from the account service"))?;
    if !response.status().is_success() || response.url().as_str() != url {
        return Err(refused(
            "the account service refused or redirected trusted-root retrieval",
        ));
    }
    let mut bytes = Vec::new();
    response
        .take(MAX_BUNDLE_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() > MAX_BUNDLE_BYTES {
        return Err(refused("trusted-root response exceeds its size limit"));
    }
    let response: RootsResponse = serde_json::from_slice(&bytes)
        .map_err(|_| refused("trusted-root response is malformed"))?;
    let bytes = URL_SAFE_NO_PAD
        .decode(response.root_set)
        .map_err(|_| refused("trusted roots have invalid encoding"))?;
    let (roots, rest): (RootSet, _) =
        postcard::take_from_bytes(&bytes).map_err(|_| refused("trusted roots are malformed"))?;
    if response.format_version != FORMAT_VERSION
        || roots.version != FORMAT_VERSION
        || !rest.is_empty()
    {
        return Err(refused("trusted-root format is unsupported"));
    }
    roots
        .validate()
        .map_err(|_| refused("the trusted-root set is invalid"))?;
    Ok(roots)
}

fn read_private(path: &Path, limit: usize) -> CliResult<Vec<u8>> {
    let file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)?;
    let metadata = file.metadata()?;
    if !metadata.is_file()
        || (metadata.uid() != cerulion_hygiene::euid() && metadata.uid() != 0)
        || metadata.mode() & 0o777 != 0o600
        || metadata.len() > limit as u64
    {
        return Err(refused(
            "registration inputs must be bounded regular files with mode 0600",
        ));
    }
    let mut bytes = Vec::new();
    file.take(limit as u64 + 1).read_to_end(&mut bytes)?;
    if bytes.len() > limit {
        return Err(refused("registration input exceeds its size limit"));
    }
    Ok(bytes)
}

fn trusted_root(path: &Path) -> CliResult<()> {
    if !path.is_absolute()
        || path
            .components()
            .any(|c| matches!(c, Component::ParentDir | Component::CurDir))
    {
        return Err(refused(
            "robot state must use an absolute path without traversal",
        ));
    }
    for ancestor in path.ancestors() {
        match fs::symlink_metadata(ancestor) {
            Ok(meta)
                if !meta.is_dir()
                    || (meta.uid() != cerulion_hygiene::euid() && meta.uid() != 0)
                    || (meta.mode() & 0o022 != 0
                        && !(meta.uid() == 0 && meta.mode() & 0o1000 != 0)) =>
            {
                return Err(refused("robot state requires trusted directory ancestry"));
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(e.into()),
        }
    }
    Ok(())
}

fn create_state_root(path: &Path) -> CliResult<()> {
    let mut missing = Vec::new();
    for ancestor in path.ancestors() {
        match fs::symlink_metadata(ancestor) {
            Ok(_) => break,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => missing.push(ancestor),
            Err(e) => return Err(e.into()),
        }
    }
    for directory in missing.into_iter().rev() {
        fs::DirBuilder::new().mode(0o700).create(directory)?;
        fs::File::open(
            directory
                .parent()
                .ok_or_else(|| refused("state directory has no parent"))?,
        )?
        .sync_all()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cached_owner_matches_uuid_mapping_even_with_expired_login() {
        use cerulion_pairing::format::{AccountId, DeviceCert, PrincipalKind, Scope, Validity};
        let owner_hex = "021f0a006866fde5256f57cfad02a022fae9e8e473a6e4d3a471f1f4bd83e917";
        let owner = AccountId(hex::decode(owner_hex).unwrap().try_into().unwrap());
        let issuer = ed25519_dalek::SigningKey::from_bytes(&[0x22; 32]);
        let device = ed25519_dalek::SigningKey::from_bytes(&[0x33; 32]);
        let key = device.verifying_key().to_bytes();
        let cert = DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(key),
            account: owner,
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 10,
                not_after_ns: 100,
            },
            issued_at_ns: 10,
            issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
        }
        .sign(&issuer);
        let mut login = LoginSnapshot {
            state: auth::AuthState {
                account_id: "00112233-4455-4677-8899-aabbccddeeff".into(),
                session_token: "expired-test-session".into(),
                refresh_token: "unused-test-refresh".into(),
                expires_at_ns: 0,
                logged_in_ever: true,
                role: None,
            },
            key_file: PathBuf::from("/state/desk.key"),
            key,
        };
        // This helper checks the cached identity only. Complete trust/grant
        // validation belongs to first registration and the persistent writer.
        let bundle = RegistrationBundle {
            version: 1,
            robot_id: "11".repeat(32),
            owner_account: owner_hex.into(),
            device_key_file: login.key_file.clone(),
            device_cert_postcard: hex::encode(postcard::to_stdvec(&cert).unwrap()),
            root_set_postcard: String::new(),
            intermediate_postcard: String::new(),
            owner_grant_postcard: String::new(),
        };
        let prepared = describe(&bundle, Path::new("/state/robot-registration.json"), &login)
            .expect("UUID and signed account use the pinned shared mapping");
        assert_eq!(prepared.owner_account, owner_hex);
        login.state.account_id = "ffffffff-ffff-ffff-ffff-ffffffffffff".into();
        assert!(describe(&bundle, Path::new("/state/robot-registration.json"), &login).is_err());
    }

    #[test]
    fn registration_publication_is_complete_and_never_replaces_existing_state() {
        use std::os::unix::fs::symlink;
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join(BUNDLE_FILE);
        let bytes = br#"{"version":1,"owner":"complete"}"#;
        let staged = auth::stage_secret(&path, bytes).unwrap();
        assert!(!path.exists(), "staging must not publish a partial bundle");
        drop(staged);
        assert!(
            !path.exists(),
            "an interrupted unpublished write is retryable"
        );
        auth::stage_secret(&path, bytes)
            .unwrap()
            .commit_new()
            .unwrap();
        assert_eq!(fs::read(&path).unwrap(), bytes);
        assert!(auth::stage_secret(&path, b"replacement")
            .unwrap()
            .commit_new()
            .is_err());
        assert_eq!(fs::read(&path).unwrap(), bytes);
        let target = temp.path().join("target");
        fs::write(&target, b"untouched").unwrap();
        let link = temp.path().join("link");
        symlink(&target, &link).unwrap();
        assert!(auth::stage_secret(&link, bytes)
            .unwrap()
            .commit_new()
            .is_err());
        assert!(link.is_symlink());
        assert_eq!(fs::read(target).unwrap(), b"untouched");
    }

    #[test]
    fn trust_bootstrap_requires_secure_transport_or_loopback() {
        for allowed in [
            "https://accounts.example.test",
            "http://127.0.0.1:8787",
            "http://[::1]:8787",
        ] {
            assert!(trusted_service_url(allowed).is_ok(), "{allowed}");
        }
        for refused in [
            "http://accounts.example.test",
            "http://192.0.2.1",
            "ftp://accounts.example.test",
            "https://user:password@accounts.example.test",
            "https://accounts.example.test?token=x",
            "https://accounts.example.test#fragment",
        ] {
            assert!(trusted_service_url(refused).is_err(), "{refused}");
        }
    }

    #[test]
    fn untrusted_paths_and_special_files_are_refused_without_reading_them() {
        use std::os::unix::fs::{symlink, PermissionsExt};
        let temp = tempfile::tempdir().unwrap();
        let root = temp.path().canonicalize().unwrap();
        let secret = root.join("input");
        fs::write(&secret, b"private").unwrap();
        fs::set_permissions(&secret, fs::Permissions::from_mode(0o600)).unwrap();
        assert_eq!(read_private(&secret, 7).unwrap(), b"private");
        assert!(read_private(&secret, 6).is_err());
        symlink(&secret, root.join("link")).unwrap();
        assert!(read_private(&root.join("link"), 100).is_err());
        assert!(read_private(&root, 100).is_err());
        fs::set_permissions(&secret, fs::Permissions::from_mode(0o644)).unwrap();
        assert!(read_private(&secret, 100).is_err());
        assert!(trusted_root(Path::new("relative")).is_err());
        assert!(trusted_root(&root.join("../escaped")).is_err());
        symlink(&root, root.join("directory-link")).unwrap();
        assert!(trusted_root(&root.join("directory-link/child")).is_err());
    }
}
