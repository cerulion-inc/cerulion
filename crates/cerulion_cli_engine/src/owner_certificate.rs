// SPDX-License-Identifier: AGPL-3.0-only
//! Load a consistent login certificate chain for automatic owner access.
//!
//! The robot verifies signatures, validity, ownership and revocation. This loader
//! checks that the locally cached chain belongs to the current login and transport
//! key. It never creates a key, refreshes a session or falls back to another account.

use std::fs::OpenOptions;
use std::io::Read;
use std::path::{Path, PathBuf};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::{
    AccountId, PublicKey, SignedDeviceCert, SignedIntermediateCert, FORMAT_VERSION,
};
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use serde::{Deserialize, Serialize};

use crate::auth;
use crate::error::{CliError, CliResult};

const MAX_CACHE_BYTES: usize = 64 * 1024;

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CachedChain {
    device_cert: String,
    intermediate: String,
}

pub(crate) fn cache_path(auth_path: &Path) -> PathBuf {
    auth_path.with_file_name("device-chain.json")
}

/// Stage the leaf and issuer together, before login publishes its account state.
pub(crate) fn stage_at(
    path: &Path,
    device_cert: &str,
    intermediate: &str,
) -> std::io::Result<auth::StagedCert> {
    let bytes = serde_json::to_vec(&CachedChain {
        device_cert: device_cert.trim().to_owned(),
        intermediate: intermediate.trim().to_owned(),
    })?;
    if bytes.len() > MAX_CACHE_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "device certificate chain exceeds the cache size limit",
        ));
    }
    auth::stage_secret(path, &bytes)
}

/// Load the current login's owner presentation without contacting the network.
///
/// A legacy or identity-only login has no usable complete chain and must log in
/// against a certificate-issuing account service before enabling owner WAN access.
/// A partial login publication is refused rather than mixing cached identities.
pub fn load() -> CliResult<OwnerCertificatePresentationWire> {
    let dir = auth::cerulion_config_dir()
        .ok_or_else(|| invalid("no configuration directory is available"))?;
    load_at(&dir)
}

fn load_at(dir: &Path) -> CliResult<OwnerCertificatePresentationWire> {
    let auth_path = dir.join("auth.json");
    auth::with_store_lock(&auth_path, || Ok(load_locked(dir, &auth_path)))
        .map_err(|_| invalid("the account store could not be locked"))?
}

fn load_locked(dir: &Path, auth_path: &Path) -> CliResult<OwnerCertificatePresentationWire> {
    let loaded = auth::load_from(auth_path);
    let state = loaded
        .state()
        .filter(|state| state.logged_in_ever)
        .ok_or_else(|| invalid("no current login is stored"))?;
    let seed: [u8; 32] = read_bounded(&dir.join("desk.key"), 32)
        .map_err(|_| invalid("the login device key could not be read"))?
        .try_into()
        .map_err(|_| invalid("the login device key is not exactly 32 bytes"))?;
    let key = PublicKey(
        ed25519_dalek::SigningKey::from_bytes(&seed)
            .verifying_key()
            .to_bytes(),
    );
    let current = read_bounded(&dir.join("device.cert"), MAX_CACHE_BYTES)
        .map_err(|_| invalid("the current device certificate is unavailable"))?;
    let current = std::str::from_utf8(&current)
        .map_err(|_| invalid("the current device certificate is not valid text"))?;
    let cached = read_bounded(&cache_path(auth_path), MAX_CACHE_BYTES)
        .map_err(|_| invalid("no readable complete device certificate chain is cached"))?;
    let cached: CachedChain = serde_json::from_slice(&cached)
        .map_err(|_| invalid("the device certificate chain cache is malformed"))?;
    if current.trim() != cached.device_cert {
        return Err(invalid(
            "the device certificate chain does not match the current certificate",
        ));
    }
    let cert: SignedDeviceCert = decode(&cached.device_cert)
        .map_err(|_| invalid("the cached device certificate could not be decoded"))?;
    let intermediate: SignedIntermediateCert = decode(&cached.intermediate)
        .map_err(|_| invalid("the cached intermediate certificate could not be decoded"))?;
    let account = crate::account_identity::pairing_account_id(&state.account_id)
        .map_err(|_| invalid("the device certificate names a different login account"))?;
    validate_leaf(&cert, account, key)?;
    if intermediate.cert.version != FORMAT_VERSION
        || intermediate.cert.validity.not_before_ns >= intermediate.cert.validity.not_after_ns
        || intermediate.signatures.is_empty()
    {
        return Err(invalid(
            "the intermediate certificate has an invalid format or validity interval",
        ));
    }
    if cert.cert.issuer_key != intermediate.cert.intermediate_key {
        return Err(invalid(
            "the device certificate and intermediate name different issuers",
        ));
    }
    Ok(OwnerCertificatePresentationWire::new(intermediate, cert))
}

// The caller already holds the auth transaction lock and checked this identity.
// Absence is compatible with legacy logins; present invalid state never becomes None.
#[cfg(unix)]
pub(crate) fn load_optional_locked(
    dir: &Path,
    auth_path: &Path,
    account: AccountId,
    key: PublicKey,
) -> CliResult<Option<OwnerCertificatePresentationWire>> {
    let leaf = read_optional(&dir.join("device.cert"))
        .map_err(|_| invalid("the current device certificate could not be read"))?;
    let chain = read_optional(&cache_path(auth_path))
        .map_err(|_| invalid("the device certificate chain could not be read"))?;
    match (leaf, chain) {
        (None, None) => Ok(None),
        (None, Some(_)) => Err(invalid(
            "a device certificate chain exists without its leaf",
        )),
        (Some(leaf), None) => {
            let text = std::str::from_utf8(&leaf)
                .map_err(|_| invalid("the current device certificate is not valid text"))?;
            let cert = decode(text.trim())
                .map_err(|_| invalid("the cached device certificate could not be decoded"))?;
            validate_leaf(&cert, account, key)?;
            Ok(None)
        }
        (Some(_), Some(_)) => {
            let proof = load_locked(dir, auth_path)?;
            validate_leaf(&proof.device_cert, account, key)?;
            Ok(Some(proof))
        }
    }
}

fn validate_leaf(cert: &SignedDeviceCert, account: AccountId, key: PublicKey) -> CliResult<()> {
    if cert.cert.version != FORMAT_VERSION
        || cert.cert.validity.not_before_ns >= cert.cert.validity.not_after_ns
    {
        return Err(invalid(
            "the device certificate has an invalid format or validity interval",
        ));
    }
    if cert.cert.device_key != key {
        return Err(invalid(
            "the device certificate names a different device key",
        ));
    }
    if cert.cert.account != account {
        return Err(invalid(
            "the device certificate names a different login account",
        ));
    }
    Ok(())
}

#[cfg(unix)]
fn read_optional(path: &Path) -> std::io::Result<Option<Vec<u8>>> {
    match read_bounded(path, MAX_CACHE_BYTES) {
        Ok(bytes) => Ok(Some(bytes)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
            match std::fs::symlink_metadata(path) {
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
                _ => Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "present certificate material could not be read",
                )),
            }
        }
        Err(error) => Err(error),
    }
}

fn decode<T: serde::de::DeserializeOwned>(encoded: &str) -> Result<T, ()> {
    let bytes = URL_SAFE_NO_PAD.decode(encoded).map_err(|_| ())?;
    let (value, remaining) = postcard::take_from_bytes(&bytes).map_err(|_| ())?;
    if !remaining.is_empty() {
        return Err(());
    }
    Ok(value)
}

fn invalid(reason: &str) -> CliError {
    CliError::Login(format!(
        "owner access is unavailable: {reason}; run `cerulion login` against a certificate-issuing account service"
    ))
}

fn read_bounded(path: &Path, max: usize) -> std::io::Result<Vec<u8>> {
    let mut options = OpenOptions::new();
    options.read(true);
    // Check the opened descriptor, and never block on a FIFO substituted between
    // a path metadata check and open. Certificate symlinks remain supported.
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.custom_flags(libc::O_NONBLOCK);
    }
    let file = options.open(path)?;
    if !file.metadata()?.is_file() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "certificate state is not a regular file",
        ));
    }
    let mut bytes = Vec::new();
    file.take(max as u64 + 1).read_to_end(&mut bytes)?;
    if bytes.len() > max {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "certificate state exceeds the size limit",
        ));
    }
    Ok(bytes)
}

#[cfg(test)]
mod tests;
