// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side device↔account binding resolver.
//!
//! The account service binds THIS machine's device key to a CLOUD account by issuing
//! a [`SignedDeviceCert`] at login, cached at `~/.cerulion/device.cert` (see
//! [`crate::login_cmd::run_login`] / [`crate::auth::stage_device_cert_at`]). This module
//! reads that cached cert and resolves the machine's VERIFIED cloud [`AccountId`]:
//!
//! - it decodes the cached `base64url(postcard(SignedDeviceCert))`, and
//! - it asserts the cert attests THIS machine's device key (invariant I1 — a cert
//!   that binds a DIFFERENT key does not describe this machine and is refused).
//!
//! This is the desk half of the A3 device↔account binding: the robot side already
//! enforces it (`device_index` → account → `is_allowed`); this resolves the SAME
//! binding from the desk's own cached cert, so the desk knows its verified cloud
//! account (not the earlier self-account derived from the device key). It reads
//! ONLY the local cert — no network, no signature-chain verification (the chain is
//! the robot's job at pairing time; the desk trusts a cert it obtained over its own
//! authenticated session).
//!
//! **Internal — NOT a user-facing verb.** Robot ownership + device binding are
//! install-automatic (by design); this is a helper the desk-side presenter uses.
//! Its production caller is the A4 desk grant-presenter (which carries the cert +
//! grant on connect); it ships now, fully tested, so A4 wires an already-proven
//! resolver rather than re-deriving the binding a second way.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::{AccountId, PublicKey, SignedDeviceCert};

use crate::auth;
use crate::error::{CliError, CliResult};
use crate::login_cmd;

/// This machine's VERIFIED device↔account binding: the device (transport) key and
/// the cloud account the cached device cert binds it to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceBinding {
    /// This machine's device (transport) public key (ed25519 = iroh EndpointId).
    pub device_key: PublicKey,
    /// The cloud account the device cert binds the device key to.
    pub account: AccountId,
}

impl DeviceBinding {
    /// The bound account as `base64url` (the shape [`crate::auth::AuthState`] stores).
    pub fn account_b64(&self) -> String {
        URL_SAFE_NO_PAD.encode(self.account.0)
    }
}

/// Resolve this machine's verified device↔account binding from the cached device cert
/// (`~/.cerulion/device.cert`) + this machine's device key (`~/.cerulion/desk.key`).
///
/// Errors (all loud [`CliError::Login`], never a silent fallback to a self-account):
/// - no config dir (`CERULION_HOME` / home) resolvable,
/// - no cached device cert (this machine has not logged in / not cached one — the fix
///   is `cerulion login`),
/// - a corrupt / undecodable cert,
/// - a cert that attests a DIFFERENT device key than this machine holds.
///
/// An absent cert triggers [`auth::recover_interrupted_device_cert_state`] before
/// it is reported: a login killed between moving a cert aside and publishing its
/// `auth.json` leaves the binding at the aside, and this is the read that notices.
/// A cert that resolves, verifies, and agrees with the store is then cached at any
/// consumer that has NONE, which is the other half of the same interruption — and
/// symmetrically, a binding that only a `CERULION_NETD_*`-relocated cache holds is
/// adopted from there rather than reported absent.
///
/// Additionally, when the resolved account diverges from the cached `auth.json`
/// account, a LOUD `warn!` is emitted (a cert cached under a different account than the
/// current session — stale after an account switch) — the cert (cryptographic truth)
/// is still returned.
pub fn resolve_device_binding() -> CliResult<DeviceBinding> {
    let seed = login_cmd::ensure_device_key_seed()?;
    let device_key = ed25519_dalek::SigningKey::from_bytes(&seed)
        .verifying_key()
        .to_bytes();
    let path = auth::device_cert_path().ok_or_else(|| {
        CliError::Login("no home directory (set CERULION_HOME) to read the device cert".into())
    })?;
    let cert_b64 = match read_cert_file(&path) {
        Ok(s) => s,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            // A `cerulion login` killed between moving the previous cert aside and
            // publishing its `auth.json` leaves this path empty while the store
            // still names an account whose binding is recoverable. Repair it HERE
            // rather than waiting for another login: this is the read that
            // noticed, and the recovery is a rename the login would have done. It
            // fixes nothing when there is nothing set aside, so the retry answers
            // the same way.
            auth::recover_interrupted_device_cert_state();
            match read_cert_file(&path) {
                Ok(s) => s,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    // Still nothing here — but the certs are published one
                    // consumer at a time, and THIS path is not always the one a
                    // killed login reached: a `CERULION_NETD_*`-relocated cache
                    // can be holding the binding alone. Adopt it when it proves
                    // to be this machine's, for the account the store names.
                    match relocated_cert_for_this_desk(&device_key)? {
                        Some(s) => s,
                        None => {
                            return Err(CliError::Login(format!(
                                "no device cert is cached ({}) — this machine has not \
                                 completed a login that issued one; run `cerulion login`",
                                path.display()
                            )));
                        }
                    }
                }
                Err(e) => return Err(unreadable_cert(&path, &e)),
            }
        }
        Err(e) => return Err(unreadable_cert(&path, &e)),
    };
    let binding = verify_device_binding(cert_b64.trim(), &device_key)?;

    // Cross-check the cert's account against the cached session account. A divergence
    // is a cert cached under a different account than the current login (e.g. after an
    // account switch) — surface it LOUDLY; the cert stays the source of truth.
    let store_names_this_account = match auth::load().state() {
        Some(state) if state.account_id != binding.account_b64() => {
            tracing::warn!(
                cert_account = %binding.account_b64(),
                session_account = %state.account_id,
                "the cached device cert binds a DIFFERENT account than ~/.cerulion/auth.json — \
                 the cert may be stale after an account switch; re-run `cerulion login` to refresh it"
            );
            false
        }
        Some(_) => true,
        None => false,
    };
    // The cert just cleared both gates the propagation needs: it attests THIS
    // machine's device key, and the store names the account it binds it to. A
    // consumer with none is therefore a copy an interrupted login never made, not
    // a disagreement to preserve — and a cert that failed either gate propagates
    // nowhere, because copying one on the strength of its bytes alone is how the
    // previous account's cert would come back after a switch.
    if store_names_this_account {
        auth::cache_verified_device_cert_at_absent_consumers(
            cert_b64.trim(),
            &binding.account_b64(),
        );
    }
    Ok(binding)
}

/// The cert a `CERULION_NETD_*`-relocated cache holds, when it is this machine's
/// and names the account `auth.json` does — the binding an interrupted login can
/// leave at netd's copy and not at the CLI's own.
///
/// Both proofs are required, and neither is about the bytes being there: a cert
/// that attests another device key is not this desk's, and one naming another
/// account is what a switch left behind at a path it could not see. Adopting
/// either would bind this desk to something it is not signed in as — reporting no
/// cert is the correct answer, and `cerulion login` is the fix. A cert adopted here
/// is copied to this path by the propagation the caller runs afterwards, so the
/// repair converges instead of being redone on every command.
///
/// A candidate that is ABSENT is simply not a candidate, but one that is present
/// and unreadable — a FIFO, a socket, a directory, a mode this user cannot open —
/// is reported as itself: reporting "no cert is cached" for a path that holds one
/// this process refused to read would name the wrong file and hide the fix.
fn relocated_cert_for_this_desk(device_key: &[u8; 32]) -> CliResult<Option<String>> {
    let loaded = auth::load();
    let Some(account) = loaded.state().map(|s| &s.account_id) else {
        return Ok(None);
    };
    for path in auth::relocated_device_cert_consumer_paths() {
        let cert_b64 = match read_cert_file(&path) {
            Ok(s) => s,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => continue,
            Err(e) => return Err(unreadable_cert(&path, &e)),
        };
        let Ok(binding) = verify_device_binding(cert_b64.trim(), device_key) else {
            continue;
        };
        if binding.account_b64() != *account {
            continue;
        }
        tracing::info!(
            path = %path.display(),
            "resolved the device binding from a relocated cert cache — a login that \
             published the certs one consumer at a time did not reach the CLI's own path"
        );
        return Ok(Some(cert_b64));
    }
    Ok(None)
}

/// Read a cached cert, refusing anything that is not a REGULAR file (following
/// symlinks, which is how netd's rotations are pointed at one).
///
/// Opening a FIFO with no writer blocks forever, and a device node is worse, so
/// the kind is checked before the open rather than after a hang. `NotFound` — a
/// missing path — is passed through as itself, because that is the case the
/// interrupted-login recovery answers; a symlink resolving to nothing is NOT
/// that case, and is reported as the present-but-unusable entry it is.
fn read_cert_file(path: &std::path::Path) -> std::io::Result<String> {
    // The entry itself first: only a MISSING directory entry may read as
    // `NotFound`, because every caller treats that kind as "no cert here" and
    // moves on. A symlink whose target is gone answers `NotFound` to
    // `metadata`, and reporting the path as empty would name the wrong file —
    // the link is present, and relinking it is the fix.
    let entry = std::fs::symlink_metadata(path)?;
    let md = if entry.file_type().is_symlink() {
        std::fs::metadata(path).map_err(|e| match e.kind() {
            std::io::ErrorKind::NotFound => {
                std::io::Error::other("it is a symlink whose target does not exist")
            }
            _ => e,
        })?
    } else {
        entry
    };
    if !md.is_file() {
        return Err(std::io::Error::other(format!(
            "it is a {}, not a regular file",
            kind_name(md.file_type())
        )));
    }
    std::fs::read_to_string(path)
}

/// A file type's name for the refusal above — the user has to know what is AT the
/// path to fix it, and "not a regular file" alone does not say.
fn kind_name(t: std::fs::FileType) -> &'static str {
    #[cfg(unix)]
    {
        use std::os::unix::fs::FileTypeExt;
        if t.is_fifo() {
            return "named pipe";
        }
        if t.is_socket() {
            return "socket";
        }
        if t.is_block_device() || t.is_char_device() {
            return "device node";
        }
    }
    if t.is_dir() {
        "directory"
    } else {
        "special file"
    }
}

fn unreadable_cert(path: &std::path::Path, e: &std::io::Error) -> CliError {
    CliError::Login(format!(
        "reading the device cert {} failed: {e}",
        path.display()
    ))
}

/// Pure core: decode a `base64url(postcard(SignedDeviceCert))` blob and assert it
/// attests `expected_device_key`, returning the verified [`DeviceBinding`]. A
/// blob that does not decode, or a cert whose `device_key` is not this machine's key,
/// is a loud [`CliError::Login`]. No network, no chain verification.
pub fn verify_device_binding(
    cert_b64: &str,
    expected_device_key: &[u8; 32],
) -> CliResult<DeviceBinding> {
    let bytes = URL_SAFE_NO_PAD
        .decode(cert_b64.as_bytes())
        .map_err(|e| CliError::Login(format!("the cached device cert is not base64url: {e}")))?;
    let signed: SignedDeviceCert = postcard::from_bytes(&bytes).map_err(|e| {
        CliError::Login(format!(
            "the cached device cert did not decode as a SignedDeviceCert: {e} \
             (re-run `cerulion login` to refresh it)"
        ))
    })?;
    // The I1 device↔account binding read is the ONE shared verifier
    // (`SignedDeviceCert::account_for_device_key`) — the netd WAN dial
    // config resolves the SAME cert through it, so the two can never derive a
    // DIFFERENT account. A `PeerKeyMismatch` is re-worded into the desk's stale/foreign
    // remediation.
    let account = signed
        .account_for_device_key(&PublicKey(*expected_device_key))
        .map_err(|_| {
            CliError::Login(
                "the cached device cert attests a DIFFERENT device key than this machine holds \
                 (a stale or foreign cert) — re-run `cerulion login` to refresh it"
                    .into(),
            )
        })?;
    Ok(DeviceBinding {
        device_key: signed.cert.device_key,
        account,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_pairing::format::{DeviceCert, PrincipalKind, Scope, Validity, FORMAT_VERSION};
    use ed25519_dalek::SigningKey;

    /// Build a `base64url(postcard(SignedDeviceCert))` blob binding `device_key` →
    /// `account`, signed by a throwaway issuer (the signature is NOT checked by the
    /// desk-side resolver — only the bound key + account are read).
    fn make_cert_b64(device_key: [u8; 32], account: [u8; 32]) -> String {
        let issuer = SigningKey::from_bytes(&[0xAB; 32]);
        let cert = DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(device_key),
            account: AccountId(account),
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: u64::MAX,
            },
            issued_at_ns: 0,
            issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
        }
        .sign(&issuer);
        URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&cert).unwrap())
    }

    fn machine_key() -> [u8; 32] {
        SigningKey::from_bytes(&[7u8; 32])
            .verifying_key()
            .to_bytes()
    }

    #[test]
    fn a_cert_for_this_machine_resolves_to_the_cloud_account() {
        let key = machine_key();
        let account = [0x42u8; 32];
        let cert_b64 = make_cert_b64(key, account);
        let binding = verify_device_binding(&cert_b64, &key).unwrap();
        assert_eq!(binding.device_key, PublicKey(key));
        assert_eq!(binding.account, AccountId(account));
        // The account round-trips to the base64url shape auth.json stores.
        assert_eq!(binding.account_b64(), URL_SAFE_NO_PAD.encode(account));
    }

    #[test]
    fn a_cert_for_a_different_key_is_refused() {
        // I1: a cert that attests a key this machine does NOT hold is rejected — never
        // trusted as this machine's binding.
        let cert_b64 = make_cert_b64([0x11u8; 32], [0x42u8; 32]);
        let err = verify_device_binding(&cert_b64, &machine_key())
            .expect_err("a cert for a different device key must be refused");
        assert!(matches!(err, CliError::Login(_)));
        assert!(
            err.to_string().contains("DIFFERENT device key"),
            "names the mismatch: {err}"
        );
    }

    #[test]
    fn non_base64url_is_a_loud_error() {
        let err = verify_device_binding("not base64url !!!", &machine_key()).unwrap_err();
        assert!(matches!(err, CliError::Login(_)));
        assert!(err.to_string().contains("base64url"), "err: {err}");
    }

    #[test]
    fn a_valid_base64url_non_cert_is_a_loud_decode_error() {
        // Valid base64url whose bytes are NOT a SignedDeviceCert → a loud decode error
        // naming the fix, never a panic or a silent empty binding.
        let junk = URL_SAFE_NO_PAD.encode(b"this is not a postcard SignedDeviceCert");
        let err = verify_device_binding(&junk, &machine_key()).unwrap_err();
        assert!(matches!(err, CliError::Login(_)));
        assert!(
            err.to_string().contains("SignedDeviceCert"),
            "names the type: {err}"
        );
    }

    #[test]
    fn account_b64_matches_the_auth_json_account_shape() {
        // The binding's account_b64 uses the SAME URL_SAFE_NO_PAD encoding auth.json's
        // `account_id` uses, so the cross-check in resolve_device_binding compares like
        // for like (a wrong encoding here would make every session falsely "diverge").
        let account = [0x99u8; 32];
        let cert_b64 = make_cert_b64(machine_key(), account);
        let binding = verify_device_binding(&cert_b64, &machine_key()).unwrap();
        // Independent oracle: exactly what auth.json stores for this account id.
        assert_eq!(binding.account_b64(), URL_SAFE_NO_PAD.encode(account));
    }
}
