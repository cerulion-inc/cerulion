// SPDX-License-Identifier: AGPL-3.0-only
//! The CLI half of account self-service device management + the
//! robot-ACL revocation ENGINE seam.
//!
//! ## What is user-facing on the CLI
//!
//! `cerulion account devices list` / `cerulion account devices revoke <device_id>` —
//! self-service management of YOUR OWN devices (a lost/decommissioned desk), consistent
//! with the account-scoped `cerulion login`. A device is revoked against the CALLER's
//! account only (the account service refuses another account's device).
//!
//! ## What is NOT a CLI verb (by design)
//!
//! Robot ACL management (revoking a guest's access, revoking a device FROM a specific
//! robot) has NO `cerulion robot` CLI verb. Robot-management flows
//! live in Studio / the web account page, NOT the CLI (see [`crate::robot_cmd`]'s
//! module docs).
//!
//! The two account-service access halves are NOT in the same state, and the difference
//! matters to anyone deciding whether they are still needed:
//!
//! * [`fetch_robot_access`] IS live in this crate — [`sync_robot_epoch_to_cache_at`]
//!   calls it to build the desk's revocation-epoch cache.
//! * [`revoke_robot_access`] has NO caller anywhere. The owner-facing revoke surface is
//!   the HOSTED team page the account service serves, which issues
//!   `POST /v1/robots/{id}/revoke` over HTTP itself — so no Rust caller for it can
//!   exist, and an earlier claim here that Studio / the web page "call" this seam was
//!   wrong in both halves (nothing calls it, and no test covers it either).
//!
//! It is KEPT rather than deleted. Retain it for a
//! future `cerulion account revoke`-style verb, which would be its first caller.

use serde::Deserialize;

use crate::error::{CliError, CliResult};
use crate::login_cmd::{self, account_service_base, http_client};

/// One device row as the account service reports it (`GET /v1/devices`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct DeviceEntry {
    /// The opaque device id (the handle `revoke` takes).
    pub device_id: String,
    /// `base64url` of the device (transport) public key = iroh EndpointId.
    pub public_key: String,
    /// `human` | `machine`.
    #[serde(default)]
    pub principal_kind: String,
    /// Registration time (Unix ns).
    #[serde(default)]
    pub created_at_ns: u64,
    /// Whether the device is revoked.
    #[serde(default)]
    pub revoked: bool,
}

#[derive(Deserialize)]
struct ListDevicesResponse {
    #[serde(default)]
    devices: Vec<DeviceEntry>,
}

/// The result of a device self-revoke (`POST /v1/devices/{id}/revoke`) — the device
/// row + how many OWNED robots had it fanned into their revocation epoch.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct RevokeDeviceResult {
    pub device_id: String,
    #[serde(default)]
    pub revoked: bool,
    /// How many robots the caller OWNS had this device added to their revocation
    /// epoch (they refuse it at their accept gate once they sync).
    #[serde(default)]
    pub robots_updated: usize,
    /// `base64url(RobotId)` of any OWNED robots whose epoch bump FAILED — the device is
    /// still revoked, but re-run the revoke (idempotent) to reach these robots.
    #[serde(default)]
    pub robots_failed: Vec<String>,
}

/// The current robot access-list revocation state (`GET /v1/robots/{id}/access`) — the
/// engine seam for Studio / the web account page.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct RobotAccess {
    /// `base64url(RobotId)`.
    #[serde(default)]
    pub robot_id: String,
    /// The current epoch number (0 when the robot has no revocations yet).
    #[serde(default)]
    pub epoch: u64,
    /// The revoked account ids, `base64url`.
    #[serde(default)]
    pub revoked_accounts: Vec<String>,
    /// The revoked device (transport) keys, `base64url`.
    #[serde(default)]
    pub revoked_devices: Vec<String>,
    /// `base64url(postcard(SignedEpoch))` — the robot's sync artifact (absent when
    /// there is no epoch yet).
    #[serde(default)]
    pub signed_epoch: Option<String>,
    /// `base64url(postcard(SignedIntermediateCert))`.
    #[serde(default)]
    pub intermediate: Option<String>,
}

/// The RFC-8628-style error body (`{ "error": ..., "error_description": ... }`).
#[derive(Deserialize, Default)]
struct ErrorBody {
    #[serde(default)]
    error: String,
    #[serde(default)]
    error_description: String,
}

/// One target of a robot-access revocation: a whole account OR one device key.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RevokeTarget {
    /// Revoke a whole account's access to the robot (`base64url(AccountId)`).
    Account(String),
    /// Revoke ONE device key from the robot (`base64url(PublicKey)`); the account's
    /// other devices keep their access.
    Device(String),
}

// -- account self-service (CLI) ---------------------------------------------

/// List the logged-in account's own devices (`GET /v1/devices`).
///
/// A service with no device surface at all (the hosted default is identity-only —
/// see [`login_cmd::DEFAULT_ACCOUNT_SERVICE`]) answers 404 for the collection
/// itself; that is a capability refusal naming
/// [`login_cmd::ACCOUNT_SERVICE_ENV`], not "you own no devices" (which is an
/// empty list) and not "check the id" (there is no id in this request).
pub fn list_my_devices(session_token: &str) -> CliResult<Vec<DeviceEntry>> {
    let base = account_service_base();
    let client = http_client()?;
    let resp = client
        .get(format!("{base}/v1/devices"))
        .bearer_auth(session_token)
        .send()
        .map_err(|e| CliError::Login(format!("listing devices ({base}) failed: {e}")))?;
    if resp.status().as_u16() == 404 {
        return Err(no_device_surface(&base));
    }
    let parsed: ListDevicesResponse = handle_json(resp, "list devices")?;
    Ok(parsed.devices)
}

/// The refusal both `account devices` verbs give a service that registers no
/// devices, so neither reports a missing SURFACE as a missing device.
fn no_device_surface(base: &str) -> CliError {
    CliError::Login(format!(
        "{base} registers no devices (it issues no device certificates), so it has no device \
         list to show or revoke from — point {} at a service that does (a local \
         `cerulion-accountd`)",
        login_cmd::ACCOUNT_SERVICE_ENV
    ))
}

/// Revoke one of the logged-in account's OWN devices (`POST /v1/devices/{id}/revoke`).
/// The account service refuses (404) a device the caller does not own, surfaced here
/// as a loud, actionable error. On success the device is fanned into the caller's
/// OWNED robots' revocation epochs (see [`RevokeDeviceResult::robots_updated`]).
///
/// A 404 here is ambiguous on its own — an identity-only service has no such route
/// at all — so the collection is probed to tell "this service registers nothing"
/// apart from "that id is not yours", and each gets its own remediation.
pub fn revoke_my_device(session_token: &str, device_id: &str) -> CliResult<RevokeDeviceResult> {
    let base = account_service_base();
    let client = http_client()?;
    let resp = client
        .post(format!("{base}/v1/devices/{device_id}/revoke"))
        .bearer_auth(session_token)
        .send()
        .map_err(|e| CliError::Login(format!("revoking device ({base}) failed: {e}")))?;
    if resp.status().as_u16() == 404 {
        // The probe's own failure is NOT the answer to the question it was asked.
        // Swallowing a timeout here would report "that device is not yours" for a
        // service that never answered whether it registers devices at all.
        let collection = client
            .get(format!("{base}/v1/devices"))
            .bearer_auth(session_token)
            .send()
            .map_err(|e| {
                CliError::Login(format!(
                    "{base} answered 404 for that device, and asking whether it registers \
                     devices at all failed too ({e}) — so whether the id is wrong or the \
                     service has no device surface is unknown; retry, or check {}",
                    login_cmd::ACCOUNT_SERVICE_ENV
                ))
            })?;
        let probe = collection.status();
        // Only two probe answers are answers: 404 = no device surface, 2xx = there
        // is one and this id is not on it. Anything else (401 on an expired
        // session, 5xx) is the probe failing, which is the same non-answer a
        // timeout is — reporting the revoke's 404 then would call a device missing
        // on the word of a request that never ran.
        if probe.as_u16() == 404 {
            return Err(no_device_surface(&base));
        }
        if !probe.is_success() {
            return Err(CliError::Login(format!(
                "{base} answered 404 for that device, and asking whether it registers \
                 devices at all answered {probe} — so whether the id is wrong or the \
                 service has no device surface is unknown; retry, or check {}",
                login_cmd::ACCOUNT_SERVICE_ENV
            )));
        }
    }
    handle_json(resp, "revoke device")
}

// -- robot ACL revocation (engine seam — NOT a CLI verb; by design) -----

/// Add an account OR device key to a robot's revocation set + bump its signed epoch
/// (`POST /v1/robots/{id}/revoke`, owner-only).
///
/// UNCALLED today, and deliberately so: the owner-facing revoke surface is the hosted
/// team page, which issues this request over HTTP itself. Retained for a future
/// `cerulion account revoke`-style verb — see the module
/// docs.
pub fn revoke_robot_access(
    session_token: &str,
    robot_id: &str,
    target: &RevokeTarget,
) -> CliResult<RobotAccess> {
    let base = account_service_base();
    let client = http_client()?;
    let body = match target {
        RevokeTarget::Account(a) => serde_json::json!({ "account_id": a }),
        RevokeTarget::Device(d) => serde_json::json!({ "device_key": d }),
    };
    let resp = client
        .post(format!("{base}/v1/robots/{robot_id}/revoke"))
        .bearer_auth(session_token)
        .json(&body)
        .send()
        .map_err(|e| CliError::Login(format!("revoking robot access ({base}) failed: {e}")))?;
    handle_json(resp, "revoke robot access")
}

/// Fetch a robot's current revocation state + the signed epoch to sync
/// (`GET /v1/robots/{id}/access`, owner-only). The engine seam for Studio / the web page.
pub fn fetch_robot_access(session_token: &str, robot_id: &str) -> CliResult<RobotAccess> {
    let base = account_service_base();
    let client = http_client()?;
    let resp = client
        .get(format!("{base}/v1/robots/{robot_id}/access"))
        .bearer_auth(session_token)
        .send()
        .map_err(|e| CliError::Login(format!("fetching robot access ({base}) failed: {e}")))?;
    handle_json(resp, "fetch robot access")
}

// -- The desk's revocation-epoch CACHE writer ------------------------
//
// The delivery chain is: accountd → (this) the desk's epoch cache → the desk PUSHES it
// on its next dial → the robot applies it. This is the WRITE half; the read + push half
// lives in `cerulion_wireclient::config::resolve_epoch_sync` + `cerulion_netd`'s WAN
// plane (which cannot be reached from here — wireclient pulls iroh and this crate must
// stay iroh-free, so the artifact is built from `cerulion_pairing` types directly).
//
// Like `revoke_robot_access` / `fetch_robot_access` this is an ENGINE seam, NOT a CLI
// verb: robot-ACL flows live in Studio / the web account page (by design). The
// caller supplies the destination path, so this function invents no robot-id → hostname
// mapping and no new user-facing surface.

/// Convert a fetched [`RobotAccess`] into the desk's on-disk epoch-cache artifact
/// (`base64url(postcard(EpochSyncWire))` — the exact shape
/// `cerulion_wireclient::config::resolve_epoch_sync` reads).
///
/// Returns `Ok(None)` when the robot has no epoch yet (a robot with no revocations —
/// there is genuinely nothing to cache or push, and writing a placeholder would be
/// fake data). A response whose two halves do not decode is a LOUD error, never a
/// silently-skipped sync.
pub fn epoch_cache_artifact_from(access: &RobotAccess) -> CliResult<Option<String>> {
    use base64::engine::general_purpose::URL_SAFE_NO_PAD;
    use base64::Engine;

    let (Some(epoch_b64), Some(inter_b64)) =
        (access.signed_epoch.as_ref(), access.intermediate.as_ref())
    else {
        return Ok(None);
    };
    let decode = |what: &str, s: &str| -> CliResult<Vec<u8>> {
        URL_SAFE_NO_PAD.decode(s.trim().as_bytes()).map_err(|e| {
            CliError::Login(format!(
                "the account service's {what} for this robot is not base64url ({e}) — \
                 refusing to write a corrupt epoch cache"
            ))
        })
    };
    let signed_epoch: cerulion_pairing::format::SignedEpoch =
        postcard::from_bytes(&decode("signed_epoch", epoch_b64)?).map_err(|e| {
            CliError::Login(format!(
                "the account service's signed_epoch did not decode as a SignedEpoch ({e}) — \
                 refusing to write a corrupt epoch cache"
            ))
        })?;
    let intermediate: cerulion_pairing::format::SignedIntermediateCert =
        postcard::from_bytes(&decode("intermediate", inter_b64)?).map_err(|e| {
            CliError::Login(format!(
                "the account service's intermediate did not decode as a \
                 SignedIntermediateCert ({e}) — refusing to write a corrupt epoch cache"
            ))
        })?;
    let wire = cerulion_pairing::verify::EpochSyncWire::new(intermediate, signed_epoch);
    let bytes = wire
        .to_postcard()
        .map_err(|e| CliError::Login(format!("encoding the epoch-cache artifact failed: {e}")))?;
    Ok(Some(URL_SAFE_NO_PAD.encode(bytes)))
}

/// The desk's epoch-cache PATH for `robot` inside `epochs_dir` — the join
/// [`resolve_epoch_cache_path`] performs once the directory is resolved.
///
/// A verbatim re-export of [`cerulion_pairing::verify::epoch_cache_path_in`]: THE one
/// keying rule, the exact function both READERS resolve through
/// (`cerulion-netd`'s `WanRegistry::epoch_cache_path` and the `cerulion connect`
/// session, via `cerulion_wireclient::epoch::epoch_cache_path`). A hand-rolled join
/// here would be invisible at BOTH ends — the writer would report "cached", every
/// reader would report "nothing cached", and revocations would silently never travel —
/// so the convention lives in the one crate all three depend on rather than being
/// duplicated. (`cerulion_cli_engine` must stay iroh-free, so it cannot reach the
/// wireclient re-export.)
pub use cerulion_pairing::verify::epoch_cache_path_in;

/// THE one epoch-cache DIRECTORY resolver + its env name, re-exported verbatim from
/// [`cerulion_pairing::verify`] — the same pair both READERS reach:
/// `cerulion connect`'s binary through the `cerulion_wireclient::epoch::resolve_epoch_dir_from_env`
/// convenience wrapper, and `cerulion-netd`'s `WanRegistry` by calling `resolve_epoch_dir`
/// itself on the env value it captured at `from_env` (resolved lazily). Each
/// party performs its OWN env read; what is shared — and must never fork — is this
/// resolver and this env NAME. Exported so the writer's own resolution
/// ([`resolve_epoch_cache_path`]) is verifiably the SAME rule, not merely a documented
/// intention.
pub use cerulion_pairing::verify::{resolve_epoch_dir, EPOCH_DIR_ENV};

/// The desk's epoch-cache path for `robot`, resolved by the WRITER
/// through THE one shared rule — [`EPOCH_DIR_ENV`] → [`resolve_epoch_dir`] →
/// [`epoch_cache_path_in`], byte for byte what both readers resolve.
///
/// A writer that only re-exported the file-NAME join and left the directory to its
/// caller would not match the docs' promise of "the same directory both readers call". A
/// caller that hand-rolled the directory (or ignored `CERULION_EPOCH_DIR`) would write a
/// cache no reader ever looks at — the writer reporting "cached" while every dial reports
/// "nothing cached", with no symptom at either end. Resolving it HERE closes that.
///
/// The key-file anchor is the WELL-KNOWN desk key `~/.cerulion/desk.key`
/// ([`crate::connect_cmd::desk_key_path`]) — the same file `cerulion connect` defaults
/// `--key-file` to, and the conventional target of `CERULION_NETD_DESK_KEY`. A deployment
/// that relocates its desk key elsewhere must set [`EPOCH_DIR_ENV`], which is precisely
/// why that var is DESK-WIDE and honored by all three parties: it is the ONE knob that
/// keeps them in agreement.
///
/// `None` only when there is no home directory at all AND no explicit env override —
/// there is then no desk-wide location to cache into.
pub fn resolve_epoch_cache_path(robot: &str) -> Option<std::path::PathBuf> {
    let dir = resolve_epoch_dir(
        std::env::var(EPOCH_DIR_ENV).ok().as_deref(),
        crate::connect_cmd::desk_key_path().as_deref(),
    )?;
    Some(epoch_cache_path_in(&dir, robot))
}

/// Write the desk's epoch-cache artifact ATOMICALLY: a sibling temp file, then a
/// rename over `cache_path` (the same temp+rename discipline
/// `TrustStore::save` and the CLI's YAML writers use).
///
/// A plain `fs::write` truncates first, so a crash / full disk mid-write leaves a
/// TRUNCATED cache — which every reader classifies as `CacheUnreadable`, i.e. this
/// desk silently stops carrying revocations until someone re-syncs. Renaming over the
/// destination means a reader only ever sees the old artifact or the new one.
fn write_epoch_cache(artifact: &str, cache_path: &std::path::Path) -> CliResult<()> {
    if let Some(parent) = cache_path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).map_err(|e| {
                CliError::Login(format!(
                    "could not create the epoch-cache directory {}: {e}",
                    parent.display()
                ))
            })?;
        }
    }
    // Same directory as the destination, so the rename stays within one filesystem
    // (a cross-device rename would fail); pid-scoped so two concurrent syncs of
    // DIFFERENT robots never collide on the temp name.
    let tmp = cache_path.with_extension(format!("epoch.tmp{}", std::process::id()));
    std::fs::write(&tmp, artifact).map_err(|e| {
        CliError::Login(format!(
            "could not write the epoch cache {}: {e}",
            tmp.display()
        ))
    })?;
    std::fs::rename(&tmp, cache_path).map_err(|e| {
        // Never leave the temp behind on a failed rename.
        let _ = std::fs::remove_file(&tmp);
        CliError::Login(format!(
            "could not install the epoch cache {}: {e}",
            cache_path.display()
        ))
    })
}

/// Fetch a robot's access state and CACHE its signed epoch so the desk's next dial pushes
/// it. Creates the parent directory if needed, and writes ATOMICALLY (temp +
/// rename) so a reader never sees a half-written artifact.
///
/// `robot_id` is the account service's id for the robot (what the HTTP fetch is keyed by).
/// `robot_name` is the name THIS DESK knows the robot by — its `~/.cerulion/robots.toml`
/// key, which `cerulion pair` pins — because that, and only that, is what both readers
/// look the cache up under (a peer-reported name would let a dialed robot choose which
/// artifact it is handed). The destination is resolved by
/// [`resolve_epoch_cache_path`] — THE shared rule, so this write and every reader's
/// lookup are the same path or neither moves.
///
/// Returns the cached epoch NUMBER on success, or `Ok(None)` when the robot has no
/// epoch yet (nothing to push — the cache is left untouched rather than emptied, so a
/// previously-cached epoch is never destroyed by a robot that reports none).
///
/// Every failure is LOUD: a refused session, an undecodable response, an unresolvable
/// cache location, or an unwritable path all error rather than leaving the desk quietly
/// not carrying revocations.
pub fn sync_robot_epoch_to_cache(
    session_token: &str,
    robot_id: &str,
    robot_name: &str,
) -> CliResult<Option<u64>> {
    let Some(cache_path) = resolve_epoch_cache_path(robot_name) else {
        return Err(CliError::Login(format!(
            "cannot resolve where to cache this robot's revocation epoch (no home \
             directory and {EPOCH_DIR_ENV} is unset), so the desk would silently carry no \
             revocations for '{robot_name}'. Set {EPOCH_DIR_ENV} to the desk-wide epochs \
             directory (the same var `cerulion connect` and `cerulion-netd` read)."
        )));
    };
    sync_robot_epoch_to_cache_at(session_token, robot_id, &cache_path)
}

/// The path-injectable core of [`sync_robot_epoch_to_cache`] — the fetch + decode + atomic
/// write against an EXPLICIT destination.
///
/// `cache_path` must come from [`resolve_epoch_cache_path`] (or, in a test, a tempdir path
/// built with [`epoch_cache_path_in`]): a hand-rolled join is exactly the divergence the
/// shared rule exists to prevent.
pub fn sync_robot_epoch_to_cache_at(
    session_token: &str,
    robot_id: &str,
    cache_path: &std::path::Path,
) -> CliResult<Option<u64>> {
    let access = fetch_robot_access(session_token, robot_id)?;
    let Some(artifact) = epoch_cache_artifact_from(&access)? else {
        tracing::info!(
            robot_id = %robot_id,
            "the robot has no revocation epoch yet — nothing to cache or push \
             (any previously cached epoch is left in place)"
        );
        return Ok(None);
    };
    write_epoch_cache(&artifact, cache_path)?;
    tracing::info!(
        robot_id = %robot_id,
        epoch = access.epoch,
        cache = %cache_path.display(),
        "cached the robot's revocation epoch — the desk's next dial pushes it"
    );
    Ok(Some(access.epoch))
}

// -- shared HTTP plumbing ----------------------------------------------------

/// Resolve the current session token, refreshing a stale one first. A missing/expired
/// session is a loud error naming the fix (`cerulion login`). The main dispatch's
/// login gate normally guarantees a session BEFORE the command runs; this is the
/// defensive backstop for a direct engine caller.
pub fn require_session() -> CliResult<String> {
    // Best-effort refresh if the session is stale (never fatal on its own).
    let _ = login_cmd::refresh_session_if_stale();
    let loaded = crate::auth::load();
    let now = now_ns();
    match loaded.state() {
        Some(s) if s.session_is_valid(now) => Ok(s.session_token.clone()),
        Some(_) => Err(CliError::Login(
            "your session has expired — run `cerulion login`".into(),
        )),
        None => Err(CliError::Login(
            "not logged in — run `cerulion login`".into(),
        )),
    }
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Decode a success response as `T`, or map a non-2xx to a loud, actionable
/// [`CliError::Login`] (401 → re-login; 404 → not-found; else the server's reason).
fn handle_json<T: serde::de::DeserializeOwned>(
    resp: reqwest::blocking::Response,
    what: &str,
) -> CliResult<T> {
    if resp.status().is_success() {
        return resp
            .json()
            .map_err(|e| CliError::Login(format!("could not parse the {what} response: {e}")));
    }
    let status = resp.status();
    let err: ErrorBody = resp.json().unwrap_or_default();
    let detail = if err.error_description.is_empty() {
        err.error
    } else {
        err.error_description
    };
    if status.as_u16() == 401 {
        return Err(CliError::Login(format!(
            "the account service refused the session ({status}) on {what}: {detail} — run \
             `cerulion login`"
        )));
    }
    if status.as_u16() == 404 {
        return Err(CliError::Login(format!(
            "{what} refused ({status}): {detail} — check the id, and that you own it"
        )));
    }
    Err(CliError::Login(format!(
        "{what} refused ({status}): {detail}"
    )))
}

/// Render the account's devices as a human-readable list to `out` (the CLI printer).
pub fn render_devices(
    out: &mut dyn std::io::Write,
    devices: &[DeviceEntry],
) -> std::io::Result<()> {
    if devices.is_empty() {
        return writeln!(out, "no devices registered to this account");
    }
    writeln!(
        out,
        "DEVICE ID                             KIND     STATE     PUBLIC KEY"
    )?;
    for d in devices {
        let state = if d.revoked { "REVOKED" } else { "active" };
        let key_short: String = d.public_key.chars().take(16).collect();
        writeln!(
            out,
            "{:<37} {:<8} {:<9} {}…",
            d.device_id, d.principal_kind, state, key_short
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn device_entry_parses_the_service_shape() {
        // The exact `GET /v1/devices` body the account service returns.
        let body = r#"{"devices":[
            {"device_id":"dev-1","public_key":"AAAA","principal_kind":"human","created_at_ns":42,"revoked":false},
            {"device_id":"dev-2","public_key":"BBBB","principal_kind":"machine","created_at_ns":7,"revoked":true}
        ]}"#;
        let parsed: ListDevicesResponse = serde_json::from_str(body).unwrap();
        assert_eq!(parsed.devices.len(), 2);
        assert_eq!(parsed.devices[0].device_id, "dev-1");
        assert!(!parsed.devices[0].revoked);
        assert!(parsed.devices[1].revoked);
        assert_eq!(parsed.devices[1].principal_kind, "machine");
    }

    #[test]
    fn revoke_device_result_parses_the_fan_out_count_and_failures() {
        // The `POST /v1/devices/{id}/revoke` body: owned-robot fan-out count + failures.
        let body = r#"{"device_id":"dev-1","public_key":"AAAA","revoked":true,"robots_updated":3,"robots_failed":["Rx"]}"#;
        let r: RevokeDeviceResult = serde_json::from_str(body).unwrap();
        assert_eq!(r.device_id, "dev-1");
        assert!(r.revoked);
        assert_eq!(r.robots_updated, 3);
        assert_eq!(r.robots_failed, vec!["Rx".to_string()]);
        // Absent robots_updated / robots_failed default to 0 / empty.
        let none = r#"{"device_id":"dev-2","revoked":true}"#;
        let r2: RevokeDeviceResult = serde_json::from_str(none).unwrap();
        assert_eq!(r2.robots_updated, 0);
        assert!(r2.robots_failed.is_empty());
    }

    #[test]
    fn robot_access_parses_and_tolerates_absent_signed_epoch() {
        // No-revocation state: epoch 0, empty sets, null signed epoch.
        let none = r#"{"robot_id":"R","epoch":0,"revoked_accounts":[],"revoked_devices":[],"signed_epoch":null,"intermediate":null}"#;
        let a: RobotAccess = serde_json::from_str(none).unwrap();
        assert_eq!(a.epoch, 0);
        assert!(a.signed_epoch.is_none());
        assert!(a.revoked_devices.is_empty());

        // A device-revoked state with a signed epoch.
        let some = r#"{"robot_id":"R","epoch":2,"revoked_accounts":["ACC"],"revoked_devices":["DEV"],"signed_epoch":"SE","intermediate":"IC"}"#;
        let b: RobotAccess = serde_json::from_str(some).unwrap();
        assert_eq!(b.epoch, 2);
        assert_eq!(b.revoked_devices, vec!["DEV".to_string()]);
        assert_eq!(b.signed_epoch.as_deref(), Some("SE"));
    }

    // ---- The epoch-cache writer -----------------------------------

    /// Build a `RobotAccess` whose two halves are the REAL base64url(postcard) blobs
    /// the account service returns, from genuine signed artifacts (Principle #13 — no
    /// hand-waved placeholder strings).
    fn access_with_real_artifacts(epoch_n: u64) -> RobotAccess {
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        use cerulion_pairing::format::*;

        let inter_sk = ed25519_dalek::SigningKey::from_bytes(&[10u8; 32]);
        let root_sk = ed25519_dalek::SigningKey::from_bytes(&[1u8; 32]);
        let inter_pk = PublicKey(inter_sk.verifying_key().to_bytes());
        let validity = Validity {
            not_before_ns: 0,
            not_after_ns: 100_000_000_000_000,
        };
        let intermediate = IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: inter_pk,
            validity,
            issued_at_ns: 5,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&root_sk]);
        let signed_epoch = AccessListEpoch {
            version: FORMAT_VERSION,
            robot: RobotId([0x0B; 32]),
            epoch: epoch_n,
            revoked_accounts: vec![AccountId([0x0C; 32])],
            revoked_devices: vec![PublicKey([0x0D; 32])],
            issued_at_ns: 6,
            issuer_key: inter_pk,
        }
        .sign(&inter_sk);
        RobotAccess {
            robot_id: "R".into(),
            epoch: epoch_n,
            revoked_accounts: vec!["ACC".into()],
            revoked_devices: vec!["DEV".into()],
            signed_epoch: Some(URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&signed_epoch).unwrap())),
            intermediate: Some(URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&intermediate).unwrap())),
        }
    }

    #[test]
    fn epoch_cache_artifact_round_trips_to_what_the_desk_reads() {
        let access = access_with_real_artifacts(4);
        let artifact = epoch_cache_artifact_from(&access)
            .expect("a well-formed access response converts")
            .expect("it carries an epoch");

        // Decode it the way the DESK does (base64url → postcard → EpochSyncWire) and
        // check every carried field against hand-stated expectations.
        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let bytes = URL_SAFE_NO_PAD.decode(artifact.as_bytes()).unwrap();
        let wire = cerulion_pairing::verify::EpochSyncWire::from_postcard(&bytes)
            .expect("the desk-side reader decodes it");
        assert_eq!(wire.signed_epoch.epoch_data.epoch, 4);
        assert_eq!(
            wire.signed_epoch.epoch_data.robot,
            cerulion_pairing::format::RobotId([0x0B; 32])
        );
        assert_eq!(
            wire.signed_epoch.epoch_data.revoked_accounts,
            vec![cerulion_pairing::format::AccountId([0x0C; 32])]
        );
        assert_eq!(
            wire.signed_epoch.epoch_data.revoked_devices,
            vec![cerulion_pairing::format::PublicKey([0x0D; 32])]
        );
    }

    /// The WRITE half of `sync_robot_epoch_to_cache` (the part that
    /// does not need the account service) is atomic and creates its directory.
    ///
    /// `sync_robot_epoch_to_cache` itself is an ENGINE seam with no CLI verb by
    /// design (robot-ACL flows live in Studio / the web account page), exactly like its
    /// siblings `revoke_robot_access` / `fetch_robot_access`; it is kept, not deleted,
    /// and its two testable halves are `epoch_cache_artifact_from` (above) and this
    /// writer.
    #[test]
    fn writing_the_epoch_cache_is_atomic_and_creates_its_directory() {
        let dir = tempfile::tempdir().unwrap();
        // A nested, not-yet-existing epochs directory.
        let path = epoch_cache_path_in(&dir.path().join("nested").join("epochs"), "go2");
        write_epoch_cache("first-artifact", &path).expect("writes into a fresh directory");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "first-artifact");

        // An overwrite REPLACES the artifact and leaves NO temp file behind (a stray
        // `.tmp` would be a half-written revocation sitting next to the real one).
        write_epoch_cache("second-artifact", &path).expect("overwrites");
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "second-artifact");
        let strays: Vec<String> = std::fs::read_dir(path.parent().unwrap())
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .filter(|n| n != "go2.epoch")
            .collect();
        assert!(
            strays.is_empty(),
            "the atomic write must leave no temp files: {strays:?}"
        );
    }

    /// The WRITER's cache path and the READERS' cache path are the
    /// SAME name, because both resolve through the one `cerulion_pairing` function.
    ///
    /// A drifted name is invisible at both ends — the writer logs "cached", every
    /// reader reports "nothing cached", and revocations silently never travel — so
    /// this pins the agreement against a HAND-written oracle rather than a
    /// self-compare, including the traversal-safety rewrites.
    #[test]
    fn the_writer_and_the_readers_agree_on_the_cache_file_name() {
        use cerulion_pairing::verify::epoch_cache_file_name;

        // Hand oracle for the shared convention (the readers'
        // `cerulion_wireclient::config::epoch_cache_file_name` is a verbatim re-export
        // of this same function, and netd joins it in `WanRegistry::epoch_cache_path`).
        assert_eq!(epoch_cache_file_name("go2"), "go2.epoch");
        assert_eq!(epoch_cache_file_name("  go2  "), "go2.epoch");
        assert_eq!(
            epoch_cache_file_name("../../etc/passwd"),
            ".._.._etc_passwd.epoch"
        );
        assert_eq!(epoch_cache_file_name(".."), "__.epoch");

        // The writer's path helper joins exactly that name.
        let dir = std::path::Path::new("/desk/.cerulion/epochs");
        assert_eq!(
            epoch_cache_path_in(dir, "go2"),
            dir.join("go2.epoch"),
            "the writer must place the artifact where every reader looks"
        );
        // And a hostile robot name cannot escape the epochs directory.
        assert_eq!(
            epoch_cache_path_in(dir, "../../etc/passwd"),
            dir.join(".._.._etc_passwd.epoch")
        );
    }

    /// The WRITER HALF of the three-party agreement pin:
    /// the writer's PRODUCTION path resolution ([`resolve_epoch_cache_path`], which
    /// `sync_robot_epoch_to_cache` calls) must equal, byte for byte, what the shared
    /// substrate resolves from the same (env, desk-key) inputs — the same literal oracle
    /// the two READER halves are anchored to
    /// (`cerulion_wireclient::epoch::both_desk_paths_resolve_one_cache_path` and
    /// `cerulion_netd::wan::netd_from_env_resolves_the_shared_epoch_cache_path`).
    ///
    /// Without this pin a writer could resolve NO directory at all — re-export the file-name join
    /// and leave the directory to its caller — while its doc claims the shared resolver
    /// "moves for the writer and both readers together or not at all". A caller that
    /// ignored `CERULION_EPOCH_DIR` would write where no dial ever looks, with the writer
    /// logging "cached" and every reader reporting "nothing cached".
    ///
    /// Env-mutating (`HOME` + `CERULION_EPOCH_DIR`) → serialized on the CRATE-WIDE env
    /// lock ([`crate::test_env`]); a file-local mutex is not enough, precisely
    /// because it would let `HOME` race `connect_cmd`.
    #[test]
    fn the_writer_resolves_the_shared_epoch_cache_path() {
        let _lock = env_lock();
        let _snap = EnvSnapshot::take(&["HOME", EPOCH_DIR_ENV]);

        // The env NAME is the shared desk-wide one (never a writer-private alias).
        assert_eq!(EPOCH_DIR_ENV, "CERULION_EPOCH_DIR");

        let home = tempfile::tempdir().unwrap();
        std::env::set_var("HOME", home.path());

        // 1. No override ⇒ the sibling `epochs/` dir next to the WELL-KNOWN desk key,
        //    i.e. `~/.cerulion/epochs/<robot>.epoch` — the literal oracle both readers
        //    are anchored to.
        std::env::remove_var(EPOCH_DIR_ENV);
        let expected = home
            .path()
            .join(".cerulion")
            .join("epochs")
            .join("go2.epoch");
        assert_eq!(
            resolve_epoch_cache_path("go2"),
            Some(expected.clone()),
            "the writer must resolve the shared ~/.cerulion/epochs/<robot>.epoch location"
        );
        // …and it is the SAME composition the readers perform (shared resolver + shared
        // join), not merely a coincidentally equal literal.
        assert_eq!(
            resolve_epoch_cache_path("go2"),
            resolve_epoch_dir(None, crate::connect_cmd::desk_key_path().as_deref())
                .map(|d| epoch_cache_path_in(&d, "go2")),
        );

        // 2. The DESK-WIDE override relocates the writer's cache too — the whole point
        //    of the var: relocating it moves all three parties or none.
        let relocated = home.path().join("elsewhere").join("epochs");
        std::env::set_var(EPOCH_DIR_ENV, &relocated);
        assert_eq!(
            resolve_epoch_cache_path("go2"),
            Some(relocated.join("go2.epoch")),
            "CERULION_EPOCH_DIR must relocate the WRITER's destination as well"
        );

        // 3. A blank override is NOT an override (an exported-but-empty var must not
        //    silently relocate the cache to the current directory).
        std::env::set_var(EPOCH_DIR_ENV, "   ");
        assert_eq!(resolve_epoch_cache_path("go2"), Some(expected));

        // 4. The traversal-safety rewrite still applies through the production path.
        std::env::set_var(EPOCH_DIR_ENV, &relocated);
        assert_eq!(
            resolve_epoch_cache_path("../../etc/passwd"),
            Some(relocated.join(".._.._etc_passwd.epoch"))
        );
    }

    /// Serializes the env-mutating tests in this module against EVERY other
    /// env-mutating lib test in the crate (all lib tests share one process, hence one
    /// environment) — see [`crate::test_env`]. A private per-module mutex would only
    /// serialize this file against itself, which is how `HOME` raced between this
    /// module and `connect_cmd`'s `plan` pin.
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    /// Snapshots the named env vars on construction and RESTORES them on drop (so a
    /// test can freely set/remove them without leaking to siblings, even on a panic).
    struct EnvSnapshot(Vec<(&'static str, Option<String>)>);
    impl EnvSnapshot {
        fn take(keys: &[&'static str]) -> Self {
            EnvSnapshot(keys.iter().map(|k| (*k, std::env::var(k).ok())).collect())
        }
    }
    impl Drop for EnvSnapshot {
        fn drop(&mut self) {
            for (k, v) in &self.0 {
                match v {
                    Some(v) => std::env::set_var(k, v),
                    None => std::env::remove_var(k),
                }
            }
        }
    }

    #[test]
    fn epoch_cache_artifact_is_none_when_the_robot_has_no_epoch() {
        // A robot with no revocations: nothing to cache. Writing a placeholder would be
        // fake data, and emptying an existing cache would DESTROY a real epoch.
        let mut access = access_with_real_artifacts(1);
        access.signed_epoch = None;
        assert_eq!(epoch_cache_artifact_from(&access).unwrap(), None);
        // A half-present response (epoch but no intermediate) is likewise not cacheable:
        // `apply_epoch` needs BOTH halves, so a one-sided artifact is useless.
        let mut half = access_with_real_artifacts(1);
        half.intermediate = None;
        assert_eq!(epoch_cache_artifact_from(&half).unwrap(), None);
    }

    #[test]
    fn epoch_cache_artifact_refuses_undecodable_halves_loudly() {
        // A corrupt service response must NEVER be written as a cache the desk would
        // then fail to read (or worse, push).
        let mut bad_b64 = access_with_real_artifacts(1);
        bad_b64.signed_epoch = Some("not base64url !!!".into());
        let err = epoch_cache_artifact_from(&bad_b64).unwrap_err();
        assert!(err.to_string().contains("not base64url"), "err: {err}");

        use base64::engine::general_purpose::URL_SAFE_NO_PAD;
        use base64::Engine;
        let mut bad_body = access_with_real_artifacts(1);
        bad_body.signed_epoch = Some(URL_SAFE_NO_PAD.encode(b"not a SignedEpoch"));
        let err = epoch_cache_artifact_from(&bad_body).unwrap_err();
        assert!(err.to_string().contains("did not decode"), "err: {err}");

        // ...and the SAME discipline on the intermediate half.
        let mut bad_inter = access_with_real_artifacts(1);
        bad_inter.intermediate = Some(URL_SAFE_NO_PAD.encode(b"not a cert"));
        let err = epoch_cache_artifact_from(&bad_inter).unwrap_err();
        assert!(err.to_string().contains("intermediate"), "err: {err}");
    }

    #[test]
    fn render_devices_marks_revoked_and_handles_empty() {
        let mut out = Vec::new();
        render_devices(&mut out, &[]).unwrap();
        assert!(String::from_utf8(out).unwrap().contains("no devices"));

        let mut out = Vec::new();
        render_devices(
            &mut out,
            &[DeviceEntry {
                device_id: "dev-9".into(),
                public_key: "ABCDEF0123456789extra".into(),
                principal_kind: "human".into(),
                created_at_ns: 1,
                revoked: true,
            }],
        )
        .unwrap();
        let s = String::from_utf8(out).unwrap();
        assert!(s.contains("dev-9"));
        assert!(s.contains("REVOKED"));
    }
}
