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
            let key_file =
                auth::machine_key_read_path().ok_or_else(|| refused("no login key path"))?;
            let key = public_key_at(&key_file)?;
            Ok(LoginSnapshot {
                state,
                key_file,
                key,
            })
        })())
    })?
}

/// The same login: the same auth state and the same machine key. The key's
/// file name is not compared: registration can adopt a legacy `desk.key` into
/// `identity/device.key` between the two snapshots, which keeps the key.
fn same_login(before: &LoginSnapshot, after: &LoginSnapshot) -> bool {
    before.state == after.state && before.key == after.key
}

/// The public key of the 32-byte seed stored at `path`.
fn public_key_at(path: &Path) -> CliResult<[u8; 32]> {
    let seed: [u8; 32] = read_private(path, 32)?
        .try_into()
        .map_err(|_| refused("the login key must contain exactly 32 bytes"))?;
    Ok(ed25519_dalek::SigningKey::from_bytes(&seed)
        .verifying_key()
        .to_bytes())
}

/// The file the robot's remote-plane daemon applies fetched epochs from, in
/// its state directory (`cerulion_remoted::epoch_pull::EPOCH_PENDING_FILE`).
const EPOCH_PENDING_FILE: &str = "epoch.pending";
/// The remote-plane daemon's state subdirectory (`cerulion_remoted::config::REMOTED_SUBDIR`).
const REMOTED_SUBDIR: &str = "remoted";

/// The hidden robot-side revocation sync: fetch this registered robot's current
/// signed access-list epoch from the account service and leave it for the
/// remote-plane daemon to verify and apply. Writes one JSON line
/// `{"version":1,"epoch":N}` (`null` when the robot has no epoch yet).
///
/// The fetch first proves possession of the robot's transport key (the
/// registration's device key): it signs a challenge request over the current
/// time, then signs the challenge it is issued. A request time the service
/// refuses (this clock more than five minutes off, or behind the robot's last
/// accepted request after a step backwards) is retried once at the time the
/// refusal names, so the robot's clock never strands the fetch. Neither needs an
/// account session. A 429 there fails this run with nothing fetched and the
/// login not tried, so the supervisor backs off as for any failed fetch and
/// counts it. When the service answers 404 (no such route, or a service that
/// does not know this robot), or the proof fetch fails otherwise, it falls back
/// to this machine's login session (refreshing a stale one, serialized across
/// processes). Afterwards, while time remains inside the supervisor's worker
/// deadline, it renews a device certificate that is due.
///
/// The stored login is checked locally in both cases: it must name the
/// registration's owner, though its session may have lapsed.
///
/// # Errors
///
/// Refuses loudly when no robot registration exists under `state_root`, when
/// the login is not the registration's owner, when the robot route answers 429,
/// or when both fetches fail; a failure changes nothing on the robot.
pub fn sync_epoch(state_root: &Path, out: &mut impl Write) -> CliResult<()> {
    let started = std::time::Instant::now();
    trusted_root(state_root)?;
    let bundle_path = state_root.join(BUNDLE_FILE);
    let bundle: RegistrationBundle = match fs::symlink_metadata(&bundle_path) {
        Ok(_) => serde_json::from_slice(&read_private(&bundle_path, MAX_BUNDLE_BYTES)?)
            .map_err(|_| refused("the robot registration is malformed"))?,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(refused(&format!(
                "no robot registration exists in {}; the revocation sync runs only on a \
                 registered robot",
                state_root.display()
            )))
        }
        Err(e) => return Err(e.into()),
    };
    // The stored login, read locally: an expired session still names its account.
    let login = auth::load()
        .state()
        .filter(|s| s.logged_in_ever)
        .cloned()
        .ok_or_else(|| refused("run `cerulion login` before serving a robot"))?;
    if hex::encode(account_identity::pairing_account_id(&login.account_id)?.0)
        != bundle.owner_account
    {
        return Err(refused(
            "this login is not the robot registration's owner; revocations were not synced",
        ));
    }
    let robot_id: [u8; 32] = hex::decode(&bundle.robot_id)
        .ok()
        .and_then(|v| v.try_into().ok())
        .ok_or_else(|| refused("the robot registration's robot id is malformed"))?;
    let robot_id_b64 = URL_SAFE_NO_PAD.encode(robot_id);
    let mut session: Option<String> = None;
    let access = match fetch_access_by_robot_proof(&bundle, &robot_id) {
        Ok(ProofFetch::Fetched(access)) => access,
        Ok(ProofFetch::RateLimited) => {
            return Err(refused(
                "the account service is rate limiting this robot's access challenges (429); \
                 revocations were not synced and the login session was not tried",
            ))
        }
        Ok(ProofFetch::NoRoute) => {
            tracing::info!(
                "the account service answered 404 for this robot's access route (an older \
                 service, or one that does not know this robot); fetching revocations with \
                 this machine's login session"
            );
            fetch_access_by_session(&robot_id_b64, &mut session)?
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                "the robot-key revocation fetch failed; fetching with this machine's \
                 login session instead"
            );
            fetch_access_by_session(&robot_id_b64, &mut session)?
        }
    };
    let epoch = match account_cmd::epoch_cache_artifact_from(&access)? {
        None => None,
        Some(artifact) => {
            let postcard = URL_SAFE_NO_PAD
                .decode(artifact.as_bytes())
                .map_err(|_| refused("the fetched epoch did not encode"))?;
            // Only into the remote-plane daemon's provisioned, owner-only
            // directory; this worker never creates it.
            let remoted = state_root.join(REMOTED_SUBDIR);
            match fs::symlink_metadata(&remoted) {
                Ok(meta)
                    if meta.is_dir()
                        && meta.uid() == cerulion_hygiene::euid()
                        && meta.mode() & 0o077 == 0 => {}
                _ => {
                    return Err(refused(
                        "the robot's remote-plane state is not provisioned; revocations were not synced",
                    ))
                }
            }
            let pending = remoted.join(EPOCH_PENDING_FILE);
            auth::atomic_write_secret(&pending, hex::encode(postcard).as_bytes())?;
            Some(access.epoch)
        }
    };
    serde_json::to_writer(
        &mut *out,
        &serde_json::json!({ "version": 1, "epoch": epoch }),
    )
    .map_err(|_| refused("could not encode the worker result"))?;
    writeln!(out)?;
    if started.elapsed() <= WORKER_RENEWAL_START_LIMIT {
        renew_device_cert(session);
    } else {
        tracing::info!(
            "no time left in this revocation sync to renew the device certificate; the next \
             sync checks it"
        );
    }
    Ok(())
}

/// The session fallback of [`sync_epoch`]: the robot's access state over this
/// machine's login session, which is kept in `session` for the renewal after it.
fn fetch_access_by_session(
    robot_id: &str,
    session: &mut Option<String>,
) -> CliResult<account_cmd::RobotAccess> {
    let token = account_cmd::require_session_unrenewed()?;
    let access = account_cmd::fetch_robot_access(&token, robot_id)?;
    *session = Some(token);
    Ok(access)
}

/// The latest the worker starts a device-certificate renewal: 15 s after it
/// starts. A renewal may first refresh the session within
/// [`WORKER_RENEWAL_REFRESH_BUDGET`] (30 s, plus 1 s to publish or retire the
/// rotated pair), then makes two requests of
/// [`device_cert_renewal::RENEW_REQUEST_TIMEOUT`] (15 s each) and takes the store
/// lock once (1 s), 62 s in all, so one started by then ends by 77 s, inside the
/// supervisor's 120 s worker deadline, which starts at spawn.
///
/// [`device_cert_renewal::RENEW_REQUEST_TIMEOUT`]: crate::device_cert_renewal::RENEW_REQUEST_TIMEOUT
const WORKER_RENEWAL_START_LIMIT: Duration = Duration::from_secs(15);

/// The bound on the session refresh a renewal in the worker may need: the
/// refresh-lock wait and the refresh request together. At most one account
/// request (30 s), so netd's default stop grace still lets a refresh in flight
/// save its rotated pair.
const WORKER_RENEWAL_REFRESH_BUDGET: Duration = Duration::from_secs(30);

/// Renew a due device certificate with `session`, or with this machine's login
/// session when the fetch did not need one (refreshed within
/// [`WORKER_RENEWAL_REFRESH_BUDGET`]). Nothing is requested unless the
/// certificate is due; a failure is one warning, and the next attempt waits an
/// hour.
fn renew_device_cert(session: Option<String>) {
    if !crate::device_cert_renewal::is_due_now() {
        return;
    }
    match session.map_or_else(worker_session, Ok) {
        Ok(token) => crate::device_cert_renewal::renew_if_due(&token),
        Err(e) => {
            crate::device_cert_renewal::record_failure();
            crate::device_cert_renewal::warn_failed(&e);
        }
    }
}

/// This machine's session for a renewal in the worker: refreshed when stale,
/// within [`WORKER_RENEWAL_REFRESH_BUDGET`], on this thread.
fn worker_session() -> CliResult<String> {
    login_cmd::refresh_session_if_stale_within(WORKER_RENEWAL_REFRESH_BUDGET)?;
    auth::load()
        .state()
        .filter(|s| s.logged_in_ever && s.session_is_valid(auth::now_unix_ns()))
        .map(|s| s.session_token.clone())
        .ok_or_else(|| refused("this machine's login session has lapsed; run `cerulion login`"))
}

/// Each robot-proof request's timeout: 8 s. The fetch makes up to three (a
/// challenge request, one retry at the time the service names, the access
/// request), 24 s, and with the session fallback after it (a 30 s refresh-lock
/// wait, a 30 s refresh and its 1 s save, a 30 s fetch) the worker ends by 115 s,
/// inside the supervisor's 120 s worker deadline.
const ROBOT_PROOF_REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

#[derive(Deserialize)]
struct RobotAccessChallenge {
    challenge: String,
    robot_id: String,
}

/// What the robot-proof fetch got.
enum ProofFetch {
    /// The robot's access state.
    Fetched(account_cmd::RobotAccess),
    /// The service has no robot access route (404).
    NoRoute,
    /// The service refused the challenge request for the robot's rate (429).
    RateLimited,
}

/// Fetch this robot's access state by proving possession of its transport key:
/// `POST /v1/robots/{id}/access/challenge` with a request signed over the
/// current time, then `POST /v1/robots/{id}/access` with the issued challenge
/// signed, both with the registration's device key. No account session.
fn fetch_access_by_robot_proof(
    bundle: &RegistrationBundle,
    robot_id: &[u8; 32],
) -> CliResult<ProofFetch> {
    use cerulion_pairing::pop::{
        sign_robot_pop, PURPOSE_ROBOT_ACCESS, PURPOSE_ROBOT_ACCESS_CHALLENGE,
    };
    let service = login_cmd::account_service_base();
    let authority = trusted_service_url(&service)?;
    let base = authority.as_str().trim_end_matches('/').to_owned();
    let seed: [u8; 32] = read_private(&bundle.device_key_file, 32)?
        .try_into()
        .map_err(|_| refused("the robot's device key must contain exactly 32 bytes"))?;
    let signing = ed25519_dalek::SigningKey::from_bytes(&seed);
    let transport_key = signing.verifying_key().to_bytes();
    let client = login_cmd::http_client_bounded(ROBOT_PROOF_REQUEST_TIMEOUT)?;
    let id = URL_SAFE_NO_PAD.encode(robot_id);
    let request_challenge = |requested_at: u64| -> CliResult<Answer<RobotAccessChallenge>> {
        let requested_at = requested_at.to_string();
        let request_signature = sign_robot_pop(
            &signing,
            PURPOSE_ROBOT_ACCESS_CHALLENGE,
            robot_id,
            &transport_key,
            &requested_at,
        );
        let response = client
            .post(format!("{base}/v1/robots/{id}/access/challenge"))
            .json(&serde_json::json!({
                "requested_at_ns": requested_at,
                "request_signature": URL_SAFE_NO_PAD.encode(request_signature.0),
            }))
            .send()
            .map_err(|e| refused(&format!("the robot access challenge request failed: {e}")))?;
        read_answer(response, "robot access challenge")
    };
    // A clock refusal names the service's time, and for a replay the next time
    // it accepts; one retry signs the later of the two.
    let mut answer = request_challenge(auth::now_unix_ns())?;
    if let Answer::ClockRefused {
        server_time_ns,
        next_requested_at_ns,
    } = answer
    {
        let retry_at = server_time_ns.max(next_requested_at_ns.unwrap_or(0));
        answer = request_challenge(retry_at)?;
    }
    let challenge = match answer {
        Answer::Body(challenge) => challenge,
        Answer::NotFound => return Ok(ProofFetch::NoRoute),
        Answer::RateLimited => return Ok(ProofFetch::RateLimited),
        Answer::ClockRefused { .. } => {
            return Err(refused(
                "the account service refused the robot's request time twice",
            ))
        }
    };
    if challenge.robot_id != id {
        return Err(refused(
            "the robot access challenge names a different robot",
        ));
    }
    let signature = sign_robot_pop(
        &signing,
        PURPOSE_ROBOT_ACCESS,
        robot_id,
        &transport_key,
        &challenge.challenge,
    );
    let response = client
        .post(format!("{base}/v1/robots/{id}/access"))
        .json(&serde_json::json!({
            "pop_challenge": challenge.challenge,
            "pop_signature": URL_SAFE_NO_PAD.encode(signature.0),
        }))
        .send()
        .map_err(|e| refused(&format!("the robot access request failed: {e}")))?;
    let access = match read_answer::<account_cmd::RobotAccess>(response, "robot access")? {
        Answer::Body(access) => access,
        Answer::NotFound => return Ok(ProofFetch::NoRoute),
        Answer::RateLimited => return Ok(ProofFetch::RateLimited),
        Answer::ClockRefused { .. } => return Err(refused("the robot access request was refused")),
    };
    if access.robot_id != id {
        return Err(refused("the robot access answer names a different robot"));
    }
    Ok(ProofFetch::Fetched(access))
}

/// A response body, or the refusals the robot-proof fetch acts on.
enum Answer<T> {
    Body(T),
    NotFound,
    RateLimited,
    /// A 403 naming the service's clock, and for a replay the next request time
    /// it accepts.
    ClockRefused {
        server_time_ns: u64,
        next_requested_at_ns: Option<u64>,
    },
}

/// A success body as `T` (read up to [`MAX_BUNDLE_BYTES`]); 404 and 429 as
/// themselves; any other status is an error naming it and the service's
/// description.
fn read_answer<T: serde::de::DeserializeOwned>(
    response: reqwest::blocking::Response,
    what: &str,
) -> CliResult<Answer<T>> {
    #[derive(Deserialize, Default)]
    struct ErrorBody {
        #[serde(default)]
        error_description: String,
        #[serde(default)]
        server_time_ns: Option<String>,
        #[serde(default)]
        next_requested_at_ns: Option<String>,
    }
    let status = response.status();
    match status.as_u16() {
        404 => return Ok(Answer::NotFound),
        429 => return Ok(Answer::RateLimited),
        _ => {}
    }
    let mut bytes = Vec::new();
    response
        .take(MAX_BUNDLE_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() > MAX_BUNDLE_BYTES {
        return Err(refused(&format!(
            "the {what} response exceeds its size limit"
        )));
    }
    if !status.is_success() {
        let body = serde_json::from_slice::<ErrorBody>(&bytes).unwrap_or_default();
        let parse = |v: &Option<String>| v.as_deref().and_then(|t| t.parse::<u64>().ok());
        if status.as_u16() == 403 {
            if let Some(server_time_ns) = parse(&body.server_time_ns) {
                return Ok(Answer::ClockRefused {
                    server_time_ns,
                    next_requested_at_ns: parse(&body.next_requested_at_ns),
                });
            }
        }
        let detail = body.error_description;
        return Err(refused(&format!(
            "the {what} request was refused ({status}): {detail}"
        )));
    }
    serde_json::from_slice(&bytes)
        .map(Answer::Body)
        .map_err(|_| refused(&format!("the {what} response is malformed")))
}

/// Prepare a verified account-service registration for the robot writer.
/// This worker never opens a listener or prompts in a background process.
pub fn prepare(state_root: &Path) -> CliResult<BootstrapPrepared> {
    let initial = login_snapshot()?;
    trusted_root(state_root)?;
    create_state_root(state_root)?;
    let metadata = fs::symlink_metadata(state_root)?;
    if metadata.uid() != cerulion_hygiene::euid() || metadata.mode() & 0o077 != 0 {
        let directory = state_root.to_string_lossy().replace('\'', "'\\''");
        return Err(refused(&format!(
            "the robot state directory must be owner-only; run `chmod 700 '{directory}'`"
        )));
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
    login_cmd::enroll_device_once_within(crate::account_robots::LISTING_FETCH_BUDGET)?;
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
        || public_key_at(&bundle.device_key_file).ok() != Some(login.key)
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
        use std::os::unix::fs::PermissionsExt;
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
            key_file: PathBuf::new(),
            key,
        };
        // The bundle recorded the legacy name; the login now reads the
        // canonical one. Both hold the same seed, so the cached identity holds.
        let temp = tempfile::tempdir().unwrap();
        let legacy = temp.path().join("desk.key");
        fs::write(&legacy, [0x33; 32]).unwrap();
        fs::set_permissions(&legacy, fs::Permissions::from_mode(0o600)).unwrap();
        login.key_file = temp.path().join("identity").join("device.key");
        // This helper checks the cached identity only. Complete trust/grant
        // validation belongs to first registration and the persistent writer.
        let bundle = RegistrationBundle {
            version: 1,
            robot_id: "11".repeat(32),
            owner_account: owner_hex.into(),
            device_key_file: legacy.clone(),
            device_cert_postcard: hex::encode(postcard::to_stdvec(&cert).unwrap()),
            root_set_postcard: String::new(),
            intermediate_postcard: String::new(),
            owner_grant_postcard: String::new(),
        };
        let prepared = describe(&bundle, Path::new("/state/robot-registration.json"), &login)
            .expect("UUID and signed account use the pinned shared mapping");
        assert_eq!(prepared.owner_account, owner_hex);
        // A bundle whose key file now holds a different key is refused.
        fs::write(&legacy, [0x44; 32]).unwrap();
        assert!(describe(&bundle, Path::new("/state/robot-registration.json"), &login).is_err());
        fs::write(&legacy, [0x33; 32]).unwrap();
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
