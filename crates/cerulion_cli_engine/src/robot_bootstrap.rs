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
use cerulion_pairing::verify::{PairingPresentation, TrustStore, MAX_EPOCH_PENDING_BYTES};
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
/// accepted request after a step backwards) is retried at the time the refusal
/// names, at most twice, so the robot's clock never strands the fetch. Neither
/// needs an account session. A 429 there fails this run with nothing fetched and the
/// login not tried, so the supervisor backs off as for any failed fetch and
/// counts it. When the service answers 404 (no such route, or a service that
/// does not know this robot), or the proof fetch fails otherwise, it falls back
/// to this machine's login session (refreshing a stale one, serialized across
/// processes).
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
            fetch_access_by_session(&robot_id_b64)?
        }
        Err(e) => {
            tracing::warn!(
                error = %e,
                "the robot-key revocation fetch failed; fetching with this machine's \
                 login session instead"
            );
            fetch_access_by_session(&robot_id_b64)?
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
            write_pending_epoch(&remoted, &postcard)?;
            Some(access.epoch)
        }
    };
    serde_json::to_writer(
        &mut *out,
        &serde_json::json!({ "version": 1, "epoch": epoch }),
    )
    .map_err(|_| refused("could not encode the worker result"))?;
    writeln!(out)?;
    Ok(())
}

/// Write an epoch's `postcard` as the epoch file in `remoted`, as hex. Above
/// [`MAX_EPOCH_PENDING_BYTES`], which the remote-plane daemon discards unread, it
/// is refused and nothing is written.
fn write_pending_epoch(remoted: &Path, postcard: &[u8]) -> CliResult<()> {
    let text = hex::encode(postcard).into_bytes();
    if text.len() > MAX_EPOCH_PENDING_BYTES {
        return Err(refused(&format!(
            "the fetched epoch is {} bytes as hex, above the {MAX_EPOCH_PENDING_BYTES}-byte \
             bound the remote-plane daemon applies; revocations were not synced",
            text.len()
        )));
    }
    auth::atomic_write_secret(&remoted.join(EPOCH_PENDING_FILE), &text)?;
    Ok(())
}

/// The session fallback of [`sync_epoch`]: the robot's access state over this
/// machine's login session.
fn fetch_access_by_session(robot_id: &str) -> CliResult<account_cmd::RobotAccess> {
    let token = account_cmd::require_session()?;
    account_cmd::fetch_robot_access(&token, robot_id)
}

/// Each robot-proof request's timeout: 8 s, cut to what is left of
/// [`ROBOT_PROOF_FETCH_BUDGET`].
const ROBOT_PROOF_REQUEST_TIMEOUT: Duration = Duration::from_secs(8);

/// The bound on the whole robot-proof fetch: 24 s, three requests' timeouts. It
/// makes up to four requests (a challenge request, up to two retries at the times
/// the service names, the access request) within it, and with the session
/// fallback after it (a 30 s refresh-lock wait, a 30 s refresh and its 1 s save,
/// a 30 s fetch) the worker ends by 115 s, inside the supervisor's 120 s worker
/// deadline.
const ROBOT_PROOF_FETCH_BUDGET: Duration = Duration::from_secs(24);

/// The session fallback's bound after the robot-proof fetch: a refresh-lock
/// wait, a refresh request and an access read, one account request each, and the
/// refresh's 1 s save: 91 s.
const SESSION_FALLBACK_BOUND: Duration =
    Duration::from_secs(3 * cerulion_discovery::robot_state::ACCOUNT_HTTP_TIMEOUT.as_secs() + 1);

/// The worker's own bound, 115 s, which the supervisor's 120 s worker deadline
/// sits above so the deadline never cuts a session refresh in flight.
const WORKER_BOUND: Duration = Duration::from_secs(115);
const _: () = assert!(
    ROBOT_PROOF_FETCH_BUDGET.as_secs() + SESSION_FALLBACK_BOUND.as_secs() <= WORKER_BOUND.as_secs()
);

/// What a robot-proof request may take at `now`: [`ROBOT_PROOF_REQUEST_TIMEOUT`],
/// cut to what is left of the fetch's `deadline`, or `None` once nothing is left.
fn proof_request_timeout(
    deadline: std::time::Instant,
    now: std::time::Instant,
) -> Option<Duration> {
    deadline
        .checked_duration_since(now)
        .filter(|left| !left.is_zero())
        .map(|left| left.min(ROBOT_PROOF_REQUEST_TIMEOUT))
}

/// The challenge requests one fetch makes at most: the first and two retries. A
/// clock refusal names the service's time, and for a replay the next request
/// time it accepts; a refusal for skew names no next time, so a robot whose
/// accepted request was ahead of the service and whose clock then stepped far
/// behind meets a skew refusal and then a replay refusal.
const ROBOT_PROOF_CHALLENGE_ATTEMPTS: usize = 3;

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
    let deadline = std::time::Instant::now() + ROBOT_PROOF_FETCH_BUDGET;
    let timeout = || -> CliResult<Duration> {
        proof_request_timeout(deadline, std::time::Instant::now())
            .ok_or_else(|| refused("the robot-proof fetch ran out of time"))
    };
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
            .timeout(timeout()?)
            .json(&serde_json::json!({
                "requested_at_ns": requested_at,
                "request_signature": URL_SAFE_NO_PAD.encode(request_signature.0),
            }))
            .send()
            .map_err(|e| refused(&format!("the robot access challenge request failed: {e}")))?;
        read_answer(response, "robot access challenge", MAX_BUNDLE_BYTES)
    };
    // Each retry signs the later of the service's time and the next request
    // time it names.
    let mut requested_at = auth::now_unix_ns();
    let mut attempt = 1;
    let challenge = loop {
        match request_challenge(requested_at)? {
            Answer::Body(challenge) => break challenge,
            Answer::NotFound => return Ok(ProofFetch::NoRoute),
            Answer::RateLimited => return Ok(ProofFetch::RateLimited),
            Answer::ClockRefused { .. } if attempt == ROBOT_PROOF_CHALLENGE_ATTEMPTS => {
                return Err(refused(&format!(
                    "the account service refused the robot's request time \
                     {ROBOT_PROOF_CHALLENGE_ATTEMPTS} times"
                )))
            }
            Answer::ClockRefused {
                server_time_ns,
                next_requested_at_ns,
            } => {
                requested_at = server_time_ns.max(next_requested_at_ns.unwrap_or(0));
                attempt += 1;
            }
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
        .timeout(timeout()?)
        .json(&serde_json::json!({
            "pop_challenge": challenge.challenge,
            "pop_signature": URL_SAFE_NO_PAD.encode(signature.0),
        }))
        .send()
        .map_err(|e| refused(&format!("the robot access request failed: {e}")))?;
    let status = response.status();
    let access = match access_answer_from(status, response)? {
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

/// The largest robot access response read: twice [`MAX_EPOCH_PENDING_BYTES`].
/// The epoch file holds an epoch's postcard as hex, 64 bytes per revoked id; the
/// response holds the same epoch as base64 and each revoked id again as a JSON
/// string, about 89 bytes per id, so the response for the largest epoch the
/// remote-plane daemon applies stays under 1.4 times that file.
const MAX_ROBOT_ACCESS_RESPONSE_BYTES: usize = 2 * MAX_EPOCH_PENDING_BYTES;

/// A success body as `T`, read up to `limit` bytes; 404 and 429 as themselves;
/// a 403 naming the service's time as a clock refusal; any other status is an
/// error naming it and the service's description.
fn read_answer<T: serde::de::DeserializeOwned>(
    response: reqwest::blocking::Response,
    what: &str,
    limit: usize,
) -> CliResult<Answer<T>> {
    let status = response.status();
    answer_from(status, response, what, limit)
}

/// The robot access answer from a response's `status` and `body`, read up to
/// [`MAX_ROBOT_ACCESS_RESPONSE_BYTES`].
fn access_answer_from(
    status: reqwest::StatusCode,
    body: impl Read,
) -> CliResult<Answer<account_cmd::RobotAccess>> {
    answer_from(
        status,
        body,
        "robot access",
        MAX_ROBOT_ACCESS_RESPONSE_BYTES,
    )
}

/// [`read_answer`] over a response's `status` and `body`.
fn answer_from<T: serde::de::DeserializeOwned>(
    status: reqwest::StatusCode,
    body: impl Read,
    what: &str,
    limit: usize,
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
    match status.as_u16() {
        404 => return Ok(Answer::NotFound),
        429 => return Ok(Answer::RateLimited),
        _ => {}
    }
    let mut bytes = Vec::new();
    body.take(limit as u64 + 1).read_to_end(&mut bytes)?;
    if bytes.len() > limit {
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

    /// An account id or device key as the access response spells it.
    fn id_text(n: u32) -> String {
        let mut id = [0u8; 32];
        id[..4].copy_from_slice(&n.to_be_bytes());
        URL_SAFE_NO_PAD.encode(id)
    }

    /// An access response revoking `devices` keys, with no epoch attached.
    fn access_listing(devices: u32) -> String {
        let revoked: Vec<String> = (0..devices).map(id_text).collect();
        serde_json::json!({
            "robot_id": "R",
            "epoch": 7,
            "revoked_accounts": [],
            "revoked_devices": revoked,
        })
        .to_string()
    }

    fn read_access(body: &str) -> CliResult<Answer<account_cmd::RobotAccess>> {
        access_answer_from(
            reqwest::StatusCode::OK,
            std::io::Cursor::new(body.as_bytes()),
        )
    }

    /// A list of 10,000 revoked keys, past the 256 KiB the registration files
    /// are held to, is read whole.
    #[test]
    fn an_access_response_past_the_registration_bound_is_read_whole() {
        let body = access_listing(10_000);
        assert!(body.len() > MAX_BUNDLE_BYTES && body.len() < 1024 * 1024);
        let Answer::Body(access) = read_access(&body).unwrap() else {
            panic!("a success body");
        };
        assert_eq!(access.epoch, 7);
        assert_eq!(access.revoked_devices.len(), 10_000);
        assert_eq!(
            access.revoked_devices[0],
            "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        );
        assert_eq!(access.revoked_devices[9_999], id_text(9_999));
    }

    /// A response of exactly the access bound, 2 MiB, is read; one byte more is
    /// refused with the response named.
    #[test]
    fn an_access_response_over_its_bound_is_refused() {
        assert_eq!(MAX_ROBOT_ACCESS_RESPONSE_BYTES, 2_097_152);
        let listing = access_listing(1);
        let at_bound = format!(
            "{}{listing}",
            " ".repeat(MAX_ROBOT_ACCESS_RESPONSE_BYTES - listing.len())
        );
        assert!(matches!(read_access(&at_bound), Ok(Answer::Body(_))));
        let over = format!(" {at_bound}");
        let err = read_access(&over).err().expect("refused");
        assert!(
            err.to_string()
                .ends_with("the robot access response exceeds its size limit"),
            "{err}"
        );
    }

    /// The signed artifacts the service returns for a robot with `devices`
    /// revoked keys: the access response and the epoch's postcard as the
    /// worker writes it.
    fn epoch_parts(devices: u32) -> (String, Vec<u8>) {
        use cerulion_pairing::format::{
            AccessListEpoch, IntermediateCert, PublicKey, RobotId, Scope, Validity,
        };
        let root = ed25519_dalek::SigningKey::from_bytes(&[1u8; 32]);
        let issuer = ed25519_dalek::SigningKey::from_bytes(&[10u8; 32]);
        let issuer_key = PublicKey(issuer.verifying_key().to_bytes());
        let intermediate = IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: issuer_key,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: 100_000_000_000_000,
            },
            issued_at_ns: 5,
            max_scope: Scope::OWNER_FULL,
        }
        .sign_by_roots(&[&root]);
        let revoked: Vec<PublicKey> = (0..devices)
            .map(|n| {
                let mut key = [0u8; 32];
                key[..4].copy_from_slice(&n.to_be_bytes());
                PublicKey(key)
            })
            .collect();
        let signed_epoch = AccessListEpoch {
            version: FORMAT_VERSION,
            robot: RobotId([0x0B; 32]),
            epoch: 1,
            revoked_accounts: vec![],
            revoked_devices: revoked,
            issued_at_ns: 6,
            issuer_key,
        }
        .sign(&issuer);
        let response = serde_json::json!({
            "robot_id": "R",
            "epoch": 1,
            "revoked_accounts": [],
            "revoked_devices": (0..devices).map(id_text).collect::<Vec<_>>(),
            "signed_epoch": URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&signed_epoch).unwrap()),
            "intermediate": URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&intermediate).unwrap()),
        })
        .to_string();
        let access: account_cmd::RobotAccess = serde_json::from_str(&response).unwrap();
        let artifact = account_cmd::epoch_cache_artifact_from(&access)
            .unwrap()
            .expect("an epoch");
        (
            response,
            URL_SAFE_NO_PAD.decode(artifact.as_bytes()).unwrap(),
        )
    }

    /// With this test's envelope (one root signature, small timestamps), the
    /// largest epoch the remote-plane daemon applies has 16,375 revoked keys, the
    /// most whose file fits its 1 MiB bound; it is written, and one key more is
    /// refused with nothing written. The access response for that epoch is
    /// larger than the file and fits the access bound, under 1.4 times the file.
    #[test]
    fn the_response_for_the_largest_epoch_the_daemon_applies_fits_its_bound() {
        // An envelope with its intermediate and an epoch with no keys: 277 bytes.
        // Each key adds 32, and from 128 keys up to 16,383 the count takes two
        // bytes instead of one: 277 + 1 + 32 * 16,375 = 524,278 bytes, 1,048,556
        // as hex; one key more is 1,048,620.
        let (_, empty) = epoch_parts(0);
        assert_eq!(empty.len(), 277);
        let (response, postcard) = epoch_parts(16_375);
        assert_eq!(postcard.len(), 524_278);
        let remoted = tempfile::tempdir().unwrap();
        write_pending_epoch(remoted.path(), &postcard).unwrap();
        assert_eq!(
            std::fs::read(remoted.path().join(EPOCH_PENDING_FILE)).unwrap(),
            hex::encode(&postcard).into_bytes()
        );
        assert!(response.len() > MAX_EPOCH_PENDING_BYTES);
        assert!(response.len() * 5 < MAX_EPOCH_PENDING_BYTES * 7);
        assert!(matches!(read_access(&response), Ok(Answer::Body(_))));

        let (_, one_more) = epoch_parts(16_376);
        let refused_dir = tempfile::tempdir().unwrap();
        let err = write_pending_epoch(refused_dir.path(), &one_more).unwrap_err();
        assert!(
            err.to_string().ends_with(
                "the fetched epoch is 1048620 bytes as hex, above the 1048576-byte bound the \
                 remote-plane daemon applies; revocations were not synced"
            ),
            "{err}"
        );
        assert!(!refused_dir.path().join(EPOCH_PENDING_FILE).exists());
    }

    #[test]
    fn a_proof_request_gets_its_timeout_cut_to_what_is_left_of_the_fetch() {
        let start = std::time::Instant::now();
        let deadline = start + Duration::from_secs(24);
        let at = |secs: u64| start + Duration::from_secs(secs);
        assert_eq!(
            proof_request_timeout(deadline, start),
            Some(Duration::from_secs(8))
        );
        assert_eq!(
            proof_request_timeout(deadline, at(16)),
            Some(Duration::from_secs(8))
        );
        assert_eq!(
            proof_request_timeout(deadline, at(21)),
            Some(Duration::from_secs(3))
        );
        assert_eq!(proof_request_timeout(deadline, deadline), None);
        assert_eq!(proof_request_timeout(deadline, at(30)), None);
    }

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
