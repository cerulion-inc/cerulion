// SPDX-License-Identifier: AGPL-3.0-only
//! Device-certificate renewal: re-register this machine's EXISTING device key
//! before its cached certificate expires.
//!
//! `cerulion login` and the one-time enrollment of an existing key are the other
//! places a device certificate is issued, and the certificate is short-lived
//! (seven days from the hosted issuer). A caller holding a valid account session
//! runs [`renew_if_due`]; [`renew_once_within`] is its bounded form, at most one
//! attempt per process. It:
//!
//! 1. reads `~/.cerulion/device.cert` and the other consumer copies (netd's
//!    relocated paths and the `device-chain.json` leaf), and does nothing (no
//!    request) unless the certificate is within the renewal window of its
//!    `not_after` ([`RENEW_WINDOW_ENV`], default half the certificate's own
//!    lifetime) or a consumer holding a certificate for the same key and account
//!    holds a different one (a publication cut short). For an hour after a failed
//!    attempt, recorded in `.device-renew.failed` beside `auth.json`, it does
//!    nothing either, so a renewal that cannot succeed costs at most one attempt
//!    an hour;
//! 2. does step 1 holding `.device-renew.lock` beside `auth.json`, taken
//!    WITHOUT waiting (a peer holding it is renewing, so this process skips), so
//!    a certificate a peer has just renewed reads as not due;
//! 3. checks that `auth.json` names the account the certificate binds, then
//!    re-registers the same key (`POST /v1/devices/challenge` + `POST
//!    /v1/devices`) with the cached certificate's principal kind; the service
//!    answers from its stored row for that key, so the device id, account and
//!    principal kind do not change;
//! 4. checks the reissued leaf against the cached one (same key, account and
//!    principal kind, a later `not_after`, signed by the returned intermediate,
//!    and both valid on this machine's clock read after the service answered,
//!    as login checks an issuance);
//! 5. publishes under the store lock, in login's order: every consumer is staged
//!    first, then the certificate copies are renamed into place (`device.cert`
//!    first), then `device-chain.json`. A consumer path is replaced when it is
//!    absent or holds a certificate for the same key and account; a symlink that
//!    already holds the new certificate, or follows `device.cert`, is left as it
//!    is (login's rule); any other content is left alone and named in a log line.
//!    The CLI's own copy and the chain must be written; a relocated copy that
//!    cannot be staged or renamed is skipped and named in a warning, so it alone
//!    stays on the old certificate (recorded as a failed attempt). `auth.json` is
//!    not written. A stop between two renames, or a failed rename of the CLI's
//!    own copy or the chain after another copy was replaced, leaves a consumer on
//!    the old certificate with no failure recorded, so step 1 reads it as due and
//!    the next account command publishes again; until then the owner chain
//!    refuses the mismatch.
//!
//! It never creates a key and never fails the command: a failure is one `warn`
//! line, and the cached certificate stays in use until it expires.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::OnceLock;
use std::time::Duration;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::format::{
    AccountId, PrincipalKind, PublicKey, SignedDeviceCert, SignedIntermediateCert, SignedPayload,
    FORMAT_VERSION,
};

use crate::error::{CliError, CliResult};
use crate::{account_identity, auth, login_cmd, owner_certificate};

/// Env var: how long before the cached device certificate's `not_after` it is
/// renewed, in whole seconds from 60 to 2592000 (thirty days). Unset: half the
/// certificate's own lifetime. A window at least as long as the lifetime renews
/// on every account-touching command.
pub const RENEW_WINDOW_ENV: &str = "CERULION_DEVICE_CERT_RENEW_WINDOW_S";

/// The shortest accepted window: one minute, the length of the renewal's two
/// requests at [`RENEW_REQUEST_TIMEOUT`] plus margin.
const MIN_RENEW_WINDOW_S: u64 = 60;

/// The longest accepted window: thirty days, four times the hosted issuer's
/// seven-day certificate lifetime.
const MAX_RENEW_WINDOW_S: u64 = 30 * 86_400;

/// The per-request timeout of the renewal's two requests: fifteen seconds each,
/// thirty in all, which the robot worker's start limit counts on.
pub const RENEW_REQUEST_TIMEOUT: Duration = Duration::from_secs(15);

/// The most bytes read from a cached certificate file.
const MAX_CERT_BYTES: u64 = 64 * 1024;

/// How long a failed attempt holds the next one back: one hour, so a renewal
/// that cannot succeed (no route, a refused key, a lapsed login, a consumer that
/// cannot be written) costs at most two requests and one warning an hour. The
/// default window is three and a half days, so the retries it leaves are many.
const RETRY_AFTER_FAILURE_NS: u64 = 3600 * 1_000_000_000;

/// What a renewal attempt did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RenewalOutcome {
    /// No readable cached certificate: nothing to renew.
    NoCert,
    /// The certificate is outside its renewal window and every consumer holds
    /// it, or an attempt failed within the last hour. No request was made.
    NotDue {
        /// The cached certificate's `not_after`, Unix nanoseconds.
        not_after_ns: u64,
    },
    /// Another process holds the renewal lock and is renewing this certificate.
    PeerRenewing,
    /// The account service has no device-certificate surface (404). Recorded as
    /// a failed attempt, so the service is asked again at most once an hour.
    IdentityOnly,
    /// The cached certificate or the login changed while the reissued one was in
    /// flight; the reissued one was not published.
    Superseded,
    /// The reissued certificate is cached at the CLI's own copy, in the chain, and
    /// at every relocated consumer that held a certificate for this key and
    /// account, or nothing, except copies that could not be written (named in a
    /// warning, and recorded as a failed attempt).
    Renewed {
        /// The old certificate's `not_after`, Unix nanoseconds.
        old_not_after_ns: u64,
        /// The new certificate's `not_after`, Unix nanoseconds.
        new_not_after_ns: u64,
    },
}

/// The renewal window in nanoseconds for a certificate valid from
/// `not_before_ns` to `not_after_ns`, given the [`RENEW_WINDOW_ENV`] value, and
/// whether that value was rejected. Unset: half the lifetime. Not whole seconds
/// in range: half the lifetime, and `true`.
pub fn renew_window_ns(not_before_ns: u64, not_after_ns: u64, env: Option<&str>) -> (u64, bool) {
    let half_life = not_after_ns.saturating_sub(not_before_ns) / 2;
    let Some(raw) = env else {
        return (half_life, false);
    };
    match raw.trim().parse::<u64>() {
        Ok(secs) if (MIN_RENEW_WINDOW_S..=MAX_RENEW_WINDOW_S).contains(&secs) => {
            (secs.saturating_mul(1_000_000_000), false)
        }
        _ => (half_life, true),
    }
}

/// Whether a certificate expiring at `not_after_ns` is due for renewal at
/// `now_ns` under `window_ns`: at most `window_ns` of validity remains.
pub fn is_due(not_after_ns: u64, now_ns: u64, window_ns: u64) -> bool {
    not_after_ns.saturating_sub(now_ns) <= window_ns
}

/// Whether the cached device certificate is due for renewal now, read locally
/// with no request. `false` when no readable certificate is cached.
pub fn is_due_now() -> bool {
    let (Some(auth_path), Some(cert_path)) = (auth::auth_json_path(), auth::device_cert_path())
    else {
        return false;
    };
    read_cached(&cert_path).is_some_and(|cached| due(&auth_path, &cached, auth::now_unix_ns()))
}

/// Renew the cached device certificate when it is due, over `session_token`.
/// Logs the result; never fails, never blocks on a peer.
pub fn renew_if_due(session_token: &str) {
    match renew_device_cert_if_due(session_token, &auth::now_unix_ns) {
        Ok(RenewalOutcome::Renewed {
            old_not_after_ns,
            new_not_after_ns,
        }) => tracing::info!(
            old_not_after_ns,
            new_not_after_ns,
            "renewed this machine's device certificate with its existing device key"
        ),
        Ok(RenewalOutcome::PeerRenewing) => tracing::debug!(
            "another Cerulion process is renewing the device certificate; not renewing here"
        ),
        Ok(RenewalOutcome::IdentityOnly) => tracing::info!(
            "the account service issues no device certificates, so this machine's certificate \
             was not renewed; the next attempt waits an hour"
        ),
        Ok(RenewalOutcome::NoCert | RenewalOutcome::NotDue { .. } | RenewalOutcome::Superseded) => {
        }
        Err(e) => warn_failed(&e),
    }
}

/// [`renew_if_due`] for the automatic account paths: at most one attempt per
/// process, over the stored session when it is still valid (this never refreshes
/// it), and the caller waits at most `budget`. The attempt runs on its own
/// thread; one still running when the budget ends finishes there, or ends with
/// the process. An exit before the first rename leaves every copy as it was
/// (and may leave a staged temporary file); an exit between renames leaves the
/// mismatch the next account command publishes again.
pub fn renew_once_within(budget: Duration) {
    static ATTEMPTED: AtomicBool = AtomicBool::new(false);
    if crate::graph_cmd::remote_network_suppressed() || !is_due_now() {
        return;
    }
    let Some(session) = auth::load()
        .state()
        .filter(|s| s.logged_in_ever && s.session_is_valid(auth::now_unix_ns()))
        .map(|s| s.session_token.clone())
    else {
        return;
    };
    if ATTEMPTED.swap(true, Ordering::SeqCst) {
        return;
    }
    let (done, finished) = std::sync::mpsc::sync_channel::<()>(1);
    let dispatch = tracing::dispatcher::get_default(Clone::clone);
    let span = tracing::Span::current();
    let spawned = std::thread::Builder::new()
        .name("cer-device-renewal".into())
        .spawn(move || {
            let _dispatch = tracing::dispatcher::set_default(&dispatch);
            let _span = span.enter();
            renew_if_due(&session);
            // The caller may have stopped waiting; nothing is owed to it then.
            let _ = done.send(());
        });
    match spawned {
        Ok(_) => {
            if finished.recv_timeout(budget).is_err() {
                tracing::debug!(
                    budget_ms = budget.as_millis() as u64,
                    "the device certificate renewal is still running past this command's budget"
                );
            }
        }
        Err(e) => tracing::debug!(error = %e, "the device certificate renewal could not start"),
    }
}

/// The one line a failed renewal prints.
pub(crate) fn warn_failed(e: &CliError) {
    let now = auth::now_unix_ns();
    let not_after_ns = auth::device_cert_path()
        .and_then(|p| read_cached(&p))
        .map(|c| c.cert.cert.validity.not_after_ns);
    match not_after_ns {
        Some(not_after_ns) if not_after_ns > now => tracing::warn!(
            error = %e,
            hours_left = (not_after_ns - now) / 3_600_000_000_000,
            "this machine's device certificate could not be renewed; it stays in use until it \
             expires, and `cerulion login` issues a new one"
        ),
        _ => tracing::warn!(
            error = %e,
            "this machine's device certificate has expired or is unreadable and could not be \
             renewed; `cerulion login` issues a new one"
        ),
    }
}

/// [`renew_if_due`] returning what it did, on the clock `now` (Unix
/// nanoseconds). The certificate is read as due at one reading of `now`, and
/// the reissued one is checked at a reading taken after the service answered,
/// so a certificate the service minted on a clock in step with this one is
/// valid here. The window comes from [`RENEW_WINDOW_ENV`]; the paths from
/// `CERULION_HOME` and the netd overrides. A failure is recorded at the first
/// reading, so attempts within the next hour of this clock read as not due; a
/// failure after a copy was replaced is not recorded, so the mismatch it left
/// is due at once.
///
/// # Errors
///
/// A due certificate that could not be renewed: no device key, a key the
/// certificate does not name, a login naming another account, a cloud
/// principal's certificate, a refused or failed request, a reissued certificate
/// that does not match the cached one, or a publication failure.
pub fn renew_device_cert_if_due(
    session_token: &str,
    now: &dyn Fn() -> u64,
) -> CliResult<RenewalOutcome> {
    let (Some(auth_path), Some(cert_path)) = (auth::auth_json_path(), auth::device_cert_path())
    else {
        return Ok(RenewalOutcome::NoCert);
    };
    let _renewing = match auth::try_hold_device_renew_lock(&auth_path) {
        Ok(lock) => lock,
        Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
            return Ok(RenewalOutcome::PeerRenewing)
        }
        Err(e) => {
            return Err(renewal_error(&format!(
                "the renewal lock is unavailable: {e}"
            )))
        }
    };
    // Read under the lock, so a certificate a peer has just renewed reads as
    // not due here.
    let Some(cached) = read_cached(&cert_path) else {
        return Ok(RenewalOutcome::NoCert);
    };
    let now_ns = now();
    if !due(&auth_path, &cached, now_ns) {
        return Ok(not_due(&cached));
    }
    match attempt(session_token, &auth_path, &cert_path, &cached, now_ns, now) {
        Ok(outcome) => {
            if outcome == RenewalOutcome::IdentityOnly {
                record_failure_at(&auth_path, now_ns);
            }
            Ok(outcome)
        }
        Err(failed) => {
            if failed.back_off {
                record_failure_at(&auth_path, now_ns);
            }
            Err(failed.error)
        }
    }
}

/// A failed attempt, and whether the next one waits an hour.
struct Failed {
    error: CliError,
    /// Nothing was replaced: retrying at once would fail the same way. A failure
    /// after a copy was replaced is retried at once instead, since the mismatch it
    /// left keeps the owner chain refusing until the next publication.
    back_off: bool,
}

impl From<CliError> for Failed {
    fn from(error: CliError) -> Self {
        Failed {
            error,
            back_off: true,
        }
    }
}

/// The request and publication half of [`renew_device_cert_if_due`], for a
/// certificate already read as due under the renewal lock at `now_ns`; `now`
/// is read again once the service answered.
fn attempt(
    session_token: &str,
    auth_path: &Path,
    cert_path: &Path,
    cached: &Cached,
    now_ns: u64,
    now: &dyn Fn() -> u64,
) -> Result<RenewalOutcome, Failed> {
    let layout =
        auth::identity_layout().ok_or_else(|| renewal_error("no configuration directory"))?;
    let Some(key) = layout
        .load_machine_key()
        .map_err(|e| renewal_error(&format!("the device key could not be read: {e}")))?
    else {
        return Err(renewal_error("this machine has no device key").into());
    };
    if key.public_key() != cached.cert.cert.device_key.0 {
        return Err(renewal_error(
            "the cached certificate names a different device key than this machine holds",
        )
        .into());
    }
    if login_account(auth_path) != Some(cached.cert.cert.account) {
        return Err(renewal_error(
            "the cached certificate names a different account than this machine's login",
        )
        .into());
    }
    auth::check_netd_cert_paths().map_err(|e| renewal_error(&e))?;
    let principal = registration_principal(cached.cert.cert.principal_kind).ok_or_else(|| {
        renewal_error(
            "the cached certificate is a cloud principal's, which device registration does \
             not issue; `cerulion login` replaces it",
        )
    })?;
    let client = login_cmd::http_client_bounded(RENEW_REQUEST_TIMEOUT)?;
    let base = login_cmd::account_service_base();
    let Some(reissued) =
        login_cmd::reregister_device_key(&client, &base, session_token, key.seed(), principal)?
    else {
        return Ok(RenewalOutcome::IdentityOnly);
    };
    // Read after the response: the service minted the certificate on its own
    // clock after `now_ns` was read.
    let new_not_after_ns = check_reissued(
        &cached.cert,
        &reissued.device_cert,
        &reissued.intermediate,
        now(),
    )?;
    match publish(auth_path, cert_path, cached, &reissued)? {
        Published::Superseded => return Ok(RenewalOutcome::Superseded),
        Published::Complete => clear_failure(auth_path),
        Published::SkippedCopies(copies) => {
            record_failure_at(auth_path, now_ns);
            tracing::warn!(
                paths = %copies.join("; "),
                "renewed this machine's device certificate, but some copies could not be \
                 updated and stay on the old one until it expires"
            );
        }
    }
    Ok(RenewalOutcome::Renewed {
        old_not_after_ns: cached.cert.cert.validity.not_after_ns,
        new_not_after_ns,
    })
}

/// The `principal_kind` device registration takes for a cached certificate's
/// principal: `human` or `machine`. `None` for a cloud principal, which the
/// hosted service holds for itself and registration never issues.
fn registration_principal(kind: PrincipalKind) -> Option<&'static str> {
    match kind {
        PrincipalKind::Human => Some("human"),
        PrincipalKind::Machine => Some("machine"),
        PrincipalKind::Cloud => None,
    }
}

fn not_due(cached: &Cached) -> RenewalOutcome {
    RenewalOutcome::NotDue {
        not_after_ns: cached.cert.cert.validity.not_after_ns,
    }
}

/// The window for `cached` from [`RENEW_WINDOW_ENV`]. A rejected setting is
/// warned about once per process.
fn window_for(cached: &Cached) -> u64 {
    static WARNED: OnceLock<()> = OnceLock::new();
    let env = std::env::var(RENEW_WINDOW_ENV).ok();
    let validity = &cached.cert.cert.validity;
    let (window, rejected) = renew_window_ns(
        validity.not_before_ns,
        validity.not_after_ns,
        env.as_deref(),
    );
    if rejected {
        WARNED.get_or_init(|| {
            tracing::warn!(
                value = ?env.as_deref().unwrap_or_default(),
                min_s = MIN_RENEW_WINDOW_S,
                max_s = MAX_RENEW_WINDOW_S,
                "{RENEW_WINDOW_ENV} is not whole seconds within min_s..=max_s; renewing at half \
                 the certificate's lifetime instead"
            )
        });
    }
    window
}

/// The sibling of `auth.json` holding the time of the last failed attempt.
fn failure_marker(auth_path: &Path) -> PathBuf {
    auth_path.with_file_name(".device-renew.failed")
}

fn record_failure_at(auth_path: &Path, now_ns: u64) {
    if let Err(e) =
        auth::atomic_write_secret(&failure_marker(auth_path), now_ns.to_string().as_bytes())
    {
        tracing::debug!(
            error = %e,
            "could not record the failed device certificate renewal; the next command may \
             try again at once"
        );
    }
}

fn clear_failure(auth_path: &Path) {
    match std::fs::remove_file(failure_marker(auth_path)) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => tracing::debug!(
            error = %e,
            "could not clear the recorded device certificate renewal failure; renewal waits \
             out its hour"
        ),
    }
}

/// Whether an attempt failed less than [`RETRY_AFTER_FAILURE_NS`] before
/// `now_ns`. A failure recorded after `now_ns` (this clock stepped backwards
/// since) holds nothing back.
fn backing_off(auth_path: &Path, now_ns: u64) -> bool {
    std::fs::read_to_string(failure_marker(auth_path))
        .ok()
        .and_then(|t| t.trim().parse::<u64>().ok())
        .is_some_and(|failed_at| failed_at <= now_ns && now_ns - failed_at < RETRY_AFTER_FAILURE_NS)
}

/// Whether `cached` (the CLI's own copy) is due at `now_ns`: no attempt failed in
/// the last [`RETRY_AFTER_FAILURE_NS`], and it is inside its window or a consumer
/// holding a certificate for its key and account holds another.
fn due(auth_path: &Path, cached: &Cached, now_ns: u64) -> bool {
    !backing_off(auth_path, now_ns)
        && (is_due(
            cached.cert.cert.validity.not_after_ns,
            now_ns,
            window_for(cached),
        ) || consumer_differs(auth_path, cached))
}

/// Whether netd's relocated copy or the owner chain's leaf binds the same key and
/// account as `cached` but holds a different certificate.
fn consumer_differs(auth_path: &Path, cached: &Cached) -> bool {
    let binding = (&cached.cert.cert.device_key, &cached.cert.cert.account);
    let differs = |other: Option<Cached>| {
        other.is_some_and(|o| {
            (&o.cert.cert.device_key, &o.cert.cert.account) == binding && o.text != cached.text
        })
    };
    auth::relocated_device_cert_consumer_paths()
        .iter()
        .any(|p| differs(read_cached(p)))
        || differs(chain_leaf(&owner_certificate::cache_path(auth_path)))
}

/// The leaf `device-chain.json` at `path` holds, when readable.
fn chain_leaf(path: &Path) -> Option<Cached> {
    let bytes = read_bounded(path).ok()?;
    let value: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    let text = value.get("device_cert")?.as_str()?.trim().to_owned();
    decode::<SignedDeviceCert>(&text).map(|cert| Cached { text, cert })
}

/// The pairing account `auth.json` at `auth_path` names, when signed in.
fn login_account(auth_path: &Path) -> Option<AccountId> {
    auth::load_from(auth_path)
        .state()
        .filter(|s| s.logged_in_ever)
        .and_then(|s| account_identity::pairing_account_id(&s.account_id).ok())
}

/// A cached certificate: its text exactly as stored (trimmed) and its decoding.
struct Cached {
    text: String,
    cert: SignedDeviceCert,
}

/// The certificate at `path`; `None` when absent, unreadable (not a regular
/// file, too large, no permission) or undecodable. The binding resolver reports a
/// corrupt cache; renewal leaves it alone.
fn read_cached(path: &Path) -> Option<Cached> {
    let bytes = match read_bounded(path) {
        Ok(bytes) => bytes,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return None,
        Err(e) => {
            tracing::debug!(
                path = %path.display(),
                error = %e,
                "a cached device certificate could not be read; renewal leaves it alone"
            );
            return None;
        }
    };
    let text = String::from_utf8(bytes).ok()?.trim().to_owned();
    decode::<SignedDeviceCert>(&text).map(|cert| Cached { text, cert })
}

fn read_bounded(path: &Path) -> std::io::Result<Vec<u8>> {
    use std::io::Read;
    use std::os::unix::fs::OpenOptionsExt;
    // O_NONBLOCK: a FIFO substituted at the path never blocks the command.
    let file = std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK)
        .open(path)?;
    if !file.metadata()?.is_file() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "not a regular file",
        ));
    }
    let mut bytes = Vec::new();
    file.take(MAX_CERT_BYTES + 1).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > MAX_CERT_BYTES {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "larger than a certificate",
        ));
    }
    Ok(bytes)
}

fn decode<T: serde::de::DeserializeOwned>(encoded: &str) -> Option<T> {
    let bytes = URL_SAFE_NO_PAD.decode(encoded.trim()).ok()?;
    let (value, rest) = postcard::take_from_bytes(&bytes).ok()?;
    rest.is_empty().then_some(value)
}

/// Check a reissued leaf against the cached one at `now_ns` and return its
/// `not_after`. The intermediate gets the format checks login's own issuance
/// check gives it; robots verify its root signatures.
///
/// # Errors
///
/// The leaf or intermediate does not decode; the leaf names another key,
/// account or principal kind; it does not outlive the cached one; it names an
/// issuer other than the intermediate; its signature does not verify under that
/// issuer; the intermediate has an invalid format or validity interval or no
/// root signature; or either is not valid at `now_ns` (`not_before` at or
/// before it, `not_after` after it), the check a robot applies.
pub fn check_reissued(
    cached: &SignedDeviceCert,
    device_cert: &str,
    intermediate: &str,
    now_ns: u64,
) -> CliResult<u64> {
    let refuse = |why: &str| renewal_error(&format!("the reissued certificate was refused: {why}"));
    let leaf: SignedDeviceCert = decode(device_cert).ok_or_else(|| refuse("it does not decode"))?;
    let issuer: SignedIntermediateCert =
        decode(intermediate).ok_or_else(|| refuse("its intermediate does not decode"))?;
    let (old, new) = (&cached.cert, &leaf.cert);
    if new.version != FORMAT_VERSION || new.validity.not_before_ns >= new.validity.not_after_ns {
        return Err(refuse("its format or validity interval is invalid"));
    }
    if new.device_key != old.device_key {
        return Err(refuse("it names a different device key"));
    }
    if new.account != old.account {
        return Err(refuse("it names a different account"));
    }
    if new.principal_kind != old.principal_kind {
        return Err(refuse("it names a different principal kind"));
    }
    if new.validity.not_after_ns <= old.validity.not_after_ns {
        return Err(refuse("it does not outlive the cached certificate"));
    }
    if new.issuer_key != issuer.cert.intermediate_key {
        return Err(refuse(
            "it names an issuer other than the returned intermediate",
        ));
    }
    let verifying = ed25519_dalek::VerifyingKey::from_bytes(&new.issuer_key.0)
        .map_err(|_| refuse("its issuer key is not a valid ed25519 key"))?;
    let signature = ed25519_dalek::Signature::from_bytes(&leaf.signature.0);
    verifying
        .verify_strict(&new.signing_payload(), &signature)
        .map_err(|_| refuse("its signature does not verify under its issuer"))?;
    let parent = &issuer.cert;
    if parent.version != FORMAT_VERSION
        || parent.validity.not_before_ns >= parent.validity.not_after_ns
        || issuer.signatures.is_empty()
    {
        return Err(refuse(
            "its intermediate has an invalid format or validity interval, or no root signature",
        ));
    }
    if !new.validity.contains(now_ns) || !parent.validity.contains(now_ns) {
        return Err(refuse("it is not valid on this machine's clock"));
    }
    Ok(new.validity.not_after_ns)
}

/// What publication does with one consumer path.
#[derive(Debug, PartialEq, Eq)]
enum ConsumerAction {
    /// Stage the reissued certificate at it.
    Replace,
    /// A symlink already resolving to the reissued certificate, the CLI's own
    /// copy under another spelling, or a symlink whose chain passes through it:
    /// left as it is.
    AlreadyHolds,
    /// Content that is not a certificate for this key and account: left alone.
    Foreign,
}

/// The one rule every consumer path follows: absent, or holding a certificate
/// for `key` and `account` (through a symlink too), is replaced, unless it is a
/// symlink already holding `new`, or is `own` (the CLI's own copy) under another
/// spelling or a symlink whose chain passes through it; anything else is left
/// alone.
fn consumer_action(
    path: &Path,
    own: &Path,
    key: &PublicKey,
    account: &AccountId,
    new: &str,
) -> ConsumerAction {
    if login_cmd::symlink_already_holds(path, new) || follows_own(path, own) {
        return ConsumerAction::AlreadyHolds;
    }
    match std::fs::symlink_metadata(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return ConsumerAction::Replace,
        Err(_) => return ConsumerAction::Foreign,
        Ok(_) => {}
    }
    match read_cached(path) {
        Some(c) if c.cert.cert.device_key == *key && c.cert.cert.account == *account => {
            ConsumerAction::Replace
        }
        _ => ConsumerAction::Foreign,
    }
}

/// The symlink hops [`follows_own`] walks at most, the kernel's own limit on
/// Linux.
const MAX_LINK_HOPS: usize = 40;

/// Whether `path` is `own`, the CLI's own copy, under another spelling (through
/// a symlinked directory, say), or a symlink whose chain passes through it: the
/// renewal renames the reissued certificate onto `own`, so either follows it
/// and is left as it is. A link that only shares `own`'s final target does not
/// follow it: when `own` is itself a symlink, the rename replaces that link and
/// leaves the shared target with the old certificate.
fn follows_own(path: &Path, own: &Path) -> bool {
    let Some(own) = entry(own) else {
        return false;
    };
    if entry(path).as_ref() == Some(&own) {
        return true;
    }
    let mut hop = path.to_path_buf();
    for _ in 0..MAX_LINK_HOPS {
        if !std::fs::symlink_metadata(&hop).is_ok_and(|m| m.file_type().is_symlink()) {
            return false;
        }
        let Ok(target) = std::fs::read_link(&hop) else {
            return false;
        };
        let next = match hop.parent() {
            Some(dir) => dir.join(target),
            None => target,
        };
        if entry(&next).as_ref() == Some(&own) {
            return true;
        }
        hop = next;
    }
    false
}

/// The directory entry `path` names: its directory, resolved, and its file
/// name as written, so spellings of one entry that differ in their directory
/// (through a symlinked directory, `.` or `..`) compare equal without following
/// the entry itself. Names that differ only in case on a case-insensitive
/// volume still compare unequal.
fn entry(path: &Path) -> Option<PathBuf> {
    let dir = match path.parent() {
        Some(dir) if !dir.as_os_str().is_empty() => dir,
        _ => Path::new("."),
    };
    Some(std::fs::canonicalize(dir).ok()?.join(path.file_name()?))
}

/// What publication did.
enum Published {
    /// The cached certificate or the login changed since it was read: nothing
    /// was written.
    Superseded,
    /// Every target holds the reissued certificate.
    Complete,
    /// The reissued certificate is in place, but these relocated copies could not
    /// be written and keep the old one.
    SkippedCopies(Vec<String>),
}

/// Publish the reissued certificate under the store lock, in login's order.
/// A failure before any copy was replaced backs the next attempt off; one after
/// does not, so the mismatch it left is published again by the next command.
fn publish(
    auth_path: &Path,
    cert_path: &Path,
    cached: &Cached,
    reissued: &login_cmd::ReissuedCert,
) -> Result<Published, Failed> {
    let new = reissued.device_cert.trim();
    let published = auth::with_store_lock(auth_path, || {
        Ok((|| -> Result<Published, Failed> {
            if login_account(auth_path) != Some(cached.cert.cert.account) {
                return Ok(Published::Superseded);
            }
            match read_cached(cert_path) {
                Some(current) if current.text == cached.text => {}
                _ => return Ok(Published::Superseded),
            }
            let (key, account) = (&cached.cert.cert.device_key, &cached.cert.cert.account);
            let mut targets: Vec<PathBuf> = vec![cert_path.to_path_buf()];
            for path in auth::relocated_device_cert_consumer_paths() {
                if targets.contains(&path) {
                    continue;
                }
                match consumer_action(&path, cert_path, key, account, new) {
                    ConsumerAction::Replace => targets.push(path),
                    ConsumerAction::AlreadyHolds => {}
                    ConsumerAction::Foreign => tracing::info!(
                        path = %path.display(),
                        "left this device-cert consumer alone in the renewal: it holds no \
                         certificate for this machine's key and account"
                    ),
                }
            }
            // The CLI's own copy and the chain must be written; a relocated copy
            // that cannot be staged or renamed is skipped and named, so one
            // unwritable consumer does not keep the others on an expiring
            // certificate.
            let mut replaced = false;
            let mut staged = Vec::new();
            let mut skipped: Vec<String> = Vec::new();
            for path in &targets {
                let relocated = path != cert_path;
                if !relocated && login_cmd::symlink_already_holds(path, new) {
                    replaced = true;
                    continue;
                }
                match auth::stage_device_cert_at(path, new) {
                    Ok(pending) => staged.push((relocated, pending)),
                    Err(e) if relocated => skipped.push(format!("{}: {e}", path.display())),
                    Err(e) => return Err(publish_error(path, &e).into()),
                }
            }
            let chain_path = owner_certificate::cache_path(auth_path);
            let chain = owner_certificate::stage_at(&chain_path, new, &reissued.intermediate)
                .map_err(|e| Failed::from(publish_error(&chain_path, &e)))?;
            // Certificate copies first, the chain last: login's order.
            for (relocated, pending) in staged {
                let path = pending.path().to_path_buf();
                match pending.commit() {
                    Ok(()) => replaced = true,
                    Err(e) if relocated => skipped.push(format!("{}: {e}", path.display())),
                    Err(e) => {
                        return Err(Failed {
                            error: publish_error(&path, &e),
                            back_off: !replaced,
                        })
                    }
                }
            }
            chain.commit().map_err(|e| Failed {
                error: publish_error(&chain_path, &e),
                back_off: !replaced,
            })?;
            Ok(if skipped.is_empty() {
                Published::Complete
            } else {
                Published::SkippedCopies(skipped)
            })
        })())
    })
    .map_err(|e| {
        Failed::from(renewal_error(&format!(
            "the account store could not be locked: {e}"
        )))
    })?;
    published
}

fn publish_error(path: &Path, e: &std::io::Error) -> CliError {
    renewal_error(&format!(
        "the reissued certificate could not be cached at {}: {e}",
        path.display()
    ))
}

fn renewal_error(message: &str) -> CliError {
    CliError::Login(format!("device certificate renewal: {message}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_pairing::format::{DeviceCert, IntermediateCert, Scope, Validity};
    use ed25519_dalek::SigningKey;

    const DAY_NS: u64 = 86_400 * 1_000_000_000;

    #[test]
    fn the_default_window_is_half_the_lifetime() {
        // A 7-day certificate renews with 3.5 days left.
        assert_eq!(
            renew_window_ns(0, 7 * DAY_NS, None),
            (7 * DAY_NS / 2, false)
        );
        assert_eq!(renew_window_ns(10, 110, None), (50, false));
        // A degenerate interval yields a zero window, never an underflow.
        assert_eq!(renew_window_ns(200, 100, None), (0, false));
    }

    #[test]
    fn the_window_setting_is_whole_seconds_in_range() {
        assert_eq!(renew_window_ns(0, 7 * DAY_NS, Some("60")).0, 60_000_000_000);
        assert_eq!(renew_window_ns(0, 7 * DAY_NS, Some(" 86400 ")).0, DAY_NS);
        assert_eq!(
            renew_window_ns(0, 7 * DAY_NS, Some("2592000")),
            (30 * DAY_NS, false)
        );
        for bad in ["59", "2592001", "-1", "1.5", "", "3d"] {
            assert_eq!(
                renew_window_ns(0, 7 * DAY_NS, Some(bad)),
                (7 * DAY_NS / 2, true),
                "{bad:?}"
            );
        }
    }

    #[test]
    fn due_means_at_most_the_window_remains() {
        let after = 7 * DAY_NS;
        let window = after / 2;
        assert!(!is_due(after, 0, window));
        assert!(!is_due(after, after - window - 1, window));
        assert!(is_due(after, after - window, window));
        assert!(is_due(after, after - 1, window));
        // An expired certificate is due, never an underflow.
        assert!(is_due(after, after + 5, window));
    }

    #[test]
    fn only_the_principals_device_registration_issues_are_renewed() {
        assert_eq!(registration_principal(PrincipalKind::Human), Some("human"));
        assert_eq!(
            registration_principal(PrincipalKind::Machine),
            Some("machine")
        );
        assert_eq!(registration_principal(PrincipalKind::Cloud), None);
    }

    struct Chain {
        issuer: SigningKey,
        intermediate: String,
        device: [u8; 32],
        account: AccountId,
    }

    const ROOT_SEED: [u8; 32] = [0x01; 32];

    fn chain() -> Chain {
        let issuer = SigningKey::from_bytes(&[0x02; 32]);
        let mut c = Chain {
            intermediate: String::new(),
            issuer,
            device: SigningKey::from_bytes(&[0x03; 32])
                .verifying_key()
                .to_bytes(),
            account: AccountId([0x0a; 32]),
        };
        c.intermediate = intermediate_for(&c, |_| {});
        c
    }

    /// `c`'s intermediate, valid for all time unless `edit` says otherwise, signed
    /// by the test root.
    fn intermediate_for(c: &Chain, edit: impl FnOnce(&mut IntermediateCert)) -> String {
        let mut intermediate = IntermediateCert {
            version: FORMAT_VERSION,
            intermediate_key: PublicKey(c.issuer.verifying_key().to_bytes()),
            max_scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 0,
                not_after_ns: u64::MAX,
            },
            issued_at_ns: 0,
        };
        edit(&mut intermediate);
        let signed = intermediate.sign_by_roots(&[&SigningKey::from_bytes(&ROOT_SEED)]);
        URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&signed).unwrap())
    }

    fn leaf(c: &Chain, not_after: u64, edit: impl FnOnce(&mut DeviceCert)) -> SignedDeviceCert {
        let mut cert = DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(c.device),
            account: c.account,
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity: Validity {
                not_before_ns: 1,
                not_after_ns: not_after,
            },
            issued_at_ns: 1,
            issuer_key: PublicKey(c.issuer.verifying_key().to_bytes()),
        };
        edit(&mut cert);
        cert.sign(&c.issuer)
    }

    fn wire(cert: &SignedDeviceCert) -> String {
        URL_SAFE_NO_PAD.encode(postcard::to_stdvec(cert).unwrap())
    }

    #[test]
    fn a_reissued_certificate_must_be_the_same_binding_and_outlive_the_old_one() {
        let c = chain();
        let old = leaf(&c, 100, |_| {});
        let ok = leaf(&c, 200, |_| {});
        // This machine's clock at 50: inside both the leaf's and the issuer's validity.
        assert_eq!(
            check_reissued(&old, &wire(&ok), &c.intermediate, 50).unwrap(),
            200
        );
        let refused = |cert: SignedDeviceCert, why: &str| {
            let e = check_reissued(&old, &wire(&cert), &c.intermediate, 50)
                .unwrap_err()
                .to_string();
            assert!(e.contains(why), "{e}");
        };
        refused(leaf(&c, 100, |_| {}), "does not outlive");
        refused(
            leaf(&c, 200, |l| l.device_key = PublicKey([0x09; 32])),
            "different device key",
        );
        refused(
            leaf(&c, 200, |l| l.account = AccountId([0x0b; 32])),
            "different account",
        );
        refused(
            leaf(&c, 200, |l| l.principal_kind = PrincipalKind::Machine),
            "different principal kind",
        );
        refused(
            leaf(&c, 200, |l| l.issuer_key = PublicKey([0x07; 32])),
            "issuer other than the returned intermediate",
        );
        refused(
            leaf(&c, 200, |l| l.validity.not_before_ns = 300),
            "validity interval",
        );
        refused(
            leaf(&c, 200, |l| l.version = FORMAT_VERSION + 1),
            "format or validity interval",
        );
        // Signed by the issuer over different bytes.
        let mut forged = ok.clone();
        forged.signature = leaf(&c, 201, |_| {}).signature;
        refused(forged, "signature does not verify");
        // Valid from this clock's reading on and not before: a leaf starting at
        // 50 is accepted at 50, one starting at 51 is not yet valid, and at 200
        // the leaf valid until 200 has expired.
        let starting_at = |not_before: u64| {
            let new = leaf(&c, 200, |l| l.validity.not_before_ns = not_before);
            check_reissued(&old, &wire(&new), &c.intermediate, 50)
        };
        assert_eq!(starting_at(50).unwrap(), 200);
        assert!(starting_at(51)
            .unwrap_err()
            .to_string()
            .contains("not valid on this machine's clock"));
        assert!(check_reissued(&old, &wire(&ok), &c.intermediate, 200)
            .unwrap_err()
            .to_string()
            .contains("not valid on this machine's clock"));
        // The intermediate: expired at this machine's clock, an empty interval, no
        // root signature.
        let issuer_refused = |intermediate: String, now: u64, why: &str| {
            let e = check_reissued(&old, &wire(&ok), &intermediate, now)
                .unwrap_err()
                .to_string();
            assert!(e.contains(why), "{e}");
        };
        issuer_refused(
            intermediate_for(&c, |i| i.validity.not_after_ns = 40),
            50,
            "not valid on this machine's clock",
        );
        issuer_refused(
            intermediate_for(&c, |i| i.validity.not_before_ns = u64::MAX),
            50,
            "invalid format or validity interval, or no root signature",
        );
        issuer_refused(
            intermediate_for(&c, |i| i.version = FORMAT_VERSION + 1),
            50,
            "invalid format or validity interval, or no root signature",
        );
        let mut unsigned: SignedIntermediateCert = decode(&c.intermediate).unwrap();
        unsigned.signatures.clear();
        issuer_refused(
            URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&unsigned).unwrap()),
            50,
            "invalid format or validity interval, or no root signature",
        );
        assert!(check_reissued(&old, "not base64url!", &c.intermediate, 50).is_err());
        assert!(check_reissued(&old, &wire(&ok), "AAAA", 50).is_err());
        // Trailing bytes after the postcard value are refused.
        let mut trailing = postcard::to_stdvec(&ok).unwrap();
        trailing.push(0);
        assert!(
            check_reissued(&old, &URL_SAFE_NO_PAD.encode(trailing), &c.intermediate, 50).is_err()
        );
    }

    #[test]
    fn every_consumer_path_follows_one_rule() {
        use std::os::unix::fs::symlink;
        let c = chain();
        let dir = tempfile::tempdir().unwrap();
        let (key, account) = (PublicKey(c.device), c.account);
        let old = wire(&leaf(&c, 100, |_| {}));
        let new = wire(&leaf(&c, 200, |_| {}));
        let other_key = wire(&leaf(&c, 300, |l| l.device_key = PublicKey([0x09; 32])));
        let other_account = wire(&leaf(&c, 300, |l| l.account = AccountId([0x0b; 32])));
        let at = |name: &str, body: &str| {
            let p = dir.path().join(name);
            std::fs::write(&p, body).unwrap();
            p
        };
        let own = at("device.cert", &old);
        let action = |p: &Path| consumer_action(p, &own, &key, &account, &new);
        // Absent, or holding a certificate for this key and account: replaced.
        assert_eq!(action(&dir.path().join("absent")), ConsumerAction::Replace);
        assert_eq!(
            action(&at("old", &format!("{old}\n"))),
            ConsumerAction::Replace
        );
        assert_eq!(action(&at("new", &new)), ConsumerAction::Replace);
        // A symlink to a certificate for this key and account: replaced too,
        // unless it already holds the new one.
        let old_target = at("old-target", &old);
        let link_old = dir.path().join("link-old");
        symlink(&old_target, &link_old).unwrap();
        assert_eq!(action(&link_old), ConsumerAction::Replace);
        let new_target = at("new-target", &new);
        let link_new = dir.path().join("link-new");
        symlink(&new_target, &link_new).unwrap();
        assert_eq!(action(&link_new), ConsumerAction::AlreadyHolds);
        // A symlink following the CLI's own copy stays a link, through any
        // number of hops and by a relative target too.
        let link_own = dir.path().join("link-own");
        symlink(&own, &link_own).unwrap();
        assert_eq!(action(&link_own), ConsumerAction::AlreadyHolds);
        let link_link_own = dir.path().join("link-link-own");
        symlink("link-own", &link_link_own).unwrap();
        assert_eq!(action(&link_link_own), ConsumerAction::AlreadyHolds);
        // When the CLI's own copy is itself a link to a managed file, a consumer
        // linking to that file directly does not follow the CLI's copy, which the
        // renewal replaces: it is replaced too. One linking to the CLI's copy
        // still follows it.
        let managed = at("managed", &old);
        let own_link = dir.path().join("own-link");
        symlink(&managed, &own_link).unwrap();
        let via_link = |p: &Path| consumer_action(p, &own_link, &key, &account, &new);
        let shares_target = dir.path().join("shares-target");
        symlink(&managed, &shares_target).unwrap();
        assert_eq!(via_link(&shares_target), ConsumerAction::Replace);
        let follows_own_link = dir.path().join("follows-own-link");
        symlink(&own_link, &follows_own_link).unwrap();
        assert_eq!(via_link(&follows_own_link), ConsumerAction::AlreadyHolds);
        // The CLI's own copy reached through a symlinked directory is that copy.
        let aliased = dir.path().join("aliased");
        symlink(dir.path(), &aliased).unwrap();
        assert_eq!(
            via_link(&aliased.join("own-link")),
            ConsumerAction::AlreadyHolds
        );
        assert_eq!(
            action(&aliased.join("device.cert")),
            ConsumerAction::AlreadyHolds
        );
        // Anything else is left alone.
        assert_eq!(action(&at("key", &other_key)), ConsumerAction::Foreign);
        assert_eq!(
            action(&at("account", &other_account)),
            ConsumerAction::Foreign
        );
        assert_eq!(
            action(&at("junk", "not a certificate")),
            ConsumerAction::Foreign
        );
        assert_eq!(action(dir.path()), ConsumerAction::Foreign);
        let dangling = dir.path().join("dangling");
        symlink(dir.path().join("nowhere"), &dangling).unwrap();
        assert_eq!(action(&dangling), ConsumerAction::Foreign);
    }

    fn login_for(account: &AccountId) -> auth::AuthState {
        auth::AuthState {
            account_id: URL_SAFE_NO_PAD.encode(account.0),
            session_token: "s".into(),
            refresh_token: "r".into(),
            expires_at_ns: u64::MAX,
            logged_in_ever: true,
            role: None,
        }
    }

    /// Holds the crate's env lock and points every path the renewal resolves at
    /// `dir`: `CERULION_HOME` there, no netd relocation, no window setting. The
    /// previous values come back on drop.
    struct IsolatedHome {
        saved: Vec<(&'static str, Option<String>)>,
        _lock: std::sync::MutexGuard<'static, ()>,
    }

    impl IsolatedHome {
        fn at(dir: &Path) -> Self {
            let lock = crate::test_env::env_lock();
            let keys = [
                "CERULION_HOME",
                "CERULION_NETD_DEVICE_CERT",
                "CERULION_NETD_DESK_KEY",
                RENEW_WINDOW_ENV,
            ];
            let saved = keys.iter().map(|k| (*k, std::env::var(k).ok())).collect();
            for key in &keys[1..] {
                std::env::remove_var(key);
            }
            std::env::set_var("CERULION_HOME", dir);
            IsolatedHome { saved, _lock: lock }
        }
    }

    impl Drop for IsolatedHome {
        fn drop(&mut self) {
            for (key, value) in &self.saved {
                match value {
                    Some(v) => std::env::set_var(key, v),
                    None => std::env::remove_var(key),
                }
            }
        }
    }

    #[test]
    fn publish_commits_only_while_the_login_and_the_cached_certificate_are_unchanged() {
        let c = chain();
        let dir = tempfile::tempdir().unwrap();
        let _home = IsolatedHome::at(dir.path());
        let auth_path = dir.path().join("auth.json");
        let cert_path = dir.path().join("device.cert");
        let old = wire(&leaf(&c, 100, |_| {}));
        let new = wire(&leaf(&c, 200, |_| {}));
        let cached = Cached {
            text: old.clone(),
            cert: decode(&old).unwrap(),
        };
        let reissued = login_cmd::ReissuedCert {
            device_cert: new.clone(),
            intermediate: c.intermediate.clone(),
        };
        let chain_path = owner_certificate::cache_path(&auth_path);
        std::fs::write(&cert_path, &old).unwrap();
        // Another account signed in since the certificate was read: nothing moves.
        auth::write_to(&auth_path, &login_for(&AccountId([0x0b; 32]))).unwrap();
        assert!(matches!(
            publish(&auth_path, &cert_path, &cached, &reissued),
            Ok(Published::Superseded)
        ));
        assert_eq!(std::fs::read_to_string(&cert_path).unwrap(), old);
        assert!(!chain_path.exists());
        // The cached certificate was replaced since it was read: nothing moves.
        auth::write_to(&auth_path, &login_for(&c.account)).unwrap();
        std::fs::write(&cert_path, "SOMETHING-ELSE").unwrap();
        assert!(matches!(
            publish(&auth_path, &cert_path, &cached, &reissued),
            Ok(Published::Superseded)
        ));
        assert_eq!(
            std::fs::read_to_string(&cert_path).unwrap(),
            "SOMETHING-ELSE"
        );
        assert!(!chain_path.exists());
        // Unchanged: the leaf and the chain both carry the reissued certificate.
        std::fs::write(&cert_path, &old).unwrap();
        assert!(matches!(
            publish(&auth_path, &cert_path, &cached, &reissued),
            Ok(Published::Complete)
        ));
        assert_eq!(std::fs::read_to_string(&cert_path).unwrap(), new);
        let stored: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&chain_path).unwrap()).unwrap();
        assert_eq!(stored["device_cert"], new.as_str());
        assert_eq!(stored["intermediate"], c.intermediate.as_str());
    }

    #[test]
    fn a_netd_copy_linked_to_the_file_the_cli_copy_links_to_is_renewed_too() {
        use std::os::unix::fs::{symlink, DirBuilderExt};
        let c = chain();
        let dir = tempfile::tempdir().unwrap();
        let _home = IsolatedHome::at(dir.path());
        let auth_path = dir.path().join("auth.json");
        let old = wire(&leaf(&c, 100, |_| {}));
        let new = wire(&leaf(&c, 200, |_| {}));
        // The CLI's copy and netd's relocated copy both link to one managed file.
        for sub in ["managed", "netd"] {
            std::fs::DirBuilder::new()
                .mode(0o700)
                .create(dir.path().join(sub))
                .unwrap();
        }
        let managed = dir.path().join("managed").join("device.cert");
        std::fs::write(&managed, &old).unwrap();
        let cert_path = dir.path().join("device.cert");
        symlink(&managed, &cert_path).unwrap();
        let netd_path = dir.path().join("netd").join("device.cert");
        symlink(&managed, &netd_path).unwrap();
        std::env::set_var(auth::NETD_DEVICE_CERT_ENV, &netd_path);
        auth::write_to(&auth_path, &login_for(&c.account)).unwrap();
        let cached = Cached {
            text: old.clone(),
            cert: decode(&old).unwrap(),
        };
        let reissued = login_cmd::ReissuedCert {
            device_cert: new.clone(),
            intermediate: c.intermediate.clone(),
        };
        assert!(matches!(
            publish(&auth_path, &cert_path, &cached, &reissued),
            Ok(Published::Complete)
        ));
        assert_eq!(std::fs::read_to_string(&cert_path).unwrap(), new);
        assert_eq!(std::fs::read_to_string(&netd_path).unwrap(), new);
        // Both links became files of their own; the managed file is not
        // written through them.
        assert_eq!(std::fs::read_to_string(&managed).unwrap(), old);
        assert!(!consumer_differs(
            &auth_path,
            &read_cached(&cert_path).unwrap()
        ));
    }

    #[test]
    fn a_publication_cut_short_reads_as_due_until_every_copy_matches() {
        let c = chain();
        let dir = tempfile::tempdir().unwrap();
        let _home = IsolatedHome::at(dir.path());
        let auth_path = dir.path().join("auth.json");
        let chain_path = owner_certificate::cache_path(&auth_path);
        // A certificate far from expiry: due only through a mismatched copy.
        let new = wire(&leaf(&c, u64::MAX - 1, |_| {}));
        let old = wire(&leaf(&c, u64::MAX - 2, |_| {}));
        let cached = Cached {
            text: new.clone(),
            cert: decode(&new).unwrap(),
        };
        let write_chain = |leaf: &str| {
            let staged = owner_certificate::stage_at(&chain_path, leaf, &c.intermediate).unwrap();
            staged.commit().unwrap();
        };
        // No chain yet (a legacy login): not due.
        assert!(!due(&auth_path, &cached, 10));
        // The certificate renamed into place, the chain still on the old leaf: due.
        write_chain(&old);
        assert!(due(&auth_path, &cached, 10));
        // Every copy matches: not due.
        write_chain(&new);
        assert!(!due(&auth_path, &cached, 10));
        // A chain for another key or account is not this machine's binding.
        write_chain(&wire(&leaf(&c, 5, |l| l.account = AccountId([0x0b; 32]))));
        assert!(!due(&auth_path, &cached, 10));
    }

    #[test]
    fn a_failed_attempt_holds_renewal_back_for_an_hour_of_this_clock() {
        let c = chain();
        let dir = tempfile::tempdir().unwrap();
        let _home = IsolatedHome::at(dir.path());
        let auth_path = dir.path().join("auth.json");
        // Valid from 1 to 2000: inside its half-life window from 1000 on.
        let text = wire(&leaf(&c, 2000, |_| {}));
        let expiring = Cached {
            text: text.clone(),
            cert: decode(&text).unwrap(),
        };
        assert!(due(&auth_path, &expiring, 1500));
        record_failure_at(&auth_path, 1000);
        assert_eq!(
            std::fs::read_to_string(failure_marker(&auth_path)).unwrap(),
            "1000"
        );
        // Inside the hour after the failure: held back, expiry included.
        assert!(!due(&auth_path, &expiring, 1500));
        assert!(!due(
            &auth_path,
            &expiring,
            1000 + RETRY_AFTER_FAILURE_NS - 1
        ));
        // From the hour on: due again.
        assert!(due(&auth_path, &expiring, 1000 + RETRY_AFTER_FAILURE_NS));
        // A failure recorded after this clock's now (it stepped backwards since)
        // holds nothing back.
        record_failure_at(&auth_path, 1_000_000);
        assert!(due(&auth_path, &expiring, 1500));
        // A mismatched copy is held back the same way.
        let far = wire(&leaf(&c, u64::MAX - 1, |_| {}));
        let far = Cached {
            text: far.clone(),
            cert: decode(&far).unwrap(),
        };
        let staged = owner_certificate::stage_at(
            &owner_certificate::cache_path(&auth_path),
            &wire(&leaf(&c, u64::MAX - 2, |_| {})),
            &c.intermediate,
        )
        .unwrap();
        staged.commit().unwrap();
        record_failure_at(&auth_path, 5000);
        assert!(!due(&auth_path, &far, 5001));
        assert!(due(&auth_path, &far, 5000 + RETRY_AFTER_FAILURE_NS));
        // A complete publication clears the record.
        clear_failure(&auth_path);
        assert!(!failure_marker(&auth_path).exists());
        assert!(due(&auth_path, &far, 5001));
    }
}
