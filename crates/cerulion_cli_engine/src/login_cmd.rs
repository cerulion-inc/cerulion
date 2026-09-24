// SPDX-License-Identifier: AGPL-3.0-only
//! The device-code login FLOW + the first-need runtime login gate.
//!
//! This is the **network-touching** half of login (the local state + the pure gate
//! classifier live in [`crate::auth`]). It:
//!
//! - runs the **RFC 8628 device-authorization grant** ([`run_login`]) against
//!   the account service — headless-friendly (the `user_code` + `verification_uri`
//!   print to the TTY; the user copies them into a browser on any device, exactly
//!   like other headless CLI logins),
//! - registers the machine's ed25519 **device key** (`POST /v1/devices`) and
//!   caches the returned `SignedDeviceCert` — when the service offers that
//!   surface at all; the hosted default does not, and login then takes the
//!   account id from `/v1/me` (see [`DEFAULT_ACCOUNT_SERVICE`]),
//! - persists `~/.cerulion/auth.json` (with `logged_in_ever: true`), and
//! - exposes [`ensure_login_gate`] — the entry every command consults, which
//!   **auto-triggers** the login flow at first-ever need.
//!
//! ## Endpoint resolution
//!
//! The account-service base URL is `CERULION_ACCOUNT_SERVICE` when set, else the
//! production default [`DEFAULT_ACCOUNT_SERVICE`] — the Supabase-backed issuer
//! the web app serves, so the CLI, Studio and app.cerulion.com are one identity.
//! The `cerulion-accountd` binary on localhost is the dev/test target, and
//! the only one that certifies device keys.
//!
//! ## Enforcement
//!
//! The gate is on in every build. Every command outside the small exempt set
//! (`login`, `completions`, and the internal `graph run-worker` / `run-gateway`
//! subprocess verbs) runs under a logged-in-ever identity, in a released binary
//! and in one built from source alike.
//!
//! It is asked once per machine, not once per command. The device-code flow
//! writes a durable `logged_in_ever` marker, and from then on the gate is a
//! local read: it proceeds with zero network, offline, and on an expired
//! session, for as long as the machine lives.
//!
//! One escape exists, for this repository's own runs: `CERULION_LOGIN_GATE` set
//! to exactly `off`. The workspace `.cargo/config.toml` carries it, so
//! everything cargo starts has it, and so do the few workflow jobs that run the
//! binary outside cargo. It is contributor machinery, not product surface: no
//! user documentation names it, and `docs/internals/cli.md` is where a
//! contributor reads what it is for.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::pop::{sign_pop, PURPOSE_DEVICE_REGISTRATION};
use serde::Deserialize;

use crate::auth::{self, AuthState, LocalGate};
use crate::error::{CliError, CliResult};

/// The production account-service base URL: the Supabase-backed issuer the web
/// app serves (`/v1/auth/device/*`, `/v1/auth/refresh`, `/v1/auth/revoke`,
/// `/v1/me`), so `cerulion login` and app.cerulion.com are ONE identity —
/// `account_id` is the Supabase user id, not a shadow account that needs
/// linking. `CERULION_ACCOUNT_SERVICE` overrides it (a local `cerulion-accountd`
/// on loopback is still a valid dev/test target).
///
/// It issues no device certificates: `/v1/devices*` is accountd's surface, and
/// login degrades to `/v1/me` for the account id when the service answers 404
/// there, and `cerulion robot register` — which needs a certified device key —
/// fails naming [`ACCOUNT_SERVICE_ENV`] instead of a bare HTTP error.
pub const DEFAULT_ACCOUNT_SERVICE: &str = "https://app.cerulion.com";

/// Env override for the account-service base URL.
pub const ACCOUNT_SERVICE_ENV: &str = "CERULION_ACCOUNT_SERVICE";

/// Exit code 7 — **the command did not run: authentication required**. Returned
/// by `main`'s first-need login gate ([`ensure_login_gate`]) when an
/// identity-needing command is refused before it executes. It lives here, with
/// the login gate that produces it, so it is platform-independent — unlike the
/// Unix-only `replay_cmd` module, which documents the full CLI-wide exit-code
/// SPACE. It deliberately sits ABOVE the re-execution 0–6 contract so a
/// login-gate refusal can never collide with its `EXIT_VIOLATION`
/// (1): a CI caller running `cerulion bag play --resim all --verify` can
/// therefore distinguish auth-refusal (7) from a data violation (1). NOT a
/// re-execution outcome — that verb never returns it.
pub const EXIT_AUTH_REQUIRED: u8 = 7;

/// Overall wall bound on the poll loop — a backstop over the server's
/// `expires_in` so a broken/hung server never hangs the CLI forever.
const POLL_HARD_CAP: Duration = Duration::from_secs(600);

/// Per-request HTTP timeout.
const HTTP_TIMEOUT: Duration = Duration::from_secs(30);

/// Resolve the account-service base URL (env override → production default),
/// trailing slash trimmed.
pub fn account_service_base() -> String {
    let base = std::env::var(ACCOUNT_SERVICE_ENV)
        .ok()
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| DEFAULT_ACCOUNT_SERVICE.to_string());
    base.trim_end_matches('/').to_string()
}

// ===========================================================================
// account-service response DTOs
// ===========================================================================

#[derive(Deserialize)]
struct DeviceStart {
    device_code: String,
    user_code: String,
    verification_uri: String,
    verification_uri_complete: String,
    #[serde(default)]
    expires_in: u64,
    #[serde(default)]
    interval: u64,
}

#[derive(Deserialize)]
struct TokenResponse {
    session_token: String,
    refresh_token: String,
    #[serde(default)]
    expires_in: u64,
}

/// The RFC 8628 §3.5 error body (`{ "error": "...", "error_description": "..." }`).
#[derive(Deserialize, Default)]
struct ErrorBody {
    #[serde(default)]
    error: String,
    #[serde(default)]
    error_description: String,
}

#[derive(Deserialize)]
struct RegisterDeviceResponse {
    #[allow(dead_code)]
    device_id: String,
    device_cert: String,
    #[allow(dead_code)]
    intermediate: String,
    account_id: String,
}

// ===========================================================================
// the gate entry
// ===========================================================================

/// The one environment variable that switches the gate off, for this
/// repository's own test and CI runs. It is read for exactly `off` and nothing
/// else: no case folding, no trimming, no second spelling, so a value that only
/// looks like it means off leaves the gate on.
const LOGIN_GATE_ENV: &str = "CERULION_LOGIN_GATE";

/// Whether this process is one of this repository's own runs.
///
/// Those runs carry `CERULION_LOGIN_GATE=off`, from the workspace cargo
/// configuration for everything cargo starts, and from a workflow `env:` for the
/// few jobs that run the binary directly. A machine that has never signed in can
/// then run the suite, which is the reason the variable exists.
fn gate_switched_off() -> bool {
    std::env::var(LOGIN_GATE_ENV).is_ok_and(|v| v == "off")
}

/// One line, once per process, so a log can tell a run that proceeded without an
/// identity from one that had an identity to proceed on.
fn gate_off_breadcrumb() {
    use std::sync::Once;
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        tracing::debug!(
            env = LOGIN_GATE_ENV,
            "the login gate is switched off for this run"
        );
    });
}

/// The two-line refusal a machine that has never signed in receives when the
/// device-code flow has nobody to talk to. `main` renders it after `Error: `,
/// then exits [`EXIT_AUTH_REQUIRED`].
///
/// It names BOTH fixes, because the two callers who hit it want different ones:
/// a person who piped the output wants the first, and a CI job or a service
/// wants the second. `CERULION_HOME` is the whole provisioning story on the
/// automation side: point it at a directory holding the state of a machine that
/// signed in, and the gate reads that state instead.
pub const NON_INTERACTIVE_REFUSAL: &str = "This machine has never signed in to a Cerulion account, and every command needs one.\nRun `cerulion login` once in a terminal, or for automation point CERULION_HOME at a directory holding the state of a machine that did.";

/// The refusal an identity-needing command gets on a signed-out machine
/// ([`LocalGate::RefuseSignedOut`]) when no terminal can run the login.
pub const SIGNED_OUT_REFUSAL: &str = "This machine is signed out of its Cerulion account, and every command needs one.\nRun `cerulion login` in a terminal to sign in again.";

/// Whether the device-code flow can actually reach a person here.
///
/// The flow prints a short code and a URL and then blocks for up to ten minutes
/// waiting for someone to approve it in a browser. That only means anything when
/// somebody is reading the stream it prints to AND is present at the session it
/// blocks, so BOTH stderr and stdin must be terminals. Either one redirected is
/// a script, a CI job or a service: it gets the refusal at once rather than a
/// ten minute wait on a prompt nobody is watching.
fn interactive_login_possible() -> bool {
    use std::io::IsTerminal as _;
    std::io::stderr().is_terminal() && std::io::stdin().is_terminal()
}

/// The first-need runtime login gate. Consulted by every
/// identity-needing command. Reads LOCAL state (microsecond-cheap, zero network
/// on the proceed path) and:
///
/// - **logged-in-ever** (valid OR expired-offline) → proceed immediately, no
///   network (I3 — no offline horizon).
/// - **never logged in, at a terminal** → run the device-code login inline. One
///   time: the state it writes makes every later run proceed with zero network.
/// - **never logged in, not at a terminal** → refuse at once with
///   [`NON_INTERACTIVE_REFUSAL`], instead of starting a ten minute poll.
///
/// The one exception is this repository's own runs, which set
/// `CERULION_LOGIN_GATE=off`. See the module docs.
pub fn ensure_login_gate(out: &mut dyn Write) -> CliResult<()> {
    if gate_switched_off() {
        gate_off_breadcrumb();
        return Ok(());
    }
    ensure_login_gate_with(out, interactive_login_possible())
}

/// [`ensure_login_gate`] with the terminal question already answered, so both
/// arms are reachable from a test without a pty.
///
/// This is NOT a bypass and cannot be made into one: `interactive = true` runs
/// the real device-code login against the real account service, `false` refuses.
/// Neither value lets a command run without an identity; the only thing it
/// chooses is which of the two never-signed-in answers a caller gets.
#[doc(hidden)]
pub fn ensure_login_gate_with(out: &mut dyn Write, interactive: bool) -> CliResult<()> {
    let loaded = auth::load();
    match auth::local_gate(&loaded, auth::now_unix_ns()) {
        LocalGate::ProceedValidSession | LocalGate::ProceedExpiredLocalForever => Ok(()),
        gate @ (LocalGate::RefuseNeverLoggedIn | LocalGate::RefuseSignedOut) => {
            if !interactive {
                let refusal = if gate == LocalGate::RefuseSignedOut {
                    SIGNED_OUT_REFUSAL
                } else {
                    NON_INTERACTIVE_REFUSAL
                };
                return Err(CliError::Login(refusal.to_string()));
            }
            writeln!(
                out,
                "\nThis machine is not signed in to a Cerulion account, and Cerulion requires \
                 one. Signing you in now (you can also run `cerulion login` yourself)...\n"
            )?;
            run_login(out)?;
            Ok(())
        }
    }
}

// ===========================================================================
// the device-code login flow
// ===========================================================================

/// Run the RFC 8628 device-authorization login end-to-end against the resolved
/// account service, persist `~/.cerulion/auth.json` (+ cache the device cert),
/// and return the fresh [`AuthState`]. This is both the explicit `cerulion
/// login` verb and the auto-trigger body.
///
/// `out` receives the human-facing prompt (the `user_code` + verification URL +
/// progress) — write it to stderr so it does not pollute a command's stdout.
pub fn run_login(out: &mut dyn Write) -> CliResult<AuthState> {
    let base = account_service_base();
    let client = http_client()?;

    // 1. Start the device-authorization request.
    let start: DeviceStart = post_json(&client, &base, "/v1/auth/device/start", &empty_body())?;

    // 2. Print the headless-friendly prompt.
    print_device_prompt(out, &start)?;

    // 3. Poll until the user authorizes (bounded).
    let tokens = poll_for_tokens(&client, &base, &start, out)?;

    // 4. Register the device key (→ device cert) and resolve the account id.
    //    Device registration is the design's "register the device key at login"
    //    — the returned cert is cached and the account id taken from it.
    //    A service that does not implement `/v1/devices` at all (the hosted
    //    Supabase issuer is identity-only) is the ORDINARY case, not a failure:
    //    `register_device` answers `Ok(None)` for that CONFIRMED 404 surface and
    //    login resolves the account id via `/v1/me`, without a warning that
    //    names nothing the user can fix.
    //    That cert-less path DISCARDS the cached cert: it attests this machine's
    //    key under whichever account issued it, and the resolver treats that as
    //    truth, so keeping it would make the desk act as the previous account
    //    while `auth.json` names the new one. The discard is DEFERRED to the
    //    commit below rather than done here — see step 5.
    //    A cert that IS issued is deferred to the same place, and for the same
    //    reason read the other way round: caching it here, under its own lock,
    //    publishes the new account's device binding while `auth.json` still names
    //    the old one, so an exit or a failed store write in between leaves those
    //    two naming DIFFERENT accounts with nothing to reconcile them.
    //    Any OTHER registration failure (transport, 401, 5xx) is UNKNOWN, not
    //    identity-only, and refuses the login: degrading to `/v1/me` would
    //    delete a cert this service does issue on a blip, and commit an account
    //    switch with no certified key on a machine whose next `robot register`
    //    then has nothing to present. The store is not written yet, so the
    //    previous sign-in and its cert stay exactly as they were.
    let mut discard_cert = false;
    let mut issued_cert: Option<String> = None;
    let account_id = match register_device(&client, &base, &tokens.session_token)? {
        Some(reg) => {
            issued_cert = Some(reg.device_cert);
            reg.account_id
        }
        None => {
            tracing::info!(
                service = %base,
                "the account service issues no device certificates; resolving the account id via /v1/me"
            );
            discard_cert = true;
            fetch_account_id(&client, &base, &tokens.session_token)?
        }
    };

    // 5. Persist the durable local state (logged_in_ever: true).
    //    Carry this machine's install-determined role FORWARD (a
    //    re-login must NOT wipe a role the install funnel stamped). A fresh machine
    //    (no prior auth.json) reads None → resolves to Desk (the documented default);
    //    the install funnel is the writer that sets Robot (deferred — see
    //    `auth::MachineRole`).
    //    The role read and the write that carries it forward are ONE
    //    read-modify-write, so they run under the store lock Studio takes too
    //    (`auth::with_store_lock`): a Studio refresh landing between them would
    //    otherwise be overwritten by a state derived from the store as it was
    //    before that refresh.
    let auth_path = auth::auth_json_path().ok_or_else(|| {
        CliError::Login(
            "no home directory (set CERULION_HOME) to store ~/.cerulion/auth.json".to_string(),
        )
    })?;
    //    The stale-cert discard rides in this same critical section, BEFORE the
    //    write. The two transitions are separately durable and nothing can fuse
    //    them, so one of the two interruption windows has to be chosen, and they
    //    are not symmetric: a kill AFTER the write and before the discard leaves
    //    the new account's tokens beside the OLD account's cert, and the resolver
    //    reads that cert as truth — a cross-account binding no one is told about.
    //    A kill after the discard and before the write leaves the previous
    //    sign-in with no cached cert, which resolves to the account `auth.json`
    //    still names and re-caches on the next certifying login. Losing a cache
    //    beats misbinding an identity, so the discard goes first and a write that
    //    then fails puts the certs back.
    //    A login that WAS issued a cert commits in that same order, for the same
    //    reason — clear, publish the store, then cache the new cert — so no window
    //    holds a cert and an `auth.json` naming different accounts. Its clear runs
    //    only when the account actually CHANGES: re-signing in to the same account
    //    cannot misbind, and clearing there would drop netd's relocated cache for
    //    nothing, since the new cert is written to `~/.cerulion/device.cert` and
    //    netd resolves its own path.
    let cert_path = auth::device_cert_path().ok_or_else(|| {
        CliError::Login(
            "no home directory (set CERULION_HOME) to store ~/.cerulion/device.cert".to_string(),
        )
    })?;
    // The netd overrides are validated on EVERY login, before anything is
    // published — including the ones that clear nothing (a same-account re-login,
    // an identity-only service with nothing cached). A relative override is a
    // property of the configuration, not of the branch this login takes, and a
    // login that skipped the check would report a binding netd resolves against
    // its own working directory instead.
    auth::check_netd_cert_paths().map_err(CliError::Login)?;
    let now = auth::now_unix_ns();
    let mut cert_failure: Option<auth::ClearCertError> = None;
    let mut cert_target_failure: Option<String> = None;
    let mut cert_commit_failure: Option<String> = None;
    let mut certs_cached: Vec<String> = Vec::new();
    let mut certs_lost: Vec<String> = Vec::new();
    let written = auth::with_store_lock(&auth_path, || {
        // An earlier login may have been killed between its clear and its
        // publication, leaving a cert moved aside. Under this same lock, and
        // before this login clears anything, that is finished one way or the
        // other: put back when the store still names the account it certifies,
        // dropped when it does not.
        auth::recover_superseded_device_certs();
        let prior = auth::load_from(&auth_path);
        let (prior_account, prior_role) = prior.prior_identity();
        let state = AuthState {
            account_id,
            session_token: tokens.session_token,
            refresh_token: tokens.refresh_token,
            expires_at_ns: now.saturating_add(tokens.expires_in.saturating_mul(1_000_000_000)),
            logged_in_ever: true,
            role: prior_role,
        };
        // Nothing is published yet, so a clear that refuses needs no rollback of
        // the store: the previous sign-in is still the one on disk, untouched,
        // including a corrupt `auth.json` (a recovery artifact, never deleted).
        let switching_accounts = prior_account.is_none_or(|p| p != state.account_id);
        let cleared = if discard_cert || (issued_cert.is_some() && switching_accounts) {
            match auth::clear_device_cert(prior_account) {
                Ok(cleared) => {
                    if cleared.any() {
                        tracing::info!(
                            replaced = issued_cert.is_some(),
                            "dropped the cached device cert(s) of the previous account: a \
                             retained one keeps naming THAT account for device binding \
                             (`replaced` is whether this login was issued one of its own)"
                        );
                    }
                    Some(cleared)
                }
                Err(e) => {
                    cert_failure = Some(e);
                    return Err(std::io::Error::other("stale device cert"));
                }
            }
        } else {
            None
        };
        // The cert is cached at every path the clear above EMPTIED as well as this
        // CLI's own: netd resolves the cert from its own environment, so a switch
        // that cleared a relocated cache and wrote only `~/.cerulion/device.cert`
        // would leave netd reading nothing — a desk with no WAN binding after a
        // login that reported success. Paths the clear did not touch are left
        // alone: on a same-account re-login they already hold that account's cert,
        // and rewriting one would replace a symlink netd rotates with a regular
        // file.
        //
        // Every one of them is STAGED here, before the store is published, and
        // published afterwards. Staging writes hidden `.device.cert.<pid>.tmp`
        // siblings no consumer reads, so it opens no window in which a cert and an
        // `auth.json` name different accounts, and it moves the only step that can
        // realistically fail (create + write + fsync in a directory that may be
        // unwritable, full or read-only) to BEFORE the publication: a target that
        // cannot take the cert refuses the whole login, certs back where they were,
        // rather than publishing a session for account B with netd still reading
        // nothing. What is left after the publication is a rename within a
        // directory that has just accepted a file.
        let mut staged: Vec<auth::StagedCert> = Vec::new();
        if let Some(cert) = issued_cert.as_deref() {
            let mut targets: Vec<PathBuf> = vec![cert_path.clone()];
            if let Some(cleared) = &cleared {
                for path in cleared.cleared_paths() {
                    if !targets.iter().any(|t| t == path) {
                        targets.push(path.to_path_buf());
                    }
                }
            }
            // A consumer holding NOTHING is cached too, cleared or not. Its two
            // causes are a login killed part-way through the per-consumer commits
            // (the ones after the kill never took it) and an override pointed at a
            // path that has never held one — in both, there is nothing to clear and
            // nothing at the path to preserve, and leaving it empty is a consumer
            // with no device binding after a login that reported success. Paths
            // that DO hold something are still left alone.
            for path in auth::absent_device_cert_consumer_paths() {
                if !targets.contains(&path) {
                    targets.push(path);
                }
            }
            for path in &targets {
                // A path that already holds THIS cert through a symlink is left
                // exactly as it is. The rewrite would be a no-op in bytes and a
                // change in kind — replacing the link with a regular file, which
                // takes the path out of whatever rotates the link's target — and
                // the only case it arises in is the same-account re-login, where
                // there is nothing to update.
                if symlink_already_holds(path, cert) {
                    tracing::info!(
                        path = %path.display(),
                        "left the device cert symlink alone: it already resolves to this \
                         account's cert, and replacing it with a regular file would take the \
                         path out of the rotation the link is part of"
                    );
                    continue;
                }
                match auth::stage_device_cert_at(path, cert) {
                    Ok(pending) => staged.push(pending),
                    Err(e) => {
                        // Nothing is published, and the staging files of the
                        // targets that DID take it are unlinked as `staged` drops.
                        cert_target_failure = Some(format!("{}: {e}", path.display()));
                        if let Some(cleared) = &cleared {
                            if let Err(lost) = cleared.put_back() {
                                certs_lost = lost;
                            }
                        }
                        return Err(std::io::Error::other("device cert target"));
                    }
                }
            }
        }
        if let Err(e) = auth::write_to(&auth_path, &state) {
            // The switch did not happen, so the certs it discarded belong back
            // where they were. Ones that cannot be put back are a LOST binding
            // and are named — the failure is no longer just "could not write".
            if let Some(cleared) = &cleared {
                if let Err(lost) = cleared.put_back() {
                    certs_lost = lost;
                }
            }
            return Err(e);
        }
        // The store now names the account these certs were issued to, so publishing
        // them can no longer split the two. The renames are still one per consumer
        // and nothing fuses them, so a failure part-way through leaves SOME consumers
        // on the new cert and the rest on nothing, and the login reports that rather
        // than returning success. The ones that landed KEEP it: every one of them
        // names the account the store now names, so nothing is misbound, and taking
        // them back out would turn a desk that is partly bound into one that is not
        // bound at all — including a path that held a perfectly good cert for this
        // same account before this login touched it. What is left empty is filled by
        // the next command that resolves the binding (which caches the cert it
        // verified at every consumer that has none) or by the next login.
        let mut committed: Vec<PathBuf> = Vec::new();
        for pending in staged {
            let path = pending.path().to_path_buf();
            match pending.commit() {
                Ok(()) => committed.push(path),
                Err(e) => {
                    cert_commit_failure = Some(format!("{}: {e}", path.display()));
                    // The stagings still pending are unlinked as they drop.
                    break;
                }
            }
        }
        // Published: the asides hold a cert for an account this machine is no
        // longer signed in to, so there is nothing left to roll back to and they
        // are dropped — including on the commit failure below, whose recovery is a
        // fresh login, never the previous account's cert.
        if let Some(cleared) = &cleared {
            cleared.discard();
        }
        if cert_commit_failure.is_some() {
            certs_cached = committed
                .iter()
                .map(|p| p.display().to_string())
                .collect::<Vec<_>>();
        }
        Ok(state)
    });
    let state = match written {
        Ok(state) => state,
        // Nothing was published, so this refusal changed nothing — unless the
        // all-or-nothing clear could not put a cert back, which the message says.
        Err(_) if cert_failure.is_some() => {
            let e = cert_failure.unwrap();
            return Err(stale_cert_error(&e, e.unrestored(), discard_cert));
        }
        // A cert target that could not even be staged: the session is NOT
        // published, so the sign-in has to be re-run once the path is writable —
        // claiming success here would name the new account in the store while a
        // consumer netd reads holds nothing.
        Err(_) if cert_target_failure.is_some() => {
            let lost = if certs_lost.is_empty() {
                String::new()
            } else {
                format!(
                    " — and the device cert(s) this login discarded could not be put back ({}), \
                     so this machine has no cached device binding at all",
                    certs_lost.join("; ")
                )
            };
            return Err(CliError::Login(format!(
                "signed in upstream, but the issued device cert cannot be cached where a \
                 consumer reads it ({}), so nothing was published locally: every consumer \
                 still reads what it read before{lost}. Make that path writable and run \
                 `cerulion login` again",
                cert_target_failure.unwrap()
            )));
        }
        Err(e) => {
            let lost = if certs_lost.is_empty() {
                String::new()
            } else {
                format!(
                    " — and the device cert(s) this login discarded could not be put back \
                     ({}), so this machine has no cached device binding: run `cerulion login` \
                     again once the store is writable",
                    certs_lost.join("; ")
                )
            };
            return Err(CliError::Login(format!(
                "could not write ~/.cerulion/auth.json: {e}{lost}"
            )));
        }
    };
    // The session is published and correct; the cert cache is not, and a caller
    // told "signed in" would go on to use a WAN plane with no device binding. It
    // is reported as a failure of THIS login, naming the path to fix.
    if let Some(failed) = cert_commit_failure {
        let cached = if certs_cached.is_empty() {
            " — no consumer on this desk has one, so it resolves no device binding".to_string()
        } else {
            format!(
                " — these caches did take it and KEEP it ({}), since they name the account \
                 just signed in, so the consumers reading them are bound and the rest are \
                 not",
                certs_cached.join("; ")
            )
        };
        return Err(CliError::Login(format!(
            "signed in as {}, but the issued device cert could not be published where a \
             consumer reads it ({failed}){cached}. Run `cerulion login` again with that \
             path writable",
            state.account_id
        )));
    }
    // NOT "account owner" — ownership binds at robot registration (`POST /v1/robots`); a desk
    // (Studio ⊃ CLI) signs in but owns no robot.
    writeln!(out, "\nSigned in as {}.", state.account_id)?;
    Ok(state)
}

/// Whether `path` is a SYMLINK that already resolves to exactly `cert` — the one
/// case where a cert target is left untouched rather than rewritten.
///
/// It arises on a same-account re-login: the cert has not changed, so the write
/// would alter nothing but the KIND of the entry, replacing a link (which whatever
/// maintains the target keeps rotating) with a regular file this CLI owns. A
/// regular file at the path is still rewritten — that is the CLI's own file, and
/// leaving it alone would be indistinguishable from never having checked.
///
/// Only a link to a REGULAR file is read: following one to a FIFO with no writer
/// would hang the login, so anything else answers `false` and takes the ordinary
/// replace path (which unlinks the link rather than writing through it).
fn symlink_already_holds(path: &Path, cert: &str) -> bool {
    match std::fs::symlink_metadata(path) {
        Ok(md) if md.file_type().is_symlink() => {}
        _ => return false,
    }
    match std::fs::metadata(path) {
        Ok(md) if md.is_file() => {}
        _ => return false,
    }
    std::fs::read(path).is_ok_and(|bytes| bytes == cert.as_bytes())
}

/// The refusal a login answers with when the stale cert outlives it. `e` names
/// the files that are still there — which is not always `~/.cerulion/device.cert`,
/// since netd's env overrides relocate the cache and naming the wrong file hands
/// the user the wrong recovery target.
///
/// `unrestored` is the second failure: the all-or-nothing clear removed those
/// certs and could not put them back, so the previous sign-in did NOT survive
/// intact — its device binding is gone. Claiming it was "left in place" would
/// send the user away from the only recovery there is (sign in again to be
/// re-certified), so the message distinguishes the two.
///
/// `identity_only` picks the CAUSE clause: a service that certifies nothing is a
/// different problem from a certifying one that just moved this machine to
/// another account, and a user told the wrong one looks for the wrong fix.
fn stale_cert_error(
    e: &auth::ClearCertError,
    unrestored: &[String],
    identity_only: bool,
) -> CliError {
    let outcome = if unrestored.is_empty() {
        "Your previous sign-in is left in place, exactly as it was. Delete the file(s) named \
         above and run `cerulion login` again"
            .to_string()
    } else {
        // NOT "delete the file(s) named above": these ones are already gone, and
        // the only thing left to do about them is to be certified again.
        format!(
            "and the device cert at {} was cleared and could NOT be put back, so the previous \
             sign-in's device binding is gone even though its session is untouched. Delete any \
             of the file(s) named above that is still there, then run `cerulion login` again to \
             be certified afresh",
            unrestored.join("; ")
        )
    };
    let cause = if identity_only {
        "this account service issues no device certificate, and one cached from an earlier \
         login could not be removed"
    } else {
        "this login was certified for a different account, and the previous account's cached \
         certificate could not be removed"
    };
    CliError::Login(format!(
        "{cause}: {e} — it would keep naming that account for device binding. {outcome}"
    ))
}

/// Print the RFC 8628 device prompt: the verification URL + short code, ready to
/// copy from an ssh session into a browser on any device.
fn print_device_prompt(out: &mut dyn Write, start: &DeviceStart) -> CliResult<()> {
    writeln!(
        out,
        "To finish signing in, open this URL in a browser on any device:\n\n    {}\n\n\
         and enter the code:\n\n    {}\n",
        start.verification_uri, start.user_code
    )?;
    writeln!(
        out,
        "(or open the direct link: {})\n",
        start.verification_uri_complete
    )?;
    writeln!(out, "Waiting for authorization...")?;
    Ok(())
}

/// Poll `/v1/auth/device/poll` until the user authorizes, honoring the server's
/// `interval` + RFC 8628 `slow_down`, bounded by `expires_in` and a hard cap.
fn poll_for_tokens(
    client: &reqwest::blocking::Client,
    base: &str,
    start: &DeviceStart,
    out: &mut dyn Write,
) -> CliResult<TokenResponse> {
    let mut interval = Duration::from_secs(start.interval.max(1));
    let expiry = if start.expires_in == 0 {
        POLL_HARD_CAP
    } else {
        Duration::from_secs(start.expires_in).min(POLL_HARD_CAP)
    };
    let deadline = Instant::now() + expiry;
    let body = serde_json::json!({ "device_code": start.device_code });
    loop {
        if Instant::now() >= deadline {
            return Err(CliError::Login(
                "login timed out waiting for authorization — run `cerulion login` again".into(),
            ));
        }
        std::thread::sleep(interval);
        let resp = client
            .post(format!("{base}/v1/auth/device/poll"))
            .json(&body)
            .send()
            .map_err(|e| CliError::Login(format!("polling the account service failed: {e}")))?;
        if resp.status().is_success() {
            return resp
                .json::<TokenResponse>()
                .map_err(|e| CliError::Login(format!("could not parse the token response: {e}")));
        }
        let err: ErrorBody = resp.json().unwrap_or_default();
        match err.error.as_str() {
            "authorization_pending" => {
                let _ = write!(out, ".");
                let _ = out.flush();
                continue;
            }
            // RFC 8628: back off by +5s and keep polling. Clamp to the time
            // left before `deadline` so a server spamming slow_down can never
            // inflate the sleep past the POLL_HARD_CAP backstop.
            "slow_down" => {
                interval += Duration::from_secs(5);
                let remaining = deadline.saturating_duration_since(Instant::now());
                interval = interval.min(remaining.max(Duration::from_secs(1)));
                continue;
            }
            "expired_token" => {
                return Err(CliError::Login(
                    "the login code expired before you authorized — run `cerulion login` again"
                        .into(),
                ));
            }
            other => {
                let detail = if err.error_description.is_empty() {
                    other.to_string()
                } else {
                    err.error_description
                };
                return Err(CliError::Login(format!("login failed: {detail}")));
            }
        }
    }
}

// ===========================================================================
// sign-out
// ===========================================================================

/// What [`run_logout`] did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LogoutOutcome {
    /// No session was stored here (never signed in, or already signed out);
    /// nothing was changed.
    NotSignedIn,
    /// The local session was removed and the account service revoked it.
    SignedOut {
        /// The account this machine was signed in to.
        account_id: String,
    },
    /// The local session was removed, but the account service did not confirm
    /// the revoke, so the session stays valid there until it expires.
    SignedOutUnrevoked {
        /// The account this machine was signed in to.
        account_id: String,
        /// Why the revoke was not confirmed.
        reason: String,
    },
}

/// Sign this machine out: remove the session from `~/.cerulion/auth.json`,
/// then revoke it at the account service (`POST /v1/auth/revoke`).
///
/// The local credential goes FIRST, under the store lock, so a sign-out with
/// no network still signs the machine out; the rewritten store keeps the
/// account id and `logged_in_ever` (see [`auth::signed_out_store`]) and every
/// identity-needing command is then refused until the next `cerulion login`.
/// The revoke is the same request Studio's "Sign out" sends, and the service
/// treats an unknown token as success, so a repeat is harmless.
///
/// A device certificate cached by a certifying issuer (`cerulion-accountd`)
/// is left in place: it is revoked with `cerulion account devices revoke`,
/// and the next `cerulion login` replaces it.
///
/// A machine with no `auth.json` is left untouched (not even the store lock
/// file is created).
///
/// # Errors
///
/// The store could not be read or rewritten; nothing was signed out. A revoke
/// the service did not confirm is [`LogoutOutcome::SignedOutUnrevoked`], not
/// an error: the machine IS signed out.
pub fn run_logout() -> CliResult<LogoutOutcome> {
    let auth_path = auth::auth_json_path().ok_or_else(|| {
        CliError::Login(
            "no home directory (set CERULION_HOME) to locate ~/.cerulion/auth.json".to_string(),
        )
    })?;
    if !auth_path.try_exists().unwrap_or(true) {
        return Ok(LogoutOutcome::NotSignedIn);
    }
    let signed_out = auth::with_store_lock(&auth_path, || {
        let state = match auth::load_from(&auth_path) {
            auth::LoadedAuth::Present(state) => state,
            auth::LoadedAuth::Absent | auth::LoadedAuth::SignedOut { .. } => return Ok(None),
            auth::LoadedAuth::Corrupt(reason) => {
                return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, reason))
            }
        };
        let prior = std::fs::read(&auth_path)?;
        let body = auth::signed_out_store(&prior)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        auth::publish_store_bytes(&auth_path, &body)?;
        Ok(Some(state))
    })
    .map_err(|e| {
        CliError::Login(format!(
            "could not sign out: {} was not changed ({e})",
            auth_path.display()
        ))
    })?;
    let Some(state) = signed_out else {
        return Ok(LogoutOutcome::NotSignedIn);
    };
    Ok(match revoke_session(&state) {
        Ok(()) => LogoutOutcome::SignedOut {
            account_id: state.account_id,
        },
        Err(e) => LogoutOutcome::SignedOutUnrevoked {
            account_id: state.account_id,
            reason: e.to_string(),
        },
    })
}

/// `POST /v1/auth/revoke` for `state`'s session. The body names both tokens:
/// the hosted issuer retires the session by its `refresh_token`,
/// `cerulion-accountd` by `token` (either of the pair).
fn revoke_session(state: &AuthState) -> CliResult<()> {
    let base = account_service_base();
    let client = http_client()?;
    let path = "/v1/auth/revoke";
    let resp = client
        .post(format!("{base}{path}"))
        .bearer_auth(&state.session_token)
        .json(&serde_json::json!({
            "refresh_token": state.refresh_token,
            "token": state.refresh_token,
        }))
        .send()
        .map_err(|e| {
            CliError::Login(format!("the account service ({base}) is unreachable: {e}"))
        })?;
    let status = resp.status();
    if status.is_success() {
        return Ok(());
    }
    let err: ErrorBody = resp.json().unwrap_or_default();
    Err(CliError::Login(format!(
        "{path} refused ({status}): {}",
        if err.error_description.is_empty() {
            err.error
        } else {
            err.error_description
        }
    )))
}

/// Opportunistically refresh a STALE session: if this
/// logged-in-ever machine's cached session is expired, exchange the refresh
/// token for a fresh session (`POST /v1/auth/refresh`) and persist it. Returns
/// the fresh state, or `None` when nothing needed refreshing — and also when
/// another writer (Studio, a second `cerulion`) moved the store while the
/// exchange was in flight, in which case the refreshed session is deliberately
/// NOT published (see the publication step).
///
/// This is the **WAN-operation companion** (the fill side of [`auth::wan_gate`]'s
/// row 4) — a cloud-touching op calls it before dialing so it presents a valid
/// session. It is deliberately NOT wired into the local gate
/// ([`ensure_login_gate`]): the local proceed path stays ZERO-network (no
/// offline horizon), and a refresh failure here NEVER blocks local operation.
///
/// Its production caller is `account_cmd::require_session` — the one
/// cloud-dial seam (do not re-implement
/// staleness handling a second way there). The e2e
/// (`stale_session_refreshes_against_the_real_service`) is its live proof.
pub fn refresh_session_if_stale() -> CliResult<Option<AuthState>> {
    let loaded = auth::load();
    let Some(state) = loaded.state() else {
        // Never logged in (or corrupt) — nothing to refresh (the gate handles it).
        return Ok(None);
    };
    if state.session_is_valid(auth::now_unix_ns()) {
        return Ok(None); // still fresh — no cloud call.
    }
    let base = account_service_base();
    let client = http_client()?;
    let refreshed: TokenResponse = post_json(
        &client,
        &base,
        "/v1/auth/refresh",
        &serde_json::json!({ "refresh_token": state.refresh_token }),
    )?;
    let now = auth::now_unix_ns();
    let new_state = AuthState {
        account_id: state.account_id.clone(),
        session_token: refreshed.session_token,
        refresh_token: refreshed.refresh_token,
        expires_at_ns: now.saturating_add(refreshed.expires_in.saturating_mul(1_000_000_000)),
        logged_in_ever: true,
        // A token refresh preserves this machine's install-determined
        // role (it is orthogonal to the session).
        role: state.role,
    };
    // Publish under the store lock, and only if the store still holds the refresh
    // token we exchanged. The lock is taken HERE rather than around the whole
    // function on purpose: an HTTP round trip inside it would make every peer
    // writer (Studio's sign-out, a concurrent `cerulion login`) wait on the
    // network, and the lock's whole value is that it is held for microseconds.
    // So the exchange runs unlocked and the publication is conditional — a peer
    // that signed out or logged in while we were refreshing is NOT overwritten
    // with the session it just retired.
    let auth_path = auth::auth_json_path().ok_or_else(|| {
        CliError::Login(
            "no home directory (set CERULION_HOME) to store ~/.cerulion/auth.json".to_string(),
        )
    })?;
    let published = auth::with_store_lock(&auth_path, || {
        match auth::load_from(&auth_path).state() {
            Some(current) if current.refresh_token == state.refresh_token => {
                auth::write_to(&auth_path, &new_state).map(|()| true)
            }
            // Absent, corrupt, or a different pair: another writer moved the
            // store while the exchange was in flight, and its state is newer
            // than ours.
            _ => Ok(false),
        }
    })
    .map_err(|e| CliError::Login(format!("could not persist the refreshed session: {e}")))?;
    if !published {
        // The exchange rotated the session onto `new_state` (refresh is
        // single-use), and nothing on this machine holds that pair: a sign-out
        // revoked only the pair we exchanged, and a login or a peer's refresh
        // holds a different session. Retire it whatever replaced it, so no
        // session stays live at the service that this machine cannot sign out.
        if let Err(e) = revoke_session(&new_state) {
            tracing::warn!(error = %e, "could not revoke the unpublished refreshed session");
        }
        tracing::warn!(
            path = %auth_path.display(),
            "the local account state changed while the session was being refreshed — the \
             refreshed session was NOT written (the other writer's state stands)"
        );
        return Ok(None);
    }
    Ok(Some(new_state))
}

/// Register this machine's ed25519 device key with the account (`POST
/// /v1/devices`) → a `SignedDeviceCert` (cached) + the account id, or `None`
/// when the service has no device-registration surface (a 404 from either
/// endpoint).
///
/// Proves possession of the device key: fetches a fresh single-use
/// challenge and signs it with the key before presenting it, so the account service
/// certifies only a key the caller actually controls (closing the device-key
/// registration-squatting DoS).
fn register_device(
    client: &reqwest::blocking::Client,
    base: &str,
    session_token: &str,
) -> CliResult<Option<RegisterDeviceResponse>> {
    let seed = ensure_device_key_seed()?;
    let signing = ed25519_dalek::SigningKey::from_bytes(&seed);
    let public_key = signing.verifying_key().to_bytes();
    let Some((pop_challenge, pop_account)) = fetch_pop_challenge(client, base, session_token)?
    else {
        return Ok(None);
    };
    let pop_signature = sign_pop(
        &signing,
        PURPOSE_DEVICE_REGISTRATION,
        &pop_account,
        &public_key,
        &pop_challenge,
    );
    let body = serde_json::json!({
        "public_key": URL_SAFE_NO_PAD.encode(public_key),
        "principal_kind": "human",
        "pop_challenge": pop_challenge,
        "pop_signature": URL_SAFE_NO_PAD.encode(pop_signature.0),
    });
    let resp = client
        .post(format!("{base}/v1/devices"))
        .bearer_auth(session_token)
        .json(&body)
        .send()
        .map_err(|e| CliError::Login(format!("registering the device key failed: {e}")))?;
    if resp.status().as_u16() == 404 {
        return Ok(None);
    }
    if !resp.status().is_success() {
        let status = resp.status();
        let err: ErrorBody = resp.json().unwrap_or_default();
        return Err(CliError::Login(format!(
            "device registration refused ({status}): {}",
            if err.error_description.is_empty() {
                err.error
            } else {
                err.error_description
            }
        )));
    }
    resp.json::<RegisterDeviceResponse>()
        .map(Some)
        .map_err(|e| CliError::Login(format!("could not parse the device-cert response: {e}")))
}

/// The `POST /v1/devices/challenge` response: the single-use PoP
/// challenge + the account it is bound to (so the client signs the exact message the
/// service rebuilds).
#[derive(Deserialize)]
struct ChallengeResponse {
    challenge: String,
    account_id: String,
    #[allow(dead_code)]
    #[serde(default)]
    expires_in: u64,
}

/// Fetch a fresh proof-of-possession challenge (`POST /v1/devices/challenge`,
/// session-authed) and return `(challenge, account)` — the account is the 32-byte
/// `AccountId` the challenge is bound to, decoded from the response so the caller signs
/// the exact message the service rebuilds. Shared by the device-registration
/// (login) + robot-registration (install) PoP paths so both fetch challenges the same
/// way. A stale session (401) names the fix (`cerulion login`).
///
/// `None` means this service has no such endpoint (404) — the hosted
/// Supabase-backed issuer is identity-only, and a caller that needs a
/// certificate must say so itself rather than read a 404 as a network fault.
pub(crate) fn fetch_pop_challenge(
    client: &reqwest::blocking::Client,
    base: &str,
    session_token: &str,
) -> CliResult<Option<(String, [u8; 32])>> {
    let resp = client
        .post(format!("{base}/v1/devices/challenge"))
        .bearer_auth(session_token)
        .send()
        .map_err(|e| {
            CliError::Login(format!(
                "requesting a proof-of-possession challenge ({base}) failed: {e}"
            ))
        })?;
    if resp.status().as_u16() == 404 {
        return Ok(None);
    }
    if !resp.status().is_success() {
        let status = resp.status();
        let err: ErrorBody = resp.json().unwrap_or_default();
        let detail = if err.error_description.is_empty() {
            err.error
        } else {
            err.error_description
        };
        if status.as_u16() == 401 {
            return Err(CliError::Login(format!(
                "the account service refused the session ({status}): {detail} — run `cerulion login`"
            )));
        }
        return Err(CliError::Login(format!(
            "requesting a proof-of-possession challenge refused ({status}): {detail}"
        )));
    }
    let ch: ChallengeResponse = resp.json().map_err(|e| {
        CliError::Login(format!(
            "could not parse the /v1/devices/challenge response: {e}"
        ))
    })?;
    let account: [u8; 32] = URL_SAFE_NO_PAD
        .decode(&ch.account_id)
        .map_err(|e| CliError::Login(format!("challenge account_id is not base64url: {e}")))?
        .try_into()
        .map_err(|_| CliError::Login("challenge account_id is not 32 bytes".into()))?;
    Ok(Some((ch.challenge, account)))
}

/// Resolve the caller's account id via `/v1/me` (the fallback when device
/// registration did not return it).
fn fetch_account_id(
    client: &reqwest::blocking::Client,
    base: &str,
    session_token: &str,
) -> CliResult<String> {
    #[derive(Deserialize)]
    struct Me {
        account_id: String,
    }
    let resp = client
        .get(format!("{base}/v1/me"))
        .bearer_auth(session_token)
        .send()
        .map_err(|e| CliError::Login(format!("resolving the account (/v1/me) failed: {e}")))?;
    if !resp.status().is_success() {
        return Err(CliError::Login(format!(
            "/v1/me refused the session ({})",
            resp.status()
        )));
    }
    resp.json::<Me>()
        .map(|m| m.account_id)
        .map_err(|e| CliError::Login(format!("could not parse /v1/me: {e}")))
}

/// Load the 32-byte ed25519 device seed from `~/.cerulion/desk.key`, creating it
/// (0600) if absent. Shared with the `cerulion connect`/`pair` desk identity —
/// one device key per machine (Studio ⊃ CLI). `pub(crate)` so the robot-claim
/// flow ([`crate::robot_cmd`]) presents the SAME machine key as its transport
/// identity (never a second device key).
pub(crate) fn ensure_device_key_seed() -> CliResult<[u8; 32]> {
    let path = auth::device_key_path().ok_or_else(|| {
        CliError::Login("no home directory (set CERULION_HOME) to store the device key".into())
    })?;
    ensure_device_key_seed_at(&path)
}

/// The path-explicit body of [`ensure_device_key_seed`] (testable without env).
fn ensure_device_key_seed_at(path: &Path) -> CliResult<[u8; 32]> {
    if path.exists() {
        let bytes = std::fs::read(path).map_err(|e| {
            CliError::Login(format!("reading the device key {}: {e}", path.display()))
        })?;
        let seed: [u8; 32] = bytes.as_slice().try_into().map_err(|_| {
            CliError::Login(format!(
                "the device key {} is not 32 bytes (delete it to regenerate)",
                path.display()
            ))
        })?;
        return Ok(seed);
    }
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            // 0700 (not the umask default 0755): ~/.cerulion also holds the
            // 0600 secrets (auth.json / desk.key / device.cert). Creating it
            // here — the device-key path can run FIRST, before any
            // `atomic_write_secret` — with umask perms would leave the config
            // dir world-listable.
            auth::create_secret_dir(parent)
                .map_err(|e| CliError::Login(format!("creating {}: {e}", parent.display())))?;
        }
    }
    let mut seed = [0u8; 32];
    getrandom::fill(&mut seed)
        .map_err(|e| CliError::Login(format!("generating the device key failed: {e}")))?;
    write_new_device_key(path, &seed)?;
    Ok(seed)
}

/// Create `path` (0600 on Unix, `create_new` so a racing writer never
/// overwrites) and write the 32-byte seed.
#[cfg(unix)]
fn write_new_device_key(path: &Path, seed: &[u8; 32]) -> CliResult<()> {
    use std::io::Write as _;
    use std::os::unix::fs::OpenOptionsExt;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .mode(0o600)
        .open(path)
        .map_err(|e| CliError::Login(format!("creating the device key {}: {e}", path.display())))?;
    f.write_all(seed)
        .map_err(|e| CliError::Login(format!("writing the device key {}: {e}", path.display())))?;
    // fsync so a crash between the buffer and the flush cannot leave a
    // 0/partial-byte key that later fails `try_into::<[u8; 32]>()` (a
    // manual-deletion recovery). Mirrors `auth::write_new_secret_file`.
    f.sync_all()
        .map_err(|e| CliError::Login(format!("syncing the device key {}: {e}", path.display())))
}

#[cfg(not(unix))]
fn write_new_device_key(path: &Path, seed: &[u8; 32]) -> CliResult<()> {
    use std::io::Write as _;
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| CliError::Login(format!("creating the device key {}: {e}", path.display())))?;
    f.write_all(seed)
        .map_err(|e| CliError::Login(format!("writing the device key {}: {e}", path.display())))?;
    // fsync so a crash between the buffer and the flush cannot leave a
    // 0/partial-byte key that later fails `try_into::<[u8; 32]>()` (a
    // manual-deletion recovery). Mirrors `auth::write_new_secret_file`.
    f.sync_all()
        .map_err(|e| CliError::Login(format!("syncing the device key {}: {e}", path.display())))
}

// ===========================================================================
// HTTP helpers
// ===========================================================================

/// The ONLY place a reqwest client is built in this crate: the rustls provider
/// install must precede every `ClientConfig::builder()` (a second builder
/// anywhere else would be one that forgets it), so account_cmd and robot_cmd
/// build through here too.
pub(crate) fn http_client() -> CliResult<reqwest::blocking::Client> {
    // `install_default` refuses a second install and hands the provider back in
    // the Err — the already-installed case (an earlier client here, or a host
    // that set one), not a failure.
    let _ = rustls::crypto::ring::default_provider().install_default();
    reqwest::blocking::Client::builder()
        .timeout(HTTP_TIMEOUT)
        .user_agent(concat!("cerulion-cli/", env!("CARGO_PKG_VERSION")))
        .build()
        .map_err(|e| CliError::Login(format!("could not build the HTTP client: {e}")))
}

fn empty_body() -> serde_json::Value {
    serde_json::json!({})
}

/// POST a JSON body and deserialize a successful JSON response; non-2xx maps to a
/// [`CliError::Login`] carrying the server's error body.
fn post_json<T: for<'de> Deserialize<'de>>(
    client: &reqwest::blocking::Client,
    base: &str,
    path: &str,
    body: &serde_json::Value,
) -> CliResult<T> {
    let resp = client
        .post(format!("{base}{path}"))
        .json(body)
        .send()
        .map_err(|e| {
            CliError::Login(format!("the account service ({base}) is unreachable: {e}"))
        })?;
    if !resp.status().is_success() {
        let status = resp.status();
        let err: ErrorBody = resp.json().unwrap_or_default();
        return Err(CliError::Login(format!(
            "{path} refused ({status}): {}",
            if err.error_description.is_empty() {
                err.error
            } else {
                err.error_description
            }
        )));
    }
    resp.json::<T>()
        .map_err(|e| CliError::Login(format!("could not parse the {path} response: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cert that was REMOVED and could not be put back is a lost device
    /// binding, and the remedy differs: the user must sign in again to be
    /// re-certified rather than delete a file that is already gone. Telling them
    /// the previous sign-in is "left in place" there sends them nowhere.
    #[test]
    fn the_refusal_only_claims_the_previous_sign_in_survived_when_it_did() {
        let intact = stale_cert_error(
            &auth::ClearCertError::for_test("/home/x/.cerulion/device.cert (busy)", Vec::new()),
            &[],
            true,
        )
        .to_string();
        assert!(
            intact.contains("left in place, exactly as it was"),
            "a clear that changed nothing says so: {intact}"
        );

        let lost = ["/srv/certs/desk.cert (read-only)".to_string()];
        let msg = stale_cert_error(
            &auth::ClearCertError::for_test("/home/x/.cerulion/device.cert (busy)", lost.to_vec()),
            &lost,
            true,
        )
        .to_string();
        assert!(
            !msg.contains("left in place")
                && msg.contains("device binding is gone")
                && msg.contains("/srv/certs/desk.cert")
                && msg.contains("`cerulion login` again"),
            "a lost binding names the path and the only recovery there is: {msg}"
        );
    }

    /// The CAUSE clause is the user's diagnosis, and the two causes have
    /// different fixes: an issuer that certifies nothing needs the stale file
    /// deleted for good, while a certifying one that moved this machine to
    /// another account will re-certify it the moment the old cert is gone.
    /// Telling either user the other's story sends them to the wrong service.
    #[test]
    fn the_refusal_names_which_kind_of_login_the_stale_cert_blocked() {
        let err = || auth::ClearCertError::for_test("/home/x/.cerulion/device.cert", Vec::new());
        let identity_only = stale_cert_error(&err(), &[], true).to_string();
        let certifying = stale_cert_error(&err(), &[], false).to_string();
        assert!(
            identity_only.contains("issues no device certificate")
                && !identity_only.contains("certified for a different account"),
            "the identity-only refusal blames the service's missing surface: {identity_only}"
        );
        assert!(
            certifying.contains("certified for a different account")
                && !certifying.contains("issues no device certificate"),
            "a certifying issuer must not be described as issuing nothing: {certifying}"
        );
    }

    struct EnvGuard(&'static str, Option<String>);
    impl EnvGuard {
        fn set(key: &'static str, val: &str) -> Self {
            let prev = std::env::var(key).ok();
            std::env::set_var(key, val);
            EnvGuard(key, prev)
        }
        fn unset(key: &'static str) -> Self {
            let prev = std::env::var(key).ok();
            std::env::remove_var(key);
            EnvGuard(key, prev)
        }
    }
    impl Drop for EnvGuard {
        fn drop(&mut self) {
            match &self.1 {
                Some(v) => std::env::set_var(self.0, v),
                None => std::env::remove_var(self.0),
            }
        }
    }

    // Env-mutating tests share ONE process env across the whole lib test binary →
    // serialize on the crate-wide lock, not a per-file one (see `crate::test_env`).
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    #[test]
    fn account_service_base_defaults_to_production() {
        let _g = env_lock();
        let _e = EnvGuard::unset(ACCOUNT_SERVICE_ENV);
        assert_eq!(account_service_base(), DEFAULT_ACCOUNT_SERVICE);
    }

    #[test]
    fn account_service_base_env_override_wins_and_trims_slash() {
        let _g = env_lock();
        let _e = EnvGuard::set(ACCOUNT_SERVICE_ENV, "http://127.0.0.1:8787/");
        assert_eq!(account_service_base(), "http://127.0.0.1:8787");
    }

    #[test]
    fn empty_account_service_env_falls_back_to_default() {
        let _g = env_lock();
        let _e = EnvGuard::set(ACCOUNT_SERVICE_ENV, "   ");
        assert_eq!(account_service_base(), DEFAULT_ACCOUNT_SERVICE);
    }

    #[test]
    fn the_non_interactive_refusal_is_two_lines_naming_both_fixes() {
        // Hand oracle. The first line says what is wrong, the second says what to
        // do about it on BOTH sides (a person at a terminal, and automation), and
        // there are exactly two of them so the whole answer is readable in a CI
        // log that shows one line of stderr.
        let lines: Vec<&str> = NON_INTERACTIVE_REFUSAL.lines().collect();
        assert_eq!(lines.len(), 2, "{NON_INTERACTIVE_REFUSAL}");
        assert!(
            lines[0].contains("never signed in"),
            "line 1 must state the condition: {}",
            lines[0]
        );
        assert!(
            lines[1].contains("cerulion login"),
            "line 2 must name the interactive fix: {}",
            lines[1]
        );
        assert!(
            lines[1].contains("CERULION_HOME"),
            "line 2 must name the automation fix: {}",
            lines[1]
        );
        assert!(
            !NON_INTERACTIVE_REFUSAL.contains("CERULION_LOGIN_GATE"),
            "the refusal must not name a variable that no longer exists"
        );
    }

    #[test]
    fn only_the_exact_value_off_switches_the_gate_off() {
        // Hand oracle. One spelling, matched byte for byte: a variable somebody
        // set to `0`, `false` or `OFF` reads as a guess at a switch that is not
        // documented, and a guess must not work. `off ` and `off\n` are here
        // because a value written by a careless shell line is the likeliest way
        // to arrive at an almost-right value.
        let _g = env_lock();
        {
            let _e = EnvGuard::unset(LOGIN_GATE_ENV);
            assert!(!gate_switched_off(), "unset leaves the gate on");
        }
        {
            let _e = EnvGuard::set(LOGIN_GATE_ENV, "off");
            assert!(gate_switched_off(), "off is the one value that works");
        }
        for v in [
            "", "0", "1", "no", "yes", "on", "true", "false", "OFF", "Off", "oFf", " off", "off ",
            "off\n", "disabled", "none",
        ] {
            let _e = EnvGuard::set(LOGIN_GATE_ENV, v);
            assert!(!gate_switched_off(), "{v:?} must leave the gate on");
        }
    }

    #[test]
    fn the_switch_is_read_by_the_gate_itself_and_returns_before_any_dial() {
        // The wiring pin. Without it the predicate above could be correct and
        // unreachable. An empty CERULION_HOME has never signed in and the
        // account service is a reserved dead port, so a gate that did not
        // return on the switch would either refuse or fail on the connection.
        let _g = env_lock();
        let dir = tempfile::tempdir().unwrap();
        let _home = EnvGuard::set("CERULION_HOME", dir.path().to_str().unwrap());
        let _svc = EnvGuard::set(ACCOUNT_SERVICE_ENV, "http://127.0.0.1:1");
        let _e = EnvGuard::set(LOGIN_GATE_ENV, "off");
        let mut out = Vec::new();
        ensure_login_gate(&mut out).expect("a run with the switch set proceeds");
        assert!(
            out.is_empty(),
            "the switch prints nothing to the command's stream: {}",
            String::from_utf8_lossy(&out)
        );
    }

    #[test]
    fn never_signed_in_without_a_terminal_refuses_at_once_and_dials_nothing() {
        // The CI / script / service arm. An empty CERULION_HOME is never signed
        // in, and the account service is a reserved dead port that MUST NOT be
        // dialled: a run that started the device flow would either error on the
        // connection or sit in the poll loop, and either way it would not return
        // the refusal below.
        let _g = env_lock();
        let dir = tempfile::tempdir().unwrap();
        let _home = EnvGuard::set("CERULION_HOME", dir.path().to_str().unwrap());
        let _svc = EnvGuard::set(ACCOUNT_SERVICE_ENV, "http://127.0.0.1:1");
        let mut out = Vec::new();
        let started = std::time::Instant::now();
        let err = ensure_login_gate_with(&mut out, false)
            .expect_err("a never-signed-in machine with no terminal is refused");
        assert_eq!(err.to_string(), NON_INTERACTIVE_REFUSAL);
        assert!(
            out.is_empty(),
            "the refusal is the returned error, not a prompt: {}",
            String::from_utf8_lossy(&out)
        );
        // Generous, and still orders of magnitude under the poll loop's cap.
        assert!(
            started.elapsed() < std::time::Duration::from_secs(5),
            "the refusal must be immediate, not a poll"
        );
    }

    #[test]
    fn a_signed_in_machine_proceeds_with_zero_network_at_a_terminal_or_not() {
        // fires-once-then-zero-network: with a seeded auth.json the gate proceeds
        // WITHOUT dialing the service (dead endpoint MUST NOT be hit), and the
        // terminal question never arises: both values of `interactive` proceed.
        let _g = env_lock();
        let dir = tempfile::tempdir().unwrap();
        auth::seed_logged_in_at(dir.path(), "acct-xyz").unwrap();
        let _home = EnvGuard::set("CERULION_HOME", dir.path().to_str().unwrap());
        let _svc = EnvGuard::set(ACCOUNT_SERVICE_ENV, "http://127.0.0.1:1");
        for interactive in [false, true] {
            let mut out = Vec::new();
            ensure_login_gate_with(&mut out, interactive).expect("seeded ⇒ proceed, zero network");
            assert!(
                out.is_empty(),
                "no prompt when already signed in (interactive={interactive})"
            );
        }
    }

    #[test]
    fn device_key_seed_is_created_0600_then_reused() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("desk.key");
        let s1 = ensure_device_key_seed_at(&path).unwrap();
        assert!(path.exists());
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600);
        }
        // Second call REUSES the existing key (never regenerates).
        let s2 = ensure_device_key_seed_at(&path).unwrap();
        assert_eq!(
            s1, s2,
            "an existing device key is reused, never regenerated"
        );
    }

    #[cfg(unix)]
    #[test]
    fn device_key_path_creates_fresh_config_dir_0700() {
        // The device-key seed can be the FIRST thing to create ~/.cerulion (before
        // any auth.json / device.cert write), so it MUST create the parent at 0700
        // — not the umask default 0755 — since that dir also holds the 0600 secrets
        // whose filenames would otherwise be world-listable. Regression guard:
        // a plain create_dir_all here leaves it 0755.
        use std::os::unix::fs::PermissionsExt;
        let base = tempfile::tempdir().unwrap();
        // A NESTED, not-yet-existing parent so the dir-creation branch actually
        // runs (the sibling 0600 test writes into the pre-existing tempdir root).
        let cfg_dir = base.path().join("nested").join(".cerulion");
        let path = cfg_dir.join("desk.key");
        assert!(!cfg_dir.exists(), "precondition: parent must not pre-exist");
        ensure_device_key_seed_at(&path).unwrap();
        let mode = std::fs::metadata(&cfg_dir).unwrap().permissions().mode() & 0o777;
        assert_eq!(mode, 0o700, "~/.cerulion must be owner-only (0700)");
    }

    #[test]
    fn wrong_size_device_key_is_a_loud_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("desk.key");
        std::fs::write(&path, b"too short").unwrap();
        let err = ensure_device_key_seed_at(&path).unwrap_err();
        assert!(matches!(err, CliError::Login(_)));
        assert!(err.to_string().contains("32 bytes"));
    }

    #[test]
    fn refresh_is_noop_and_zero_network_when_never_logged_in() {
        // Never logged in ⇒ nothing to refresh — the dead endpoint MUST NOT be
        // dialed (a network call would hang/error on 127.0.0.1:1).
        let _g = env_lock();
        let dir = tempfile::tempdir().unwrap();
        let _home = EnvGuard::set("CERULION_HOME", dir.path().to_str().unwrap());
        let _svc = EnvGuard::set(ACCOUNT_SERVICE_ENV, "http://127.0.0.1:1");
        assert!(super::refresh_session_if_stale().unwrap().is_none());
    }

    #[test]
    fn refresh_is_noop_and_zero_network_when_session_valid() {
        // A seeded VALID session ⇒ no refresh, no cloud call (dead endpoint MUST
        // NOT be dialed).
        let _g = env_lock();
        let dir = tempfile::tempdir().unwrap();
        auth::seed_logged_in_at(dir.path(), "acct").unwrap();
        let _home = EnvGuard::set("CERULION_HOME", dir.path().to_str().unwrap());
        let _svc = EnvGuard::set(ACCOUNT_SERVICE_ENV, "http://127.0.0.1:1");
        assert!(super::refresh_session_if_stale().unwrap().is_none());
    }

    #[test]
    fn challenge_response_parses_and_decodes_the_bound_account() {
        // The challenge DTO (shared by the device + robot PoP paths) deserializes a
        // SERVER-shaped body and the account_id decodes back to the exact 32 bytes the
        // client signs the PoP over — a mismatch here would make the client sign the
        // wrong message and fail verify.
        let account_bytes = [0x33u8; 32];
        let account_b64 = URL_SAFE_NO_PAD.encode(account_bytes);
        let json = serde_json::json!({
            "challenge": "Y2hhbGxlbmdlLWJlYXJlcg",
            "account_id": account_b64,
            "expires_in": 300u64,
        })
        .to_string();
        let ch: ChallengeResponse = serde_json::from_str(&json).unwrap();
        assert_eq!(ch.challenge, "Y2hhbGxlbmdlLWJlYXJlcg");
        let decoded: [u8; 32] = URL_SAFE_NO_PAD
            .decode(&ch.account_id)
            .unwrap()
            .try_into()
            .unwrap();
        assert_eq!(decoded, account_bytes);

        // A signature built from the decoded account round-trips through verify —
        // proving the client half signs exactly what the issuer half rebuilds (the
        // device-registration purpose here; the robot purpose is the same shape).
        let sk = ed25519_dalek::SigningKey::from_bytes(&[9u8; 32]);
        let pk = sk.verifying_key().to_bytes();
        let sig = sign_pop(
            &sk,
            PURPOSE_DEVICE_REGISTRATION,
            &decoded,
            &pk,
            &ch.challenge,
        );
        assert_eq!(
            cerulion_pairing::pop::verify_pop(
                &cerulion_pairing::format::PublicKey(pk),
                PURPOSE_DEVICE_REGISTRATION,
                &decoded,
                &ch.challenge,
                &sig,
            ),
            cerulion_pairing::pop::PopVerification::Ok
        );
    }

    /// Two pins cover the TLS-less reqwest build (the production default
    /// account service is HTTPS, so `http_client` MUST carry a TLS backend):
    /// building the client at all, and the manifest declaration. A runtime
    /// request probe cannot discriminate here — `is_connect()` is true for
    /// both a refused CONNECT (TLS present) and hyper-util's https-scheme
    /// refusal (TLS absent).
    ///
    /// Pin one — behavioural: the lib-test build dev-depends on
    /// `cerulion_accountd`, whose reqwest `rustls` feature unifies aws-lc-rs
    /// alongside our ring, so the build below panics inside
    /// `ClientConfig::builder()` ("Could not automatically determine the
    /// process-level CryptoProvider") the moment `http_client`'s provider
    /// install is removed.
    ///
    /// Pin two — structural: the lib tests can never be TLS-less (that same
    /// unification always supplies a backend), so only the manifest assertion
    /// catches a TLS-less `cargo build -p cerulion_cli`.
    #[test]
    fn reqwest_declares_a_tls_backend_and_the_client_builds_under_unified_providers() {
        let _client = http_client().expect(
            "the client builds under unified aws-lc-rs + ring providers — \
             without `http_client`'s ring install this panics in \
             ClientConfig::builder()",
        );

        let manifest = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/Cargo.toml"))
            .expect("the crate manifest reads");
        let reqwest_decl = manifest
            .split("reqwest = ")
            .nth(1)
            .and_then(|rest| rest.split(']').next())
            .expect("the reqwest dependency declaration");
        assert!(
            reqwest_decl.contains("rustls"),
            "cerulion_cli_engine's own reqwest declaration must enable a TLS \
             backend — the HTTPS default cannot rely on another crate's \
             feature unification: {reqwest_decl}"
        );
    }

    // Structural pin for the same finding: a second `Client::builder()`
    // anywhere in the crate skips the provider install in `http_client` and
    // panics in two-provider builds (aws-lc-rs + ring unified). The behavioural
    // pin is `reqwest_declares_a_tls_backend_and_the_client_builds_under_unified_providers`
    // above: the lib-test build unifies both providers, so it panics the
    // moment the install is dropped.
    #[test]
    fn every_reqwest_client_in_the_crate_is_built_through_http_client() {
        fn rs_files(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for entry in std::fs::read_dir(dir).unwrap() {
                let path = entry.unwrap().path();
                if path.is_dir() {
                    rs_files(&path, out);
                } else if path.extension().is_some_and(|e| e == "rs") {
                    out.push(path);
                }
            }
        }
        let mut files = Vec::new();
        rs_files(
            std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src")),
            &mut files,
        );
        let mut builders = Vec::new();
        for file in &files {
            let text = std::fs::read_to_string(file).unwrap();
            if text.contains("reqwest::blocking::Client::builder()") {
                builders.push(file.file_name().unwrap().to_string_lossy().into_owned());
            }
        }
        assert_eq!(
            builders,
            ["login_cmd.rs"],
            "a second reqwest::blocking::Client::builder() skips the ring \
             provider install in login_cmd::http_client and panics \
             ClientConfig::builder() in two-provider builds — build through \
             http_client instead; found builders in: {builders:?}"
        );
    }
}
