// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded enrollment of an existing login key, without an interactive sign-in.

use std::sync::{mpsc, Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use cerulion_core::transport::failure_regime_latch::{FailureRegimeLatch, RegimeDecision};
use cerulion_pairing::format::PublicKey;

use super::*;

struct EnrollmentRegime {
    attempted: bool,
    result: Option<Result<(), String>>,
    failures: FailureRegimeLatch,
}

impl Default for EnrollmentRegime {
    fn default() -> Self {
        Self {
            attempted: false,
            result: None,
            failures: FailureRegimeLatch::new(),
        }
    }
}

/// Enroll a legacy sign-in's existing key at most once per process. Work runs on
/// a background thread; the caller waits only its budget, and later calls back
/// off for this process. Never refreshes tokens, prompts or creates a new key.
pub fn enroll_device_once_within(budget: Duration) -> CliResult<bool> {
    static REGIME: OnceLock<Arc<Mutex<EnrollmentRegime>>> = OnceLock::new();
    enroll_with_regime(
        budget,
        REGIME.get_or_init(|| Arc::new(Mutex::new(EnrollmentRegime::default()))),
    )
}

#[cfg(test)]
mod tests;

fn enroll_with_regime(budget: Duration, regime: &Arc<Mutex<EnrollmentRegime>>) -> CliResult<bool> {
    if crate::graph_cmd::remote_network_suppressed() || complete_certificate() {
        return Ok(false);
    }
    if !auth::load()
        .state()
        .is_some_and(|state| state.logged_in_ever)
    {
        return Ok(false);
    }
    {
        let mut state = regime
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if state.attempted {
            if matches!(&state.result, Some(Ok(()))) {
                return Ok(false);
            }
            let _ = state.failures.on_failure();
            return Err(CliError::Login(DEVICE_ENROLLMENT_REQUIRED.into()));
        }
        state.attempted = true;
    }
    let deadline = Instant::now() + budget;
    // Resolve identity and issuer on the caller, before background work can
    // outlive a configuration change. The publication fence rechecks identity.
    let auth_path =
        auth::auth_json_path().ok_or_else(|| CliError::Login(DEVICE_ENROLLMENT_REQUIRED.into()))?;
    let base = account_service_base();
    let configuration_error = auth::check_netd_cert_paths().err();
    let mut paths = auth::absent_device_cert_consumer_paths();
    let leaf = auth_path.with_file_name("device.cert");
    if !paths.contains(&leaf) {
        paths.push(leaf);
    }
    let request_budget = budget / 4;
    let worker_regime = Arc::clone(regime);
    let dispatch = tracing::dispatcher::get_default(Clone::clone);
    let span = tracing::Span::current();
    let (tx, rx) = mpsc::sync_channel(1);
    let worker = std::thread::Builder::new().name("cer-device-enrollment".into()).spawn(move || {
        let _dispatch = tracing::dispatcher::set_default(&dispatch);
        let _span = span.enter();
        let result = configuration_error.map_or_else(
            || enroll_existing_key(deadline, request_budget, &auth_path, &base, &paths),
            Err,
        );
        let success = result.is_ok();
        let mut state = worker_regime.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Err(error) = &result {
            match state.failures.on_failure() {
                RegimeDecision::Loud => tracing::debug!(error, "automatic device enrollment unavailable; sign in again to enroll this machine"),
                RegimeDecision::Suppressed { suppressed } | RegimeDecision::StillFailing { suppressed, .. } => tracing::debug!(suppressed, "automatic device enrollment remains unavailable"),
            }
        } else {
            let _ = state.failures.on_success();
            tracing::info!("this machine was enrolled as a device on your account");
        }
        state.result = Some(result);
        let _ = tx.send(success);
    });
    if worker.is_err() {
        let mut state = regime
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        state.result = Some(Err("could not start the enrollment worker".into()));
        let _ = state.failures.on_failure();
    }
    match rx.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
        Ok(true) => Ok(true),
        _ => Err(CliError::Login(DEVICE_ENROLLMENT_REQUIRED.into())),
    }
}

fn complete_certificate() -> bool {
    auth::device_cert_path().is_some_and(|path| path.is_file())
        && auth::auth_json_path()
            .is_some_and(|path| crate::owner_certificate::cache_path(&path).is_file())
}

fn remaining(deadline: Instant) -> std::io::Result<Duration> {
    let remaining = deadline.saturating_duration_since(Instant::now());
    if remaining.is_zero() {
        Err(std::io::Error::new(
            std::io::ErrorKind::TimedOut,
            "device enrollment exceeded its time budget",
        ))
    } else {
        Ok(remaining)
    }
}

fn enroll_existing_key(
    deadline: Instant,
    request_budget: Duration,
    auth_path: &std::path::Path,
    base: &str,
    paths: &[std::path::PathBuf],
) -> Result<(), String> {
    let layout = cerulion_pairing::identity::IdentityLayout::at(
        auth_path
            .parent()
            .ok_or_else(|| "no machine key directory".to_string())?,
    );
    let snapshot = auth::with_store_lock_within(
        auth_path,
        remaining(deadline).map_err(|e| e.to_string())?,
        || {
            let state = auth::load_from(auth_path)
                .state()
                .filter(|state| state.logged_in_ever && state.session_is_valid(auth::now_unix_ns()))
                .cloned()
                .ok_or_else(|| {
                    std::io::Error::other("a still-valid sign-in is required for device enrollment")
                })?;
            let key = layout
                .load_machine_key()
                .map_err(std::io::Error::other)?
                .ok_or_else(|| std::io::Error::other("the existing device key is missing"))?;
            Ok((state, *key.seed()))
        },
    )
    .map_err(|e| e.to_string())?;
    let result = (|| -> CliResult<()> {
        let url = reqwest::Url::parse(base)
            .map_err(|_| CliError::Login("invalid account service URL".into()))?;
        let loopback = url.host_str().is_some_and(|host| {
            host == "localhost"
                || host
                    .trim_matches(['[', ']'])
                    .parse::<std::net::IpAddr>()
                    .is_ok_and(|ip| ip.is_loopback())
        });
        if url.scheme() != "https" && !(url.scheme() == "http" && loopback) {
            return Err(CliError::Login(
                "device enrollment requires HTTPS or a loopback account service".into(),
            ));
        }
        let client =
            build_http_client_with_timeout(reqwest::redirect::Policy::none(), request_budget)?;
        let issued =
            register_device_with_seed(&client, base, &snapshot.0.session_token, snapshot.1)?
                .ok_or_else(|| {
                    CliError::Login("the account service issues no device certificates".into())
                })?;
        let profile = fetch_account_profile(&client, base, &snapshot.0.session_token)?;
        if verified_session_account(profile, &issued)? != snapshot.0.account_id {
            return Err(CliError::Login(
                "device enrollment names a different sign-in account".into(),
            ));
        }
        let account = crate::account_identity::pairing_account_id(&snapshot.0.account_id)?;
        let verified = crate::owner_certificate::verify_issued(
            &issued.response.device_cert,
            &issued.response.intermediate,
            account,
            PublicKey(issued.device_key),
        )?;
        let now = auth::now_unix_ns();
        for validity in [
            verified.device_cert.cert.validity,
            verified.intermediate.cert.validity,
        ] {
            if now < validity.not_before_ns || now >= validity.not_after_ns {
                return Err(CliError::Login(
                    "the issued device certificate is not currently valid".into(),
                ));
            }
        }
        auth::with_store_lock_within(auth_path, remaining(deadline)?, || {
            let current = auth::load_from(auth_path);
            if current.state() != Some(&snapshot.0)
                || !snapshot.0.session_is_valid(auth::now_unix_ns())
            {
                return Err(std::io::Error::other(
                    "the sign-in changed during device enrollment",
                ));
            }
            let key = layout.load_machine_key().map_err(std::io::Error::other)?;
            if key.as_ref().map(|key| key.public_key()) != Some(issued.device_key) {
                return Err(std::io::Error::other(
                    "the device key changed during enrollment",
                ));
            }
            // Publish the verified response even if another enrollment filled
            // these paths meanwhile; the account and key fence still holds.
            let mut staged = paths
                .iter()
                .map(|path| auth::stage_device_cert_at(path, &issued.response.device_cert))
                .collect::<std::io::Result<Vec<_>>>()?;
            staged.push(crate::owner_certificate::stage_at(
                &crate::owner_certificate::cache_path(auth_path),
                &issued.response.device_cert,
                &issued.response.intermediate,
            )?);
            for pending in staged {
                remaining(deadline)?;
                pending.commit()?;
            }
            Ok(())
        })?;
        Ok(())
    })();
    result.map_err(|error| {
        let mut message = error.to_string();
        for token in [&snapshot.0.session_token, &snapshot.0.refresh_token] {
            if !token.is_empty() {
                message = message.replace(token, "[redacted]");
            }
        }
        message
    })
}
