// SPDX-License-Identifier: AGPL-3.0-only
//! A2/A3 — the CLI half of login-gated ownership at install: register THIS
//! machine as a robot owned by the logged-in account, proving possession of its
//! transport key.
//!
//! A standalone `cerulion` CLI install is a robot (the rule is "any computer with
//! the standalone CLI is a robot"). After the install-time login, this machine
//! registers itself as a robot with the account service — `POST /v1/robots`
//! binds the logged-in account as the owner and returns an OWNER_FULL owner
//! `SignedGrant` for it. The robot verifies that grant OFFLINE (via the shipped
//! `cerulion_pairing::verify::TrustStore::claim_by_owner_grant` seam) to write its
//! owner access row — zero robot↔cloud contact beyond the one-time install login
//! (invariant I3).
//!
//! **A3 — proof of possession.** Registration is no longer a bare authenticated
//! POST: the caller first fetches a single-use challenge
//! (`POST /v1/devices/challenge`) and signs it with the private half of its device
//! key ([`cerulion_pairing::pop::sign_pop`]), so the service can prove the caller
//! actually controls the key it is registering (closing the squatting hole). The
//! signing key is the SAME `~/.cerulion/desk.key` device seed — one machine, one
//! key — so the proof is free (no extra key material).
//!
//! **Ownership is install-automatic — there is NO user-facing `cerulion robot`
//! verb** (by design). [`register_robot`] is the internal **install-funnel
//! entry** — the cloud call the install performs on a robot, never a command a user
//! types. It is deliberately NOT auto-wired into the login flow
//! ([`crate::login_cmd::run_login`]): a plain login is a DESK (Studio ⊃ CLI — a
//! Studio machine shares the account and owns no robot), so registering a robot on
//! every login would contradict "a Studio machine is a desk (no robot row)". The
//! trigger that distinguishes a robot install from a desk install (an install-time
//! `role`) is an install-funnel concern; this function stays the tested, ready cloud
//! call the funnel invokes on a robot. Takeover / recovery flows live in Studio / the
//! web account page, NOT the CLI. Its live proof is `robot_registration_e2e_test.rs`
//! (a real `cerulion_accountd` round-trip) — it is tested, NOT inert.

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use cerulion_pairing::pop::{sign_pop, PURPOSE_ROBOT_REGISTRATION};
use serde::Deserialize;

use crate::error::{CliError, CliResult};
use crate::login_cmd::{self, account_service_base, http_client};

/// The result of `POST /v1/robots`: the minted robot id + the offline-verifiable
/// owner credentials, base64url (no-padding) encoded exactly as the account
/// service returns them (the robot decodes these into a
/// `cerulion_pairing::verify::PairingPresentation` alongside its cached device
/// cert to claim ownership offline).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RobotOwnership {
    /// `base64url(RobotId)` — the id the account service minted for this robot.
    pub robot_id: String,
    /// `base64url(postcard(SignedGrant))` — the OWNER_FULL owner grant for the
    /// logged-in account.
    pub owner_grant: String,
    /// `base64url(postcard(SignedIntermediateCert))` — the chain link presented
    /// alongside the grant + device cert during the offline owner claim.
    pub intermediate: String,
    /// `base64url(AccountId)` — the account now bound as owner.
    pub account_id: String,
}

/// The `POST /v1/robots` response DTO.
#[derive(Deserialize)]
struct RegisterRobotResponse {
    robot_id: String,
    owner_grant: String,
    intermediate: String,
    account_id: String,
}

/// The RFC-8628-style error body (`{ "error": ..., "error_description": ... }`).
#[derive(Deserialize, Default)]
struct ErrorBody {
    #[serde(default)]
    error: String,
    #[serde(default)]
    error_description: String,
}

/// Register THIS machine as a robot owned by the logged-in account (`POST
/// /v1/robots`, session-authed). `hostname` is the robot's stamped hostname — its
/// runtime identity (the hostname is the robot identity); the caller resolves it
/// (the OS hostname on a robot install). The machine's own device key
/// (`~/.cerulion/desk.key`, shared with `cerulion connect`/`pair`) is presented as
/// the robot's transport public key, so the returned owner grant + a device cert
/// for the SAME key form the offline owner-claim presentation.
///
/// A `hostname` that is empty/whitespace is a loud error (it is the robot
/// identity). A session that is missing/expired is refused server-side; the
/// error names `cerulion login`.
pub fn register_robot(session_token: &str, hostname: &str) -> CliResult<RobotOwnership> {
    if hostname.trim().is_empty() {
        return Err(CliError::Login(
            "a robot hostname is required (it is the robot's identity)".into(),
        ));
    }
    let seed = login_cmd::ensure_device_key_seed()?;
    let signing = ed25519_dalek::SigningKey::from_bytes(&seed);
    let transport_key = signing.verifying_key().to_bytes();
    let base = account_service_base();
    let client = http_client()?;

    // A3 proof-of-possession: fetch a fresh single-use challenge and sign it with the
    // transport key's PRIVATE half, so the service can prove we control the key we
    // register. The challenge carries the account it is bound to, so the signed
    // message matches exactly what the service rebuilds. The challenge fetch is shared
    // with the device-registration PoP path ([`login_cmd::fetch_pop_challenge`]).
    // A service with no `/v1/devices/challenge` cannot register a robot at all
    // (the hosted issuer is identity-only), so say that rather than let a 404
    // surface as a challenge-fetch fault.
    let Some((pop_challenge, pop_account)) =
        login_cmd::fetch_pop_challenge(&client, &base, session_token)?
    else {
        return Err(CliError::Login(format!(
            "{base} does not register robots (it issues no device certificates) — \
             point {} at a service that does",
            login_cmd::ACCOUNT_SERVICE_ENV
        )));
    };
    let pop_signature = sign_pop(
        &signing,
        PURPOSE_ROBOT_REGISTRATION,
        &pop_account,
        &transport_key,
        &pop_challenge,
    );

    let body = serde_json::json!({
        "hostname": hostname.trim(),
        "robot_transport_key": URL_SAFE_NO_PAD.encode(transport_key),
        "pop_challenge": pop_challenge,
        "pop_signature": URL_SAFE_NO_PAD.encode(pop_signature.0),
    });
    let resp = client
        .post(format!("{base}/v1/robots"))
        .bearer_auth(session_token)
        .json(&body)
        .send()
        .map_err(|e| CliError::Login(format!("registering the robot ({base}) failed: {e}")))?;
    if !resp.status().is_success() {
        let status = resp.status();
        let err: ErrorBody = resp.json().unwrap_or_default();
        let detail = if err.error_description.is_empty() {
            err.error
        } else {
            err.error_description
        };
        // A 401 means the session is stale — name the fix (re-login).
        if status.as_u16() == 401 {
            return Err(CliError::Login(format!(
                "the account service refused the session ({status}): {detail} — run `cerulion login`"
            )));
        }
        return Err(CliError::Login(format!(
            "robot registration refused ({status}): {detail}"
        )));
    }
    let reg: RegisterRobotResponse = resp
        .json()
        .map_err(|e| CliError::Login(format!("could not parse the /v1/robots response: {e}")))?;
    Ok(RobotOwnership {
        robot_id: reg.robot_id,
        owner_grant: reg.owner_grant,
        intermediate: reg.intermediate,
        account_id: reg.account_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mirror the sibling `login_cmd` env-test pattern: an RAII guard restores the
    // prior env value on drop (panic-safe — the old manual restore leaked the var
    // if an assert panicked first), and the CRATE-WIDE env lock (`crate::test_env`,
    // see `env_lock` below) serializes the env-mutating tests in this module against
    // every other env-mutating lib test in the crate — not merely against each other.
    struct EnvGuard(&'static str, Option<String>);
    impl EnvGuard {
        fn set(key: &'static str, val: &str) -> Self {
            let prev = std::env::var(key).ok();
            std::env::set_var(key, val);
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

    // Serializes against EVERY env-mutating lib test in the crate (one process, one
    // environment) — see `crate::test_env`.
    fn env_lock() -> std::sync::MutexGuard<'static, ()> {
        crate::test_env::env_lock()
    }

    #[test]
    fn empty_hostname_is_a_loud_error_before_any_network() {
        // A blank hostname is refused up front (it is the robot identity) — no
        // device key is read, no network call is made. Point the service at a dead
        // port to prove the early return fires before any dial.
        let _g = env_lock();
        let _svc = EnvGuard::set("CERULION_ACCOUNT_SERVICE", "http://127.0.0.1:1");
        let err = register_robot("sess", "   ").unwrap_err();
        assert!(matches!(err, CliError::Login(_)));
        assert!(
            err.to_string().contains("hostname"),
            "names the field: {err}"
        );
    }

    #[test]
    fn register_robot_response_parses_and_carries_the_offline_verifiable_fields() {
        // Real behavior (not a struct-literal echo): the client's response DTO
        // deserializes a SERVER-shaped JSON body and each base64url field routes to
        // its RobotOwnership slot, base64url-decodable into the bytes the robot
        // hands to the offline verifier. The robot_id/account_id round-trip their
        // exact 32 bytes against independently-computed oracles, and DISTINCT values
        // make a field swap (robot_id↔account_id, or grant↔intermediate) fail.
        let robot_id_bytes = [0x11u8; 32];
        let account_id_bytes = [0x22u8; 32];
        let robot_id_b64 = URL_SAFE_NO_PAD.encode(robot_id_bytes);
        let account_id_b64 = URL_SAFE_NO_PAD.encode(account_id_bytes);
        let json = serde_json::json!({
            "robot_id": robot_id_b64,
            "owner_grant": "b3duZXItZ3JhbnQ",
            "intermediate": "aW50ZXItY2VydA",
            "account_id": account_id_b64,
        })
        .to_string();

        // The client DTO accepts the server's exact field names (a serde rename or a
        // missing field would fail this parse).
        let reg: RegisterRobotResponse = serde_json::from_str(&json).unwrap();
        let ownership = RobotOwnership {
            robot_id: reg.robot_id,
            owner_grant: reg.owner_grant,
            intermediate: reg.intermediate,
            account_id: reg.account_id,
        };

        // robot_id and account_id decode back to their exact 32-byte oracles.
        assert_eq!(
            URL_SAFE_NO_PAD.decode(&ownership.robot_id).unwrap(),
            robot_id_bytes
        );
        assert_eq!(
            URL_SAFE_NO_PAD.decode(&ownership.account_id).unwrap(),
            account_id_bytes
        );
        // The distinct grant/intermediate blobs routed to their own slots.
        assert_eq!(ownership.owner_grant, "b3duZXItZ3JhbnQ");
        assert_eq!(ownership.intermediate, "aW50ZXItY2VydA");
        // Anti-swap: the two 32-byte-id slots carry distinct values, so a transposed
        // mapping would fail the decode asserts above.
        assert_ne!(ownership.robot_id, ownership.account_id);
    }
}
