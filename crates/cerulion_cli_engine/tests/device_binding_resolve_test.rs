// SPDX-License-Identifier: AGPL-3.0-only
//! A3 — coverage for the A4 production entry point
//! [`cerulion_cli_engine::device_binding::resolve_device_binding`].
//!
//! The module's in-crate unit tests exercise the pure core (`verify_device_binding`);
//! the PUBLIC `resolve_device_binding` — the function the desk grant-presenter (A4)
//! actually wires — reads `CERULION_HOME` + the cached files and cross-checks the
//! cert account against `auth.json`. Those production-facing arms had ZERO coverage
//! before these tests: the missing-cert → `cerulion login` remediation, the
//! read-error arm, and the account-divergence WARN (loud, cert still returned) were
//! all untested, so a regression to the loud-inference / actionable-error would only
//! surface once A4 wired it. These tests drive it over a temp `CERULION_HOME`. Each is
//! `#[serial]` because `CERULION_HOME` is process-global; the divergence arms are
//! `#[traced_test]` (the crate's `no-env-filter` feature captures the module's warn).

use std::path::Path;

use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use base64::Engine;
use ed25519_dalek::SigningKey;
use serial_test::serial;
use tracing_test::traced_test;

use cerulion_cli_engine::auth::{self, AuthState};
use cerulion_cli_engine::device_binding::resolve_device_binding;
use cerulion_cli_engine::error::CliError;
use cerulion_pairing::format::{
    AccountId, DeviceCert, PrincipalKind, PublicKey, Scope, Validity, FORMAT_VERSION,
};

/// The substring of the loud divergence warn `resolve_device_binding` emits when the
/// cert's account differs from the cached session — the exact contract text.
const DIVERGENCE_WARN: &str = "binds a DIFFERENT account";

/// RAII guard: set `CERULION_HOME` to `dir` and restore the prior value on drop
/// (panic-safe — a failed assert never leaks the override into a sibling test).
struct HomeGuard(Option<String>);
impl HomeGuard {
    fn set(dir: &Path) -> Self {
        let prev = std::env::var("CERULION_HOME").ok();
        std::env::set_var("CERULION_HOME", dir);
        HomeGuard(prev)
    }
}
impl Drop for HomeGuard {
    fn drop(&mut self) {
        match &self.0 {
            Some(v) => std::env::set_var("CERULION_HOME", v),
            None => std::env::remove_var("CERULION_HOME"),
        }
    }
}

/// The same, for any variable: the `CERULION_NETD_*` overrides relocate the cert
/// consumers a resolve fills in, and they are process-global too.
struct EnvGuard(&'static str, Option<String>);
impl EnvGuard {
    fn set(key: &'static str, value: &Path) -> Self {
        let prev = std::env::var(key).ok();
        std::env::set_var(key, value);
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

/// Build a `base64url(postcard(SignedDeviceCert))` blob binding `device_key` →
/// `account`, signed by a throwaway issuer (the desk-side resolver reads only the
/// bound key + account, never the signature). Mirrors the in-crate test helper.
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

/// Write a KNOWN 32-byte device seed to `<home>/desk.key` (so the resolver's
/// `ensure_device_key_seed` reads exactly this machine key) and return the ed25519
/// public key it derives — letting a cert be built that attests THIS machine.
fn seed_and_key(home: &Path, seed: [u8; 32]) -> [u8; 32] {
    std::fs::write(home.join("desk.key"), seed).unwrap();
    SigningKey::from_bytes(&seed).verifying_key().to_bytes()
}

/// Seed `<home>/auth.json` with a logged-in session bound to `account_b64` (the shape
/// `resolve_device_binding`'s cross-check reads). Written via the real `AuthState` +
/// `auth::write_to` — no `cfg(test)`-gated seed seam (invisible to an integration
/// binary). The token/expiry are placeholders; only `account_id` is under test.
fn seed_session_account(home: &Path, account_b64: &str) {
    let state = AuthState {
        account_id: account_b64.to_string(),
        session_token: "test-session".to_string(),
        refresh_token: "test-refresh".to_string(),
        expires_at_ns: u64::MAX,
        logged_in_ever: true,
        // The device-binding cross-check is role-agnostic; leave it unmarked.
        role: None,
    };
    auth::write_to(&home.join("auth.json"), &state).unwrap();
}

#[test]
#[serial]
fn resolve_with_no_cached_cert_is_a_loud_login_error() {
    // No device.cert cached → a LOUD CliError::Login naming the fix (`cerulion login`),
    // never a silent fallback to a self-derived account.
    let home = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    // (desk.key is absent too; the resolver creates it, then fails on the missing cert.)

    let err = resolve_device_binding().expect_err("a missing device cert must be a loud error");
    assert!(matches!(err, CliError::Login(_)), "got {err:?}");
    let msg = err.to_string();
    assert!(msg.contains("cerulion login"), "names the fix: {msg}");
    assert!(msg.contains("device cert"), "names what is missing: {msg}");
}

#[test]
#[serial]
fn resolve_with_an_unreadable_cert_is_a_loud_login_error() {
    // A cert path that exists but is unreadable (a directory, not a file) → the
    // read-error arm: a loud CliError::Login naming the read failure, never a panic
    // and never a silent empty binding.
    let home = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    std::fs::create_dir(home.path().join("device.cert")).unwrap();

    let err = resolve_device_binding().expect_err("an unreadable cert must be a loud error");
    assert!(matches!(err, CliError::Login(_)), "got {err:?}");
    assert!(
        err.to_string().contains("reading the device cert"),
        "names the read failure: {err}"
    );
}

#[test]
#[serial]
#[traced_test]
fn resolve_with_a_divergent_session_account_warns_but_returns_the_cert() {
    // The cached cert binds account A; auth.json records a DIFFERENT account B (a stale
    // session after an account switch). The cert (cryptographic truth) is STILL
    // returned — with account A — but a LOUD divergence warn fires.
    let home = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    let cert_account = [0x42u8; 32];
    std::fs::write(
        home.path().join("device.cert"),
        make_cert_b64(device_key, cert_account),
    )
    .unwrap();

    // Seed auth.json with a DIFFERENT account (base64url — the shape auth.json stores).
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode([0x99u8; 32]));

    let binding = resolve_device_binding().expect("a valid cert resolves despite divergence");
    // The CERT's account is returned — NOT the diverging session's.
    assert_eq!(binding.account, AccountId(cert_account));
    assert_eq!(binding.device_key, PublicKey(device_key));
    assert_eq!(binding.account_b64(), URL_SAFE_NO_PAD.encode(cert_account));

    // The divergence was surfaced LOUDLY, not silently swallowed.
    assert!(
        logs_contain(DIVERGENCE_WARN),
        "the stale-cert account-divergence warn must fire"
    );
}

#[test]
#[serial]
#[traced_test]
fn resolve_with_a_matching_session_account_does_not_warn() {
    // Anti-tautology control: when the cert account and the auth.json account AGREE,
    // NO divergence warn fires (the warn is tied to the divergence, not merely to the
    // cross-check running). Without this control the divergence test could pass on a
    // warn that ALWAYS fires.
    let home = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    let account = [0x42u8; 32];
    std::fs::write(
        home.path().join("device.cert"),
        make_cert_b64(device_key, account),
    )
    .unwrap();
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode(account));

    let binding = resolve_device_binding().expect("a matching-account cert resolves");
    assert_eq!(binding.account, AccountId(account));
    assert!(
        !logs_contain(DIVERGENCE_WARN),
        "a matching session account must NOT warn"
    );
}

/// A login publishes the cert one consumer at a time, so a kill between the
/// renames leaves netd's path empty while the CLI's holds the cert: a desk that
/// is signed in and, to netd, has no device binding. The read that RESOLVES the
/// binding fixes it — the cert it just verified is cached at every consumer that
/// has none. Nothing about the CLI's own path is missing here, which is exactly
/// the case a recovery keyed on "the cert I read is absent" never reaches.
#[test]
#[serial]
fn a_consumer_with_no_cert_is_given_the_one_the_reader_verified() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let netd_cert = elsewhere.path().join("desk.cert");
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    let account = [0x42u8; 32];
    let cert_b64 = make_cert_b64(device_key, account);
    std::fs::write(home.path().join("device.cert"), &cert_b64).unwrap();
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode(account));

    resolve_device_binding().expect("the cert the CLI holds resolves");

    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        cert_b64,
        "the consumer the interrupted login never reached now holds the SAME cert \
         the reader verified, so netd resolves the binding the CLI just did"
    );
}

/// The mirror image of the case above: the CLI's OWN path is the one the killed
/// login never reached, and the binding is sitting at netd's relocated cache
/// alone. Reporting "no device cert is cached" there would be false — the cert is
/// on this desk, for this account, attesting this machine — so the relocated copy
/// is adopted, and the propagation then converges the CLI's own path onto it.
#[test]
#[serial]
fn a_binding_only_the_relocated_cache_holds_is_still_resolved() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let netd_cert = elsewhere.path().join("desk.cert");
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    let account = [0x42u8; 32];
    let cert_b64 = make_cert_b64(device_key, account);
    std::fs::write(&netd_cert, &cert_b64).unwrap();
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode(account));

    let binding = resolve_device_binding().expect("the desk HAS a cert, at netd's path");
    assert_eq!(
        binding.account_b64(),
        URL_SAFE_NO_PAD.encode(account),
        "and it resolves the account that cert binds"
    );
    assert_eq!(
        std::fs::read_to_string(home.path().join("device.cert")).unwrap(),
        cert_b64,
        "and the CLI's own path is filled from it, so the repair converges instead \
         of being redone by every command"
    );
}

/// The propagation re-checks the account UNDER the store lock, so a `cerulion
/// login` that switches accounts AFTER the resolve read the store and BEFORE the
/// propagation gets the lock wins — otherwise the repair refills the very paths
/// that login just cleared, with the cert of the account the desk signed out of.
///
/// The interleaving is real, not asserted: this test takes the store lock the
/// helper needs, starts the propagation (which blocks on it), performs the switch
/// while it is blocked, and only then releases. An implementation that trusted the
/// account it was handed would write; one that re-reads under the lock cannot.
#[cfg(unix)]
#[test]
#[serial]
fn a_switch_that_lands_first_stops_the_propagation_it_raced() {
    use std::os::unix::io::AsRawFd;

    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let netd_cert = elsewhere.path().join("desk.cert");
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    let signed_in = URL_SAFE_NO_PAD.encode([0x42u8; 32]);
    let switched_to = URL_SAFE_NO_PAD.encode([0x99u8; 32]);
    let cert_b64 = make_cert_b64(device_key, [0x42u8; 32]);
    // The bytes the switching login publishes, captured through the real writer
    // BEFORE the lock is held (the writer takes the lock itself, and the switch
    // has to land while the propagation is blocked).
    let store = home.path().join("auth.json");
    seed_session_account(home.path(), &switched_to);
    let switched_store = std::fs::read(&store).unwrap();
    // The store the resolve observed: it named the account the cert binds, which
    // is what made the propagation legitimate at that instant.
    seed_session_account(home.path(), &signed_in);

    // Hold the store lock, so the propagation cannot proceed past its own
    // acquisition until the switch below has run.
    let held = std::fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(home.path().join(auth::STORE_LOCK_FILE))
        .unwrap();
    assert_eq!(
        unsafe { libc::flock(held.as_raw_fd(), libc::LOCK_EX) },
        0,
        "the test must hold the lock the propagation waits on"
    );

    let cert_for_thread = cert_b64.clone();
    let account_for_thread = signed_in.clone();
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    let worker = std::thread::spawn(move || {
        auth::cache_verified_device_cert_at_absent_consumers(&cert_for_thread, &account_for_thread);
        let _ = done_tx.send(());
    });
    // It must be BLOCKED, not finished: the seam this test is about only exists
    // while the lock is held elsewhere.
    assert!(
        done_rx
            .recv_timeout(std::time::Duration::from_millis(300))
            .is_err(),
        "the propagation must wait for the store lock, or there is no window to race"
    );

    // The switch lands while the propagation is blocked — the login cleared the
    // consumers and published a store naming the other account. Published as the
    // captured bytes, because the real writer would contend for the lock this
    // test is holding to create the window.
    std::fs::write(&store, &switched_store).unwrap();
    assert_eq!(unsafe { libc::flock(held.as_raw_fd(), libc::LOCK_UN) }, 0);
    drop(held);
    worker.join().unwrap();

    assert!(
        std::fs::symlink_metadata(&netd_cert).is_err()
            && std::fs::symlink_metadata(home.path().join("device.cert")).is_err(),
        "the store named another account by the time the lock was granted, so the \
         cert of the account this desk signed out of reached no consumer"
    );
}

/// A relocated candidate that is a DANGLING symlink is present, not absent: the
/// entry is there and its target is gone, which relinking fixes. Reading through
/// it answers `NotFound`, so folding that into "no device cert is cached" would
/// name a path that is fine and hide the one that is not.
#[cfg(unix)]
#[test]
#[serial]
fn a_dangling_relocated_symlink_is_reported_not_treated_as_absent() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let netd_cert = elsewhere.path().join("desk.cert");
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    seed_and_key(home.path(), [0x5A; 32]);
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode([0x42u8; 32]));
    std::os::unix::fs::symlink(elsewhere.path().join("gone.cert"), &netd_cert).unwrap();

    let err = resolve_device_binding()
        .expect_err("a link to nothing is not a cert")
        .to_string();
    assert!(
        err.contains(&netd_cert.display().to_string()) && err.contains("symlink"),
        "and the refusal names the link, not the CLI's own path: {err}"
    );
}

/// A relocated candidate that is PRESENT and unreadable is reported as itself,
/// not folded into "no device cert is cached" for the CLI's own path: that answer
/// names the wrong file and hides the fix. A FIFO also proves the kind is checked
/// before the open on this path too, so a candidate cannot hang the command.
#[cfg(unix)]
#[test]
#[serial]
fn an_unreadable_relocated_candidate_is_named_not_reported_as_absent() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let netd_cert = elsewhere.path().join("desk.cert");
    assert!(
        std::process::Command::new("mkfifo")
            .arg(&netd_cert)
            .status()
            .map(|s| s.success())
            .unwrap_or(false),
        "mkfifo is needed to build the hazard this test is about"
    );
    let home_path = home.path().to_path_buf();
    let netd_path = netd_cert.clone();

    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _g = HomeGuard::set(&home_path);
        let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_path);
        seed_and_key(&home_path, [0x5A; 32]);
        seed_session_account(&home_path, &URL_SAFE_NO_PAD.encode([0x42u8; 32]));
        let _ = tx.send(
            resolve_device_binding()
                .map(|_| ())
                .map_err(|e| e.to_string()),
        );
    });
    let err = rx
        .recv_timeout(std::time::Duration::from_secs(10))
        .expect("resolving must ANSWER: a read that blocks on the FIFO never returns")
        .expect_err("a named pipe is not a cert");
    assert!(
        err.contains("named pipe") && err.contains(&netd_cert.display().to_string()),
        "and it names the relocated path and what is there: {err}"
    );
}

/// A relocated cache that names ANOTHER account is not adopted: that is what an
/// account switch leaves at a path it could not see, and believing it would bind
/// this desk to the account it signed out of. No cert is the correct answer.
#[test]
#[serial]
fn a_relocated_cert_for_another_account_is_not_adopted() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let netd_cert = elsewhere.path().join("desk.cert");
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    std::fs::write(&netd_cert, make_cert_b64(device_key, [0x42u8; 32])).unwrap();
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode([0x99u8; 32]));

    let err = resolve_device_binding()
        .expect_err("a cert for another account is not this desk's binding")
        .to_string();
    assert!(
        err.contains("no device cert is cached") && err.contains("cerulion login"),
        "and the refusal says what to run: {err}"
    );
    assert!(
        std::fs::symlink_metadata(home.path().join("device.cert")).is_err(),
        "and nothing was copied to the CLI's own path"
    );
}

/// The propagation is gated on the cert's IDENTITY, not on its bytes existing: a
/// cert binding an account the store does not name is the previous account's,
/// left behind by a switch that could not see this consumer path. Copying it
/// would re-bind netd to the account this desk signed OUT of.
#[test]
#[serial]
#[traced_test]
fn a_cert_the_store_does_not_name_is_not_spread_to_a_consumer_with_none() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let netd_cert = elsewhere.path().join("desk.cert");
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    // The cert binds account A; the store has moved on to account B.
    std::fs::write(
        home.path().join("device.cert"),
        make_cert_b64(device_key, [0x42u8; 32]),
    )
    .unwrap();
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode([0x99u8; 32]));

    resolve_device_binding().expect("the cert is still the cryptographic truth");

    assert!(
        logs_contain(DIVERGENCE_WARN),
        "the divergence is surfaced, as before"
    );
    assert!(
        std::fs::symlink_metadata(&netd_cert).is_err(),
        "and the stale cert is NOT copied to the consumer with none: it names an \
         account this desk is not signed in as"
    );
}

/// A consumer that holds SOMETHING is left exactly as it is, including a symlink
/// whose target something else rotates — replacing it with a regular file would
/// take the path out of that rotation. Only an ABSENT one is filled.
#[cfg(unix)]
#[test]
#[serial]
fn a_consumer_that_already_holds_something_is_left_alone() {
    let home = tempfile::tempdir().unwrap();
    let elsewhere = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let rotated = elsewhere.path().join("rotated-by-netd.cert");
    let netd_cert = elsewhere.path().join("desk.cert");
    let device_key = seed_and_key(home.path(), [0x5A; 32]);
    let account = [0x42u8; 32];
    let cert_b64 = make_cert_b64(device_key, account);
    std::fs::write(&rotated, &cert_b64).unwrap();
    std::os::unix::fs::symlink(&rotated, &netd_cert).unwrap();
    let _n = EnvGuard::set("CERULION_NETD_DEVICE_CERT", &netd_cert);
    std::fs::write(home.path().join("device.cert"), &cert_b64).unwrap();
    seed_session_account(home.path(), &URL_SAFE_NO_PAD.encode(account));

    resolve_device_binding().expect("the cert resolves");

    assert!(
        std::fs::symlink_metadata(&netd_cert)
            .unwrap()
            .file_type()
            .is_symlink(),
        "the link is still a link, so the path still follows the rotation"
    );
    std::fs::write(&rotated, "rotated-since").unwrap();
    assert_eq!(
        std::fs::read_to_string(&netd_cert).unwrap(),
        "rotated-since",
        "and a regular file frozen at the identical bytes would stop tracking it"
    );
}

/// A cert path that is a FIFO is REFUSED by kind, not opened: reading a pipe with
/// no writer blocks forever, and a command that hangs with no output is worse
/// than one that says what is wrong. Run on a worker with a deadline, so a
/// regression to a blocking read FAILS instead of hanging the suite.
#[cfg(unix)]
#[test]
#[serial]
fn a_special_file_at_the_cert_path_is_refused_by_kind_not_opened() {
    let home = tempfile::tempdir().unwrap();
    let cert = home.path().join("device.cert");
    let made = std::process::Command::new("mkfifo")
        .arg(&cert)
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    assert!(
        made,
        "mkfifo is needed to build the hazard this test is about"
    );
    let home_path = home.path().to_path_buf();

    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _g = HomeGuard::set(&home_path);
        let _ = tx.send(
            resolve_device_binding()
                .map(|_| ())
                .map_err(|e| e.to_string()),
        );
    });
    let outcome = rx
        .recv_timeout(std::time::Duration::from_secs(10))
        .expect("resolving must ANSWER: a read that blocks on the FIFO never returns");

    let err = outcome.expect_err("a named pipe is not a cert");
    assert!(
        err.contains("named pipe") && err.contains("reading the device cert"),
        "and it names what is at the path: {err}"
    );
}

#[test]
#[serial]
#[traced_test]
fn resolve_with_no_session_returns_the_cert_without_a_divergence_warn() {
    // No auth.json at all (state() == None) → the cross-check is skipped: the cert is
    // returned and NO divergence warn fires (the None branch, not a false-positive).
    let home = tempfile::tempdir().unwrap();
    let _g = HomeGuard::set(home.path());
    let device_key = seed_and_key(home.path(), [0x5A; 32]);

    let account = [0x77u8; 32];
    std::fs::write(
        home.path().join("device.cert"),
        make_cert_b64(device_key, account),
    )
    .unwrap();
    // (no auth.json seeded)

    let binding = resolve_device_binding().expect("a cert with no cached session resolves");
    assert_eq!(binding.account, AccountId(account));
    assert!(
        !logs_contain(DIVERGENCE_WARN),
        "no session means no divergence warn"
    );
}
