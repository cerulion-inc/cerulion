// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use serde_json::{json, Value};
use serial_test::serial;

const ENDPOINT: &str = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
const OWNER: AccountId = AccountId([0x11; 32]);

fn row(id: u8, name: &str) -> Value {
    json!({
        "robot_id": URL_SAFE_NO_PAD.encode([id; 32]),
        "hostname": name,
        "owner_account_id": URL_SAFE_NO_PAD.encode(OWNER.0),
        "robot_transport_key": ENDPOINT
    })
}

fn body(rows: Vec<Value>) -> Vec<u8> {
    serde_json::to_vec(&json!({ "robots": rows })).unwrap()
}

#[test]
fn owned_rows_sort_stably_and_known_endpoint_does_not_assert_presence() {
    let bytes = body(vec![row(2, "beta"), row(3, "alpha"), row(1, "alpha")]);
    let first = parse_robots(&bytes, OWNER).unwrap();
    assert_eq!(
        first.iter().map(|r| r.robot_id.0[0]).collect::<Vec<_>>(),
        [1, 3, 2]
    );
    assert_eq!(first, parse_robots(&bytes, OWNER).unwrap());
    assert_eq!(
        first[0].endpoint,
        RobotEndpoint::Known(PublicKey([
            0xd7, 0x5a, 0x98, 0x01, 0x82, 0xb1, 0x0a, 0xb7, 0xd5, 0x4b, 0xfe, 0xd3, 0xc9, 0x64,
            0x07, 0x3a, 0x0e, 0xe1, 0x72, 0xf3, 0xda, 0xa6, 0x23, 0x25, 0xaf, 0x02, 0x1a, 0x68,
            0xf7, 0x07, 0x51, 0x1a,
        ]))
    );
    assert!(parse_robots(br#"{"robots":[]}"#, OWNER).unwrap().is_empty());
}

#[test]
fn omitted_endpoint_is_unknown_but_null_or_malformed_keys_are_errors() {
    let mut legacy = row(1, "legacy");
    legacy
        .as_object_mut()
        .unwrap()
        .remove("robot_transport_key");
    assert_eq!(
        parse_robots(&body(vec![legacy]), OWNER).unwrap()[0].endpoint,
        RobotEndpoint::Unknown {
            reason:
                "the account service did not provide this robot's endpoint key; presence is unknown"
        }
    );
    for value in [
        Value::Null,
        json!(7),
        json!(""),
        json!("a".repeat(63)),
        json!("g".repeat(64)),
        json!("00".repeat(32)),
        json!(format!(" {ENDPOINT}")),
    ] {
        let mut invalid = row(1, "robot");
        invalid["robot_transport_key"] = value;
        assert!(parse_robots(&body(vec![invalid]), OWNER).is_err());
    }
}

#[test]
fn foreign_owner_duplicate_ids_and_unsafe_names_refuse_whole_catalog() {
    let mut foreign = row(2, "foreign");
    foreign["owner_account_id"] = json!(URL_SAFE_NO_PAD.encode([0x22; 32]));
    assert!(parse_robots(&body(vec![row(1, "owned"), foreign]), OWNER)
        .unwrap_err()
        .to_string()
        .contains("another account"));
    assert!(
        parse_robots(&body(vec![row(1, "one"), row(1, "two")]), OWNER)
            .unwrap_err()
            .to_string()
            .contains("repeats")
    );
    for name in [
        "",
        "--option",
        "bad\nname",
        "bad\u{1b}[31m",
        "../name",
        "a b",
        &"x".repeat(254),
    ] {
        assert!(parse_robots(&body(vec![row(1, name)]), OWNER).is_err());
    }
    for field in ["robot_id", "owner_account_id"] {
        for value in [
            "not-an-id",
            "",
            &URL_SAFE_NO_PAD.encode([7; 31]),
            &format!("{}=", URL_SAFE_NO_PAD.encode([7; 32])),
        ] {
            let mut malformed = row(1, "robot");
            malformed[field] = json!(value);
            assert!(parse_robots(&body(vec![malformed]), OWNER).is_err());
        }
    }
}

#[test]
fn body_count_and_reader_errors_are_bounded_and_do_not_echo_untrusted_text() {
    assert!(
        parse_robots(&body(vec![row(1, "robot"); MAX_ROBOTS + 1]), OWNER)
            .unwrap_err()
            .to_string()
            .contains("count limit")
    );
    let exact = vec![b' '; MAX_BODY_BYTES];
    assert_eq!(read_body(exact.as_slice()).unwrap().len(), MAX_BODY_BYTES);
    let oversized = vec![b' '; MAX_BODY_BYTES + 1];
    assert!(read_body(oversized.as_slice()).is_err());
    assert!(parse_robots(&oversized, OWNER).is_err());
    assert!(!parse_robots(b"server-secret\x1b[31m", OWNER)
        .unwrap_err()
        .to_string()
        .contains("server-secret"));
    struct Broken;
    impl Read for Broken {
        fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
            Err(std::io::Error::other("private-peer-secret"))
        }
    }
    assert!(!read_body(Broken)
        .unwrap_err()
        .to_string()
        .contains("private-peer-secret"));
}

#[test]
fn unsupported_catalog_is_distinct_from_empty_and_errors_do_not_depend_on_body() {
    assert!(check_status(reqwest::StatusCode::OK).is_ok());
    assert!(check_status(reqwest::StatusCode::NOT_FOUND)
        .unwrap_err()
        .to_string()
        .contains("does not provide a robot catalog"));
    assert!(check_status(reqwest::StatusCode::UNAUTHORIZED)
        .unwrap_err()
        .to_string()
        .contains("cerulion login"));
    assert!(check_status(reqwest::StatusCode::FORBIDDEN)
        .unwrap_err()
        .to_string()
        .contains("HTTP 403"));
    for url in [
        "https://accounts.example.test",
        "http://127.0.0.1:8787",
        "http://[::1]:8787",
    ] {
        assert!(service_url(url).is_ok());
    }
    for url in [
        "http://accounts.example.test",
        "https://token@accounts.example.test",
        "https://accounts.example.test?secret=token",
        "https://accounts.example.test#fragment",
    ] {
        let error = service_url(url).unwrap_err().to_string();
        assert!(!error.contains("secret=token"));
        assert!(!error.contains("token@"));
    }
}

struct EnvGuard(Option<std::ffi::OsString>);
impl EnvGuard {
    fn home(path: &Path) -> Self {
        let old = std::env::var_os("CERULION_HOME");
        std::env::set_var("CERULION_HOME", path);
        Self(old)
    }
}
impl Drop for EnvGuard {
    fn drop(&mut self) {
        match self.0.take() {
            Some(value) => std::env::set_var("CERULION_HOME", value),
            None => std::env::remove_var("CERULION_HOME"),
        }
    }
}

fn login_state(account: String) -> AuthState {
    AuthState {
        account_id: account,
        session_token: "private-session-token".into(),
        refresh_token: "private-refresh-token".into(),
        expires_at_ns: u64::MAX,
        logged_in_ever: true,
        role: None,
    }
}

#[test]
#[serial]
fn account_key_logout_and_refresh_changes_discard_results_without_cache_write() {
    let temp = tempfile::tempdir().unwrap();
    let _home = EnvGuard::home(temp.path());
    let auth_path = temp.path().join("auth.json");
    let key_path = temp.path().join("desk.key");
    let state = login_state(URL_SAFE_NO_PAD.encode(OWNER.0));
    auth::write_to(&auth_path, &state).unwrap();
    std::fs::write(&key_path, [0x31; 32]).unwrap();
    let before = snapshot(&auth_path).unwrap();
    let directory = finish(before.clone(), &body(vec![row(1, "robot")])).unwrap();
    directory.validate_current_login().unwrap();
    let debug = format!("{directory:?}");
    assert!(!debug.contains("private-session-token") && !debug.contains("private-refresh-token"));
    for changed in [
        AuthState {
            account_id: URL_SAFE_NO_PAD.encode([0x22; 32]),
            ..state.clone()
        },
        AuthState {
            session_token: "new-session".into(),
            ..state.clone()
        },
        AuthState {
            session_token: String::new(),
            ..state.clone()
        },
    ] {
        auth::write_to(&auth_path, &changed).unwrap();
        assert!(finish(before.clone(), &body(vec![row(1, "robot")])).is_err());
        assert!(directory.validate_current_login().is_err());
    }
    auth::write_to(&auth_path, &state).unwrap();
    std::fs::write(&key_path, [0x32; 32]).unwrap();
    assert!(finish(before.clone(), &body(vec![row(1, "robot")])).is_err());
    std::fs::write(&key_path, [0x31; 32]).unwrap();
    assert!(directory.validate_current_login().is_ok());
    std::fs::remove_file(&key_path).unwrap();
    assert!(finish(before, &body(vec![row(1, "robot")])).is_err());
    assert!(!temp.path().join("robots.toml").exists());
    assert!(
        !temp.path().join("device-chain.json").exists(),
        "listing does not require or create an owner chain"
    );
}

#[test]
#[serial]
fn uuid_login_maps_to_the_wire_owner_without_mutating_auth_identity() {
    let temp = tempfile::tempdir().unwrap();
    let _home = EnvGuard::home(temp.path());
    let auth_path = temp.path().join("auth.json");
    let uuid = "00112233-4455-6677-8899-aabbccddeeff";
    let state = login_state(uuid.into());
    auth::write_to(&auth_path, &state).unwrap();
    std::fs::write(temp.path().join("desk.key"), [0x31; 32]).unwrap();
    let before = snapshot(&auth_path).unwrap();
    let expected = account_identity::pairing_account_id(uuid).unwrap();
    let mut robot = row(1, "robot");
    robot["owner_account_id"] = json!(URL_SAFE_NO_PAD.encode(expected.0));
    let directory = finish(before, &body(vec![robot])).unwrap();
    assert_eq!(directory.account_id, expected);
    assert_eq!(
        auth::load_from(&auth_path).state().unwrap().account_id,
        uuid
    );
}

#[cfg(unix)]
fn write_expired_proof(dir: &Path, account: AccountId, seed: [u8; 32], issued: u64) {
    use cerulion_pairing::format::{
        DeviceCert, IntermediateCert, PrincipalKind, Scope, Validity, FORMAT_VERSION,
    };
    use ed25519_dalek::SigningKey;
    let issuer = SigningKey::from_bytes(&[0x22; 32]);
    let root = SigningKey::from_bytes(&[0x44; 32]);
    let validity = Validity {
        not_before_ns: 10,
        not_after_ns: 100,
    };
    let intermediate = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: PublicKey(issuer.verifying_key().to_bytes()),
        validity,
        issued_at_ns: 10,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&root]);
    let leaf = DeviceCert {
        version: FORMAT_VERSION,
        device_key: PublicKey(SigningKey::from_bytes(&seed).verifying_key().to_bytes()),
        account,
        principal_kind: PrincipalKind::Human,
        scope: Scope::OWNER_FULL,
        validity,
        issued_at_ns: issued,
        issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
    }
    .sign(&issuer);
    let leaf = URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&leaf).unwrap());
    let intermediate = URL_SAFE_NO_PAD.encode(postcard::to_stdvec(&intermediate).unwrap());
    std::fs::write(dir.join("device.cert"), &leaf).unwrap();
    crate::owner_certificate::stage_at(&dir.join("device-chain.json"), &leaf, &intermediate)
        .unwrap()
        .commit()
        .unwrap();
}

#[cfg(unix)]
#[test]
#[serial]
fn daemon_snapshot_preserves_public_identity_and_uses_current_proof_without_secrets() {
    let temp = tempfile::tempdir().unwrap();
    let _home = EnvGuard::home(temp.path());
    let auth_path = temp.path().join("auth.json");
    let state = login_state(URL_SAFE_NO_PAD.encode(OWNER.0));
    let seed: [u8; 32] =
        hex::decode("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60")
            .unwrap()
            .try_into()
            .unwrap();
    auth::write_to(&auth_path, &state).unwrap();
    std::fs::write(temp.path().join("desk.key"), seed).unwrap();
    let mut legacy = row(2, "legacy");
    legacy
        .as_object_mut()
        .unwrap()
        .remove("robot_transport_key");
    let directory = finish(
        snapshot(&auth_path).unwrap(),
        &body(vec![row(1, "known"), legacy]),
    )
    .unwrap();
    let initial = directory.daemon_snapshot().unwrap();
    assert_eq!(initial.auth_account_id, state.account_id);
    assert_eq!(initial.pairing_account_id, [0x11; 32]);
    assert_eq!(hex::encode(initial.device_key), ENDPOINT);
    assert_eq!(initial.robots[0].robot_id, [1; 32]);
    assert_eq!(initial.robots[0].hostname, "known");
    assert_eq!(initial.robots[0].endpoint_key, Some(initial.device_key));
    assert_eq!(initial.robots[1].robot_id, [2; 32]);
    assert_eq!(initial.robots[1].endpoint_key, None);
    assert!(initial.owner_chain.is_none());

    // A proof refresh after the HTTP directory read is intentionally allowed for
    // the exact same login and key. Its historical expiry is still robot-verified.
    write_expired_proof(temp.path(), OWNER, seed, 10);
    let snapshot = directory.daemon_snapshot().unwrap();
    let proof = cerulion_pairing::verify::OwnerCertificatePresentationWire::from_postcard(
        snapshot.owner_chain.as_deref().unwrap(),
    )
    .unwrap();
    assert_eq!(proof.device_cert.cert.account, OWNER);
    assert_eq!(proof.device_cert.cert.device_key.0, initial.device_key);
    assert_eq!(proof.device_cert.cert.validity.not_after_ns, 100);
    assert_eq!(proof.device_cert.cert.issued_at_ns, 10);
    write_expired_proof(temp.path(), OWNER, seed, 11);
    let refreshed = directory.daemon_snapshot().unwrap();
    assert_eq!(
        cerulion_pairing::verify::OwnerCertificatePresentationWire::from_postcard(
            refreshed.owner_chain.as_deref().unwrap(),
        )
        .unwrap()
        .device_cert
        .cert
        .issued_at_ns,
        11
    );
    for public in [initial, snapshot, refreshed] {
        let wire = serde_json::to_string(&public).unwrap();
        assert!(!wire.contains("private-session-token"));
        assert!(!wire.contains("private-refresh-token"));
        assert!(!wire.contains(&hex::encode(seed)));
        assert!(!wire.contains(&serde_json::to_string(&seed).unwrap()));
    }
    std::fs::remove_file(temp.path().join("device-chain.json")).unwrap();
    assert!(directory.daemon_snapshot().unwrap().owner_chain.is_none());
}

#[cfg(unix)]
#[test]
#[serial]
fn daemon_snapshot_refuses_changed_login_key_expiry_and_partial_proof() {
    let temp = tempfile::tempdir().unwrap();
    let _home = EnvGuard::home(temp.path());
    let auth_path = temp.path().join("auth.json");
    let state = login_state(URL_SAFE_NO_PAD.encode(OWNER.0));
    let seed = [0x31; 32];
    auth::write_to(&auth_path, &state).unwrap();
    std::fs::write(temp.path().join("desk.key"), seed).unwrap();
    let directory = finish(snapshot(&auth_path).unwrap(), &body(vec![row(1, "robot")])).unwrap();
    for changed in [
        AuthState {
            account_id: URL_SAFE_NO_PAD.encode([0x22; 32]),
            ..state.clone()
        },
        AuthState {
            session_token: "rotated-session".into(),
            ..state.clone()
        },
        AuthState {
            refresh_token: "rotated-refresh".into(),
            ..state.clone()
        },
        AuthState {
            logged_in_ever: false,
            ..state.clone()
        },
        AuthState {
            expires_at_ns: 0,
            ..state.clone()
        },
    ] {
        auth::write_to(&auth_path, &changed).unwrap();
        assert!(directory.daemon_snapshot().is_err());
    }
    auth::write_to(&auth_path, &state).unwrap();
    std::fs::write(temp.path().join("desk.key"), [0x32; 32]).unwrap();
    assert!(directory.daemon_snapshot().is_err());
    std::fs::write(temp.path().join("desk.key"), seed).unwrap();
    std::fs::write(temp.path().join("device-chain.json"), "{}").unwrap();
    assert!(directory.daemon_snapshot().is_err());
    write_expired_proof(temp.path(), AccountId([0x22; 32]), seed, 10);
    assert!(directory.daemon_snapshot().is_err());
    write_expired_proof(temp.path(), OWNER, seed, 10);
    // The public path finishes under the existing transaction without trying to
    // acquire a second lock through owner_certificate::load().
    let completed = auth::with_store_lock(&auth_path, || Ok(directory.daemon_snapshot())).unwrap();
    assert!(completed.is_ok());
}
