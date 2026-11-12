// SPDX-License-Identifier: AGPL-3.0-only
use super::*;

const EXPIRED: &[u8] = br#"{"account_id":"00112233-4455-4677-8899-aabbccddeeff","session_token":"expired-session","refresh_token":"offline-refresh","expires_at_ns":0,"logged_in_ever":true}"#;

#[test]
fn persisted_prior_login_allows_offline_serving_without_considering_expiry() {
    assert_eq!(classify(EXPIRED), ServingLogin::Allowed);
    for role in [
        "null",
        "\"robot\"",
        "\"future-role\"",
        "42",
        "{\"future\":true}",
    ] {
        let mut text = String::from_utf8(EXPIRED.to_vec()).unwrap();
        text.pop();
        text.push_str(&format!(",\"role\":{role}}}"));
        assert_eq!(classify(text.as_bytes()), ServingLogin::Allowed);
    }
}

#[test]
fn incomplete_false_and_corrupt_login_records_do_not_authorize_serving() {
    let mut record: serde_json::Value = serde_json::from_slice(EXPIRED).unwrap();
    record["logged_in_ever"] = serde_json::json!(false);
    assert_eq!(
        classify(&serde_json::to_vec(&record).unwrap()),
        ServingLogin::Refused(LoginProblem::NeverLoggedIn)
    );
    for key in [
        "account_id",
        "session_token",
        "refresh_token",
        "expires_at_ns",
        "logged_in_ever",
    ] {
        let mut record: serde_json::Value = serde_json::from_slice(EXPIRED).unwrap();
        record.as_object_mut().unwrap().remove(key);
        assert_eq!(
            classify(&serde_json::to_vec(&record).unwrap()),
            ServingLogin::Refused(LoginProblem::Malformed),
            "missing {key}"
        );
        let mut record: serde_json::Value = serde_json::from_slice(EXPIRED).unwrap();
        record[key] = serde_json::Value::Null;
        assert_eq!(
            classify(&serde_json::to_vec(&record).unwrap()),
            ServingLogin::Refused(LoginProblem::Malformed),
            "null {key}"
        );
    }
    for bytes in [
        b"".as_slice(),
        b"{",
        b"{\"logged_in_ever\":true}",
        b"[]",
        b"null",
    ] {
        assert_eq!(
            classify(bytes),
            ServingLogin::Refused(LoginProblem::Malformed)
        );
    }
    let duplicate = String::from_utf8(EXPIRED.to_vec()).unwrap().replace(
        "\"logged_in_ever\":true",
        "\"logged_in_ever\":false,\"logged_in_ever\":true",
    );
    assert_eq!(
        classify(duplicate.as_bytes()),
        ServingLogin::Refused(LoginProblem::Malformed)
    );
}

#[test]
fn bounded_reader_preserves_valid_corrupt_and_oversized_state_without_writes() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("auth.json");
    assert_eq!(read_at(&path), ServingLogin::Refused(LoginProblem::Missing));
    assert!(!path.exists());
    for (bytes, expected) in [
        (EXPIRED.to_vec(), ServingLogin::Allowed),
        (
            b"{broken".to_vec(),
            ServingLogin::Refused(LoginProblem::Malformed),
        ),
        (
            vec![b' '; MAX_AUTH_BYTES as usize + 1],
            ServingLogin::Refused(LoginProblem::TooLarge),
        ),
    ] {
        std::fs::write(&path, &bytes).unwrap();
        assert_eq!(read_at(&path), expected);
        assert_eq!(std::fs::read(&path).unwrap(), bytes);
    }
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
    assert_eq!(
        read_at(dir.path()),
        ServingLogin::Refused(LoginProblem::NotRegular)
    );
}

#[cfg(unix)]
#[test]
fn fifo_is_rejected_without_waiting_for_a_writer_and_regular_symlinks_remain_compatible() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("auth.json");
    let status = std::process::Command::new("mkfifo")
        .arg(&path)
        .status()
        .unwrap();
    assert!(status.success());
    assert_eq!(
        read_at(&path),
        ServingLogin::Refused(LoginProblem::NotRegular)
    );
    std::fs::remove_file(&path).unwrap();
    let target = dir.path().join("stored.json");
    std::fs::write(&target, EXPIRED).unwrap();
    std::os::unix::fs::symlink(&target, &path).unwrap();
    assert_eq!(read_at(&path), ServingLogin::Allowed);
    assert!(std::fs::symlink_metadata(&path).unwrap().is_symlink());
}

#[test]
fn identity_reader_returns_only_the_saved_account_and_refuses_absent_or_empty_identity() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("auth.json");
    assert_eq!(read_identity_at(&path), Err(LoginProblem::Missing));
    std::fs::write(&path, EXPIRED).unwrap();
    assert_eq!(
        read_identity_at(&path).unwrap(),
        "00112233-4455-4677-8899-aabbccddeeff"
    );
    for account in [
        serde_json::Value::Null,
        serde_json::json!(""),
        serde_json::json!(" "),
        serde_json::json!(42),
    ] {
        let mut record: serde_json::Value = serde_json::from_slice(EXPIRED).unwrap();
        record["account_id"] = account;
        std::fs::write(&path, serde_json::to_vec(&record).unwrap()).unwrap();
        assert_eq!(read_identity_at(&path), Err(LoginProblem::Malformed));
    }
    assert_eq!(
        read_regular_bounded(&path, u64::MAX),
        Err(LoginProblem::TooLarge)
    );
}

#[test]
fn durable_pairing_authorizes_without_login_and_corrupt_or_incomplete_state_does_not() {
    use cerulion_pairing::client::DeviceIdentity;
    use cerulion_pairing::format::{AccountId, PrincipalKind, RobotId, RootSet};
    use cerulion_pairing::verify::TrustStore;
    let root = tempfile::tempdir().unwrap();
    let dir = root.path().join("remoted");
    std::fs::create_dir(&dir).unwrap();
    let seed = [0x61; 32];
    let key = DeviceIdentity::from_seed(&seed).public_key();
    let mac = [0x62; 32];
    let mut store = TrustStore::provision(
        RobotId([0x63; 32]),
        key,
        RootSet::new(vec![DeviceIdentity::from_seed(&[0x64; 32]).public_key()], 1).unwrap(),
        b"physical-pair-secret",
        0,
    )
    .unwrap()
    .with_path(dir.join("trust_store"));
    store
        .claim(
            AccountId([0x65; 32]),
            b"physical-pair-secret",
            PrincipalKind::Human,
            0,
        )
        .unwrap();
    store.save(&mac).unwrap();
    let mut index =
        cerulion_remoted::DeviceAccountIndex::new().with_path(dir.join("device_index.json"));
    index.bind(key, AccountId([0x65; 32]));
    index.save(&mac).unwrap();
    std::fs::write(dir.join("device_key"), seed).unwrap();
    std::fs::write(dir.join("trust_store.mac_key"), mac).unwrap();
    let policy = EgressLoginPolicy::PriorLoginOrPairedRobot {
        auth_path: None,
        state_root: Some(root.path().to_path_buf()),
    };
    assert_eq!(policy.check(), Ok(()));
    let saved = saved_robot(root.path()).unwrap();
    assert_eq!(saved.robot_id, "63".repeat(32));
    assert_eq!(saved.owner_account, "65".repeat(32));
    for (name, bad) in [
        ("device_key", vec![0x66; 32]),
        ("device_key", vec![0x61; 33]),
        ("trust_store.mac_key", vec![]),
        ("trust_store.mac_key", vec![0x66; 32]),
        ("trust_store", b"corrupt-store".to_vec()),
        ("device_index.json", b"corrupt-index".to_vec()),
    ] {
        let path = dir.join(name);
        let original = std::fs::read(&path).unwrap();
        std::fs::write(&path, &bad).unwrap();
        assert_eq!(policy.check(), Err(REFUSAL), "{name}");
        assert_eq!(std::fs::read(&path).unwrap(), bad);
        std::fs::write(path, original).unwrap();
    }
    std::fs::remove_file(dir.join("device_index.json")).unwrap();
    assert_eq!(policy.check(), Err(REFUSAL));
    assert!(!dir.join("device_index.json").exists());
    let auth = root.path().join("auth.json");
    std::fs::write(&auth, EXPIRED).unwrap();
    let offline = EgressLoginPolicy::PriorLoginOrPairedRobot {
        auth_path: Some(auth),
        state_root: Some(root.path().to_path_buf()),
    };
    assert_eq!(offline.check(), Ok(()));
}
