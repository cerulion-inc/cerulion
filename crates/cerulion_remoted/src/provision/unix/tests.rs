// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use std::os::unix::fs::{symlink, PermissionsExt};

use cerulion_pairing::format::{
    DeviceCert, Grant, IntermediateCert, PrincipalKind, Scope, Validity,
};
use ed25519_dalek::SigningKey;

const NOW: u64 = 1_000_000_000_000;
const EXPIRES: u64 = 2_000_000_000_000;
const SEED: [u8; 32] = [0x31; 32];
const OWNER: AccountId = AccountId([0x41; 32]);
const ROBOT: RobotId = RobotId([0x51; 32]);

fn artifact(value: &impl serde::Serialize) -> String {
    hex::encode(postcard::to_stdvec(value).unwrap())
}

fn setup() -> (tempfile::TempDir, ProvisionConfig, RegistrationBundle) {
    let temporary = tempfile::tempdir().unwrap();
    let base = temporary.path().canonicalize().unwrap();
    let state_root = base.join("state");
    fs::DirBuilder::new()
        .mode(0o700)
        .create(&state_root)
        .unwrap();
    let key_file = base.join("desk.key");
    write_new(&key_file, &SEED).unwrap();
    let root = SigningKey::from_bytes(&[0x11; 32]);
    let issuer = SigningKey::from_bytes(&[0x21; 32]);
    let issuer_key = PublicKey(issuer.verifying_key().to_bytes());
    let validity = Validity {
        not_before_ns: 1,
        not_after_ns: EXPIRES,
    };
    let roots = RootSet::new(vec![PublicKey(root.verifying_key().to_bytes())], 1).unwrap();
    let intermediate = IntermediateCert {
        version: FORMAT_VERSION,
        intermediate_key: issuer_key,
        validity,
        issued_at_ns: 1,
        max_scope: Scope::OWNER_FULL,
    }
    .sign_by_roots(&[&root]);
    let cert = DeviceCert {
        version: FORMAT_VERSION,
        device_key: DeviceIdentity::from_seed(&SEED).public_key(),
        account: OWNER,
        principal_kind: PrincipalKind::Human,
        scope: Scope::OWNER_FULL,
        validity,
        issued_at_ns: 1,
        issuer_key,
    }
    .sign(&issuer);
    let grant = Grant {
        version: FORMAT_VERSION,
        subject: OWNER,
        robot: ROBOT,
        scope: Scope::OWNER_FULL,
        principal_kind: PrincipalKind::Human,
        delegation_depth: 0,
        validity,
        issued_at_ns: 1,
        issuer: OWNER,
        issuer_key,
    }
    .sign(&issuer);
    let bundle = RegistrationBundle {
        version: 1,
        robot_id: hex::encode(ROBOT.0),
        owner_account: hex::encode(OWNER.0),
        device_key_file: key_file,
        root_set_postcard: artifact(&roots),
        intermediate_postcard: artifact(&intermediate),
        device_cert_postcard: artifact(&cert),
        owner_grant_postcard: artifact(&grant),
    };
    let config = ProvisionConfig {
        registration_bundle: state_root.join("registration.json"),
        state_root,
    };
    write_new(
        &config.registration_bundle,
        &serde_json::to_vec(&bundle).unwrap(),
    )
    .unwrap();
    (temporary, config, bundle)
}

fn update(config: &ProvisionConfig, bundle: &RegistrationBundle) {
    fs::write(
        &config.registration_bundle,
        serde_json::to_vec(bundle).unwrap(),
    )
    .unwrap();
}

fn snapshot(dir: &Path) -> Vec<(String, Vec<u8>)> {
    let mut files: Vec<_> = fs::read_dir(dir)
        .unwrap()
        .map(|entry| {
            let entry = entry.unwrap();
            (
                entry.file_name().to_str().unwrap().to_owned(),
                fs::read(entry.path()).unwrap(),
            )
        })
        .collect();
    files.sort_by(|a, b| a.0.cmp(&b.0));
    files
}

#[tokio::test]
async fn signed_registration_publishes_claimed_v3_state_with_the_login_key() {
    let (_temporary, config, _bundle) = setup();
    let result = provision(&config, NOW).unwrap();
    assert_eq!(
        result,
        Provisioned {
            robot_id: ROBOT,
            endpoint_id: DeviceIdentity::from_seed(&SEED).public_key(),
            owner_account: OWNER
        }
    );
    let dir = config.state_root.join(REMOTED_SUBDIR);
    assert_eq!(fs::read(dir.join(DEVICE_KEY_FILENAME)).unwrap(), SEED);
    let mac = fs::read(dir.join(TRUST_STORE_MAC_KEY_FILENAME)).unwrap();
    let store_bytes = fs::read(dir.join(TRUST_STORE_FILENAME)).unwrap();
    assert_eq!(&store_bytes[8..10], &[3, 0]);
    let store = TrustStore::load(dir.join(TRUST_STORE_FILENAME), &mac).unwrap();
    assert_eq!(store.robot_id(), ROBOT);
    assert_eq!(store.owner(), Some(OWNER));
    assert_eq!(store.high_water_ns(), NOW);
    assert_eq!(store.access_rows().len(), 1);
    assert_eq!(store.access_rows()[0].scope, Scope::OWNER_FULL);
    let index = DeviceAccountIndex::load(dir.join(DEVICE_INDEX_FILENAME), &mac).unwrap();
    assert_eq!(index.account_for(&result.endpoint_id), Some(OWNER));
    assert_eq!(index.len(), 1);
    for (name, _) in snapshot(&dir) {
        assert_eq!(
            fs::metadata(dir.join(name)).unwrap().permissions().mode() & 0o777,
            0o600
        );
    }
    assert!(!dir.join(".trust_store.provision").exists());
}

#[tokio::test]
async fn a_complete_restart_preserves_every_byte_after_registration_certificates_expire() {
    let (_temporary, config, _bundle) = setup();
    let original = provision(&config, NOW).unwrap();
    let dir = config.state_root.join(REMOTED_SUBDIR);
    let before = snapshot(&dir);
    assert_eq!(provision(&config, EXPIRES + 1).unwrap(), original);
    assert_eq!(snapshot(&dir), before);
}

#[tokio::test]
async fn partial_or_corrupt_state_is_never_regenerated() {
    for missing in [
        DEVICE_KEY_FILENAME,
        TRUST_STORE_MAC_KEY_FILENAME,
        TRUST_STORE_FILENAME,
        DEVICE_INDEX_FILENAME,
        CHASSIS_SECRET_FILENAME,
    ] {
        let (_temporary, config, _bundle) = setup();
        provision(&config, NOW).unwrap();
        let dir = config.state_root.join(REMOTED_SUBDIR);
        fs::remove_file(dir.join(missing)).unwrap();
        let before = snapshot(&dir);
        assert!(provision(&config, NOW).is_err(), "{missing}");
        assert_eq!(snapshot(&dir), before);
    }
    let (_temporary, config, _bundle) = setup();
    provision(&config, NOW).unwrap();
    let dir = config.state_root.join(REMOTED_SUBDIR);
    fs::write(dir.join(TRUST_STORE_FILENAME), b"corrupt").unwrap();
    let before = snapshot(&dir);
    assert!(provision(&config, NOW).is_err());
    assert_eq!(snapshot(&dir), before);
}

#[tokio::test]
async fn changed_account_robot_or_login_key_cannot_reown_existing_state() {
    for change in 0..3 {
        let (_temporary, config, mut bundle) = setup();
        provision(&config, NOW).unwrap();
        let dir = config.state_root.join(REMOTED_SUBDIR);
        let before = snapshot(&dir);
        match change {
            0 => bundle.owner_account = hex::encode([0x61; 32]),
            1 => bundle.robot_id = hex::encode([0x62; 32]),
            _ => fs::write(&bundle.device_key_file, [0x63; 32]).unwrap(),
        }
        update(&config, &bundle);
        assert!(provision(&config, NOW).is_err());
        assert_eq!(snapshot(&dir), before);
    }
}

#[tokio::test]
async fn a_valid_mac_without_the_registered_owner_binding_is_not_complete_state() {
    for account in [None, Some(AccountId([0x61; 32]))] {
        let (_temporary, config, _bundle) = setup();
        let registered = provision(&config, NOW).unwrap();
        let dir = config.state_root.join(REMOTED_SUBDIR);
        let mac = fs::read(dir.join(TRUST_STORE_MAC_KEY_FILENAME)).unwrap();
        let path = dir.join(DEVICE_INDEX_FILENAME);
        let mut index = DeviceAccountIndex::new().with_path(&path);
        if let Some(account) = account {
            index.bind(registered.endpoint_id, account);
        }
        reserve_save_temp(&path).unwrap();
        index.save(&mac).unwrap();
        let before = snapshot(&dir);
        let error = provision(&config, NOW).unwrap_err().to_string();
        assert!(error.contains("stored owner device binding"), "{error}");
        assert_eq!(snapshot(&dir), before);
    }
}

#[tokio::test]
async fn invalid_owner_proofs_refuse_before_creating_robot_state() {
    for change in 0..6 {
        let (_temporary, config, mut bundle) = setup();
        match change {
            0 => bundle.owner_account = hex::encode([0x61; 32]),
            1 => bundle.robot_id = hex::encode([0x62; 32]),
            2 => fs::write(&bundle.device_key_file, [0x63; 32]).unwrap(),
            3 => {
                let other = SigningKey::from_bytes(&[0x71; 32]);
                bundle.root_set_postcard = artifact(
                    &RootSet::new(vec![PublicKey(other.verifying_key().to_bytes())], 1).unwrap(),
                );
            }
            4 => {
                let mut grant: SignedGrant = decode(&bundle.owner_grant_postcard).unwrap();
                grant.grant.scope = Scope::CODE_PAIR_DEFAULT;
                bundle.owner_grant_postcard =
                    artifact(&grant.grant.sign(&SigningKey::from_bytes(&[0x21; 32])));
            }
            _ => {
                let mut cert: SignedDeviceCert = decode(&bundle.device_cert_postcard).unwrap();
                cert.signature.0[0] ^= 1;
                bundle.device_cert_postcard = artifact(&cert);
            }
        }
        update(&config, &bundle);
        assert!(provision(&config, NOW).is_err(), "case {change}");
        assert!(!config.state_root.join(REMOTED_SUBDIR).exists());
    }
    let (_temporary, config, _bundle) = setup();
    assert!(provision(&config, EXPIRES + 1).is_err());
    assert!(!config.state_root.join(REMOTED_SUBDIR).exists());
}

#[tokio::test]
async fn bundle_and_login_key_require_private_bounded_regular_files() {
    let (_temporary, config, bundle) = setup();
    let uid = current_uid().unwrap();
    fs::set_permissions(
        &config.registration_bundle,
        fs::Permissions::from_mode(0o644),
    )
    .unwrap();
    assert!(read_registration(&config, uid).is_err());
    fs::set_permissions(
        &config.registration_bundle,
        fs::Permissions::from_mode(0o600),
    )
    .unwrap();
    fs::set_permissions(&bundle.device_key_file, fs::Permissions::from_mode(0o644)).unwrap();
    assert!(read_registration(&config, uid).is_err());
    fs::set_permissions(&bundle.device_key_file, fs::Permissions::from_mode(0o600)).unwrap();
    for size in [0, 31, 33] {
        fs::write(&bundle.device_key_file, vec![0; size]).unwrap();
        assert!(read_registration(&config, uid).is_err());
    }
    fs::write(&bundle.device_key_file, SEED).unwrap();
    let link = config.state_root.join("linked.json");
    symlink(&config.registration_bundle, &link).unwrap();
    let linked = ProvisionConfig {
        state_root: config.state_root.clone(),
        registration_bundle: link,
    };
    assert!(read_registration(&linked, uid).is_err());
    let key_target = bundle.device_key_file.clone();
    let mut linked_key = RegistrationBundle {
        device_key_file: config.state_root.join("linked.key"),
        ..bundle
    };
    symlink(&key_target, &linked_key.device_key_file).unwrap();
    update(&config, &linked_key);
    assert!(read_registration(&config, uid).is_err());
    linked_key.device_key_file = config.state_root.parent().unwrap().join("desk.key");
    update(&config, &linked_key);
    let outside = ProvisionConfig {
        state_root: config.state_root.join("nested"),
        registration_bundle: config.registration_bundle.clone(),
    };
    assert!(read_registration(&outside, uid).is_err());
    fs::write(
        &config.registration_bundle,
        vec![b' '; MAX_BUNDLE_BYTES + 1],
    )
    .unwrap();
    assert!(read_registration(&config, uid).is_err());
    assert!(!config.state_root.join(REMOTED_SUBDIR).exists());
}

#[tokio::test]
async fn unsupported_malformed_and_trailing_registration_artifacts_are_refused() {
    let (_temporary, config, mut bundle) = setup();
    let uid = current_uid().unwrap();
    let valid_grant = bundle.owner_grant_postcard.clone();
    bundle.version = 2;
    update(&config, &bundle);
    assert!(read_registration(&config, uid).is_err());
    bundle.version = 1;
    bundle.owner_grant_postcard.push_str("00");
    update(&config, &bundle);
    assert!(read_registration(&config, uid).is_err());
    bundle.owner_grant_postcard = valid_grant;
    bundle.robot_id = "short".into();
    update(&config, &bundle);
    assert!(read_registration(&config, uid).is_err());
    fs::write(&config.registration_bundle, b"{}").unwrap();
    assert!(read_registration(&config, uid).is_err());
}

#[tokio::test]
async fn symlinked_or_shared_state_roots_are_refused() {
    let (temporary, mut config, _bundle) = setup();
    let real = config.state_root.clone();
    let link = temporary.path().canonicalize().unwrap().join("link");
    symlink(&real, &link).unwrap();
    config.registration_bundle = link.join("registration.json");
    config.state_root = link;
    assert!(provision(&config, NOW).is_err());
    config.state_root = real.clone();
    config.registration_bundle = real.join("registration.json");
    fs::set_permissions(&real, fs::Permissions::from_mode(0o755)).unwrap();
    assert!(provision(&config, NOW).is_err());
    assert!(!real.join(REMOTED_SUBDIR).exists());
}

#[tokio::test]
async fn failed_state_steps_never_replace_files_or_publish_an_unclaimed_store() {
    let (temporary, config, _bundle) = setup();
    let registration = read_registration(&config, current_uid().unwrap()).unwrap();
    for blocked in [
        DEVICE_KEY_FILENAME,
        TRUST_STORE_MAC_KEY_FILENAME,
        CHASSIS_SECRET_FILENAME,
        DEVICE_INDEX_FILENAME,
        "device_index.json.tmp",
        TRUST_STORE_FILENAME,
        ".trust_store.provision.tmp",
    ] {
        let dir = temporary
            .path()
            .canonicalize()
            .unwrap()
            .join(blocked.replace('.', "_"));
        fs::DirBuilder::new().mode(0o700).create(&dir).unwrap();
        let canary = temporary
            .path()
            .canonicalize()
            .unwrap()
            .join(format!("canary-{}", blocked.replace('.', "_")));
        write_new(&canary, b"untouched").unwrap();
        symlink(&canary, dir.join(blocked)).unwrap();
        let store = claimed_store(&registration, &[0x81; 32], NOW).unwrap();
        assert!(write_state(&dir, &registration, &[0x81; 32], &[0x82; 32], store).is_err());
        assert_eq!(fs::read(&canary).unwrap(), b"untouched");
        assert_eq!(fs::read_dir(&dir).unwrap().count(), 1);
    }
}

#[test]
fn missing_runtime_is_reported_without_a_panic_or_robot_state_write() {
    let (_temporary, config, _bundle) = setup();
    assert!(matches!(
        super::super::provision(&config, NOW),
        Err(RemotedError::Provision(_))
    ));
    assert!(!config.state_root.join(REMOTED_SUBDIR).exists());
}
