// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use base64::{engine::general_purpose::URL_SAFE_NO_PAD, Engine};
use cerulion_pairing::format::{
    DeviceCert, IntermediateCert, PrincipalKind, Scope, SignedDeviceCert, SignedIntermediateCert,
    Validity, FORMAT_VERSION,
};
use ed25519_dalek::SigningKey;

const UUID: &str = "00112233-4455-4677-8899-aabbccddeeff";
const ACCOUNT: [u8; 32] = [
    0x02, 0x1f, 0x0a, 0x00, 0x68, 0x66, 0xfd, 0xe5, 0x25, 0x6f, 0x57, 0xcf, 0xad, 0x02, 0xa0, 0x22,
    0xfa, 0xe9, 0xe8, 0xe4, 0x73, 0xa6, 0xe4, 0xd3, 0xa4, 0x71, 0xf1, 0xf4, 0xbd, 0x83, 0xe9, 0x17,
];
const SEED: [u8; 32] = [0x33; 32];

struct Fixture {
    dir: tempfile::TempDir,
    leaf: SignedDeviceCert,
    issuer: SignedIntermediateCert,
}

fn encode(value: &impl serde::Serialize) -> String {
    URL_SAFE_NO_PAD.encode(postcard::to_stdvec(value).unwrap())
}

impl Fixture {
    fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let root = SigningKey::from_bytes(&[0x11; 32]);
        let issuer = SigningKey::from_bytes(&[0x22; 32]);
        let device = SigningKey::from_bytes(&SEED);
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
            device_key: PublicKey(device.verifying_key().to_bytes()),
            account: AccountId(ACCOUNT),
            principal_kind: PrincipalKind::Human,
            scope: Scope::OWNER_FULL,
            validity,
            issued_at_ns: 10,
            issuer_key: PublicKey(issuer.verifying_key().to_bytes()),
        }
        .sign(&issuer);
        let fixture = Self {
            dir,
            leaf,
            issuer: intermediate,
        };
        std::fs::write(fixture.path(STORE_LOCK_FILE), []).unwrap();
        std::fs::write(fixture.path("desk.key"), SEED).unwrap();
        fixture.auth(UUID, "expired", 1);
        fixture.certificates();
        fixture
    }
    fn path(&self, file: &str) -> std::path::PathBuf {
        self.dir.path().join(file)
    }
    fn auth(&self, account: &str, token: &str, expires: u64) {
        std::fs::write(
            self.path("auth.json"),
            serde_json::to_vec(&serde_json::json!({
                "account_id": account, "session_token": token, "refresh_token": "refresh",
                "expires_at_ns": expires, "logged_in_ever": true,
            }))
            .unwrap(),
        )
        .unwrap();
    }
    fn certificates(&self) {
        std::fs::write(self.path("device.cert"), encode(&self.leaf)).unwrap();
        self.cache(&encode(&self.leaf), &encode(&self.issuer));
    }
    fn cache(&self, leaf: &str, issuer: &str) {
        std::fs::write(
            self.path("device-chain.json"),
            serde_json::to_vec(&serde_json::json!({
                "device_cert": leaf, "intermediate": issuer,
            }))
            .unwrap(),
        )
        .unwrap();
    }
    fn load(&self) -> IdentitySnapshot {
        load_at(self.dir.path()).unwrap()
    }
    fn refuse(&self, expected: SnapshotError) {
        assert_eq!(load_at(self.dir.path()).err().unwrap(), expected);
    }
}

#[test]
fn snapshot_pins_account_key_and_public_proof_without_expiry_or_token_invalidation() {
    let fixture = Fixture::new();
    let first = fixture.load();
    assert_eq!(first.auth_account_id(), UUID);
    assert_eq!(first.account(), AccountId(ACCOUNT));
    let registry = first
        .wan_registry(
            Default::default(),
            cerulion_link::RelayConfig::Disabled,
            None,
        )
        .unwrap();
    assert_eq!(registry.account(), Some(ACCOUNT));
    assert_eq!(registry.robot_count().unwrap(), 0);
    assert_eq!(first.device_key(), fixture.leaf.cert.device_key);
    assert_eq!(
        first
            .owner_chain()
            .unwrap()
            .device_cert
            .cert
            .validity
            .not_after_ns,
        100
    );
    let bytes = first.owner_chain_wire().unwrap();
    let decoded = OwnerCertificatePresentationWire::from_postcard(bytes).unwrap();
    assert_eq!(decoded.device_cert, fixture.leaf);
    assert_eq!(decoded.intermediate, fixture.issuer);
    assert!(first.stamp() == fixture.load().stamp());
    fixture.auth(UUID, "rotated-session", u64::MAX);
    let refreshed = fixture.load();
    assert!(first.stamp() == refreshed.stamp());
    assert_eq!(Some(bytes), refreshed.owner_chain_wire());
}

#[test]
fn shared_lock_refuses_active_writer_and_releases_after_every_read() {
    let fixture = Fixture::new();
    let writer = OpenOptions::new()
        .read(true)
        .write(true)
        .open(fixture.path(STORE_LOCK_FILE))
        .unwrap();
    writer.try_lock().unwrap();
    fixture.refuse(SnapshotError::Busy);
    drop(writer);
    let reader = File::open(fixture.path(STORE_LOCK_FILE)).unwrap();
    reader.try_lock_shared().unwrap();
    let _snapshot = fixture.load();
    drop(reader);
    let writer = File::open(fixture.path(STORE_LOCK_FILE)).unwrap();
    writer.try_lock().unwrap();
    fixture.refuse(SnapshotError::Busy);
}

#[test]
fn missing_or_partial_login_never_creates_a_lock_key_or_proof() {
    let empty = tempfile::tempdir().unwrap();
    assert_eq!(
        load_at(empty.path()).err().unwrap(),
        SnapshotError::Lock(LoginProblem::Missing)
    );
    assert_eq!(std::fs::read_dir(empty.path()).unwrap().count(), 0);
    let fixture = Fixture::new();
    std::fs::remove_file(fixture.path("device-chain.json")).unwrap();
    assert!(fixture.load().owner_chain().is_none());
    std::fs::remove_file(fixture.path("device.cert")).unwrap();
    assert!(fixture.load().owner_chain().is_none());
    fixture.cache(&encode(&fixture.leaf), &encode(&fixture.issuer));
    fixture.refuse(SnapshotError::IncompleteChain);
    fixture.certificates();
    std::fs::remove_file(fixture.path("desk.key")).unwrap();
    fixture.refuse(SnapshotError::Key(LoginProblem::Missing));
    assert!(!fixture.path("desk.key").exists());
    std::fs::write(fixture.path("desk.key"), SEED).unwrap();
    std::fs::remove_file(fixture.path("auth.json")).unwrap();
    fixture.refuse(SnapshotError::Auth(LoginProblem::Missing));
}

#[test]
fn account_key_leaf_and_issuer_mismatches_refuse_instead_of_dropping_the_proof() {
    let fixture = Fixture::new();
    fixture.auth(&URL_SAFE_NO_PAD.encode([0x55; 32]), "expired", 1);
    fixture.refuse(SnapshotError::InvalidCertificate);
    fixture.auth(UUID, "expired", 1);
    std::fs::write(fixture.path("desk.key"), [0x66; 32]).unwrap();
    fixture.refuse(SnapshotError::InvalidCertificate);
    std::fs::write(fixture.path("desk.key"), SEED).unwrap();
    fixture.cache("another-leaf", &encode(&fixture.issuer));
    fixture.refuse(SnapshotError::InvalidCertificate);
    let mut issuer = fixture.issuer.clone();
    issuer.cert.intermediate_key = PublicKey([0x77; 32]);
    fixture.cache(&encode(&fixture.leaf), &encode(&issuer));
    fixture.refuse(SnapshotError::InvalidCertificate);
    fixture.certificates();
    assert_eq!(fixture.load().account(), AccountId(ACCOUNT));
}

#[test]
fn malformed_present_material_and_oversized_files_fail_without_writing() {
    let fixture = Fixture::new();
    for bytes in [vec![0xff], b"%".to_vec(), Vec::new()] {
        std::fs::write(fixture.path("device.cert"), &bytes).unwrap();
        fixture.refuse(SnapshotError::InvalidCertificate);
        assert_eq!(std::fs::read(fixture.path("device.cert")).unwrap(), bytes);
    }
    fixture.certificates();
    let mut extra = postcard::to_stdvec(&fixture.issuer).unwrap();
    extra.push(0);
    fixture.cache(&encode(&fixture.leaf), &URL_SAFE_NO_PAD.encode(extra));
    fixture.refuse(SnapshotError::InvalidCertificate);
    fixture.certificates();
    std::fs::write(fixture.path("device-chain.json"), "{").unwrap();
    fixture.refuse(SnapshotError::InvalidCertificate);
    std::fs::write(
        fixture.path("device-chain.json"),
        vec![b' '; MAX_CERT_BYTES as usize + 1],
    )
    .unwrap();
    fixture.refuse(SnapshotError::Chain(LoginProblem::TooLarge));
    fixture.certificates();
    std::fs::write(fixture.path("desk.key"), [0; 31]).unwrap();
    fixture.refuse(SnapshotError::KeySize);
    std::fs::write(fixture.path("desk.key"), [0; 33]).unwrap();
    fixture.refuse(SnapshotError::Key(LoginProblem::TooLarge));
}

#[test]
fn valid_proof_refresh_changes_stamp_without_changing_account_or_key() {
    let mut fixture = Fixture::new();
    let before = fixture.load();
    fixture.leaf.cert.issued_at_ns = 11;
    fixture.leaf = fixture
        .leaf
        .cert
        .clone()
        .sign(&SigningKey::from_bytes(&[0x22; 32]));
    fixture.certificates();
    let after = fixture.load();
    assert!(before.stamp() != after.stamp());
    assert_eq!(before.account(), after.account());
    assert_eq!(before.device_key(), after.device_key());
    assert_ne!(before.owner_chain_wire(), after.owner_chain_wire());
}

#[test]
fn malformed_identity_and_nonregular_lock_or_key_are_typed_refusals() {
    let fixture = Fixture::new();
    for account in ["", " ", "opaque-account"] {
        fixture.auth(account, "hidden-token", 1);
        let expected = if account.trim().is_empty() {
            SnapshotError::Auth(LoginProblem::Malformed)
        } else {
            SnapshotError::Account
        };
        fixture.refuse(expected);
        assert!(!format!("{expected:?}").contains("hidden-token"));
    }
    fixture.auth(UUID, "expired", 1);
    std::fs::remove_file(fixture.path(STORE_LOCK_FILE)).unwrap();
    std::fs::create_dir(fixture.path(STORE_LOCK_FILE)).unwrap();
    fixture.refuse(SnapshotError::Lock(LoginProblem::NotRegular));
    std::fs::remove_dir(fixture.path(STORE_LOCK_FILE)).unwrap();
    std::fs::write(fixture.path(STORE_LOCK_FILE), []).unwrap();
    std::fs::remove_file(fixture.path("desk.key")).unwrap();
    std::fs::create_dir(fixture.path("desk.key")).unwrap();
    fixture.refuse(SnapshotError::Key(LoginProblem::NotRegular));
}

#[cfg(unix)]
#[test]
fn a_present_dangling_certificate_symlink_is_not_an_absent_legacy_proof() {
    let fixture = Fixture::new();
    std::fs::remove_file(fixture.path("device.cert")).unwrap();
    std::fs::remove_file(fixture.path("device-chain.json")).unwrap();
    std::os::unix::fs::symlink(fixture.path("missing"), fixture.path("device.cert")).unwrap();
    fixture.refuse(SnapshotError::Certificate(LoginProblem::Unreadable));
}

#[test]
fn failed_snapshot_releases_shared_lock_and_legacy_base64_preserves_account_bytes() {
    let fixture = Fixture::new();
    fixture.auth(&URL_SAFE_NO_PAD.encode(ACCOUNT), "expired", 1);
    assert_eq!(fixture.load().account(), AccountId(ACCOUNT));
    std::fs::write(fixture.path("device-chain.json"), "invalid").unwrap();
    fixture.refuse(SnapshotError::InvalidCertificate);
    let writer = File::open(fixture.path(STORE_LOCK_FILE)).unwrap();
    writer.try_lock().unwrap();
    fixture.refuse(SnapshotError::Busy);
}
