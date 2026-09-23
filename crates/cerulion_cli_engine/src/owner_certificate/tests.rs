// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use cerulion_pairing::format::{
    AccountId, DeviceCert, IntermediateCert, PrincipalKind, Scope, Validity, FORMAT_VERSION,
};
use ed25519_dalek::SigningKey;

const ACCOUNT: [u8; 32] = [0x11; 32];
const DESK_SEED: [u8; 32] = [0x33; 32];

struct Fixture {
    dir: tempfile::TempDir,
    cert: SignedDeviceCert,
    intermediate: SignedIntermediateCert,
}

fn encode(value: &impl serde::Serialize) -> String {
    URL_SAFE_NO_PAD.encode(postcard::to_stdvec(value).unwrap())
}

impl Fixture {
    fn new() -> Self {
        let dir = tempfile::tempdir().unwrap();
        let issuer = SigningKey::from_bytes(&[0x22; 32]);
        let root = SigningKey::from_bytes(&[0x44; 32]);
        let desk = SigningKey::from_bytes(&DESK_SEED);
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
        let cert = DeviceCert {
            version: FORMAT_VERSION,
            device_key: PublicKey(desk.verifying_key().to_bytes()),
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
            cert,
            intermediate,
        };
        fixture.write_account(ACCOUNT);
        std::fs::write(fixture.path("desk.key"), DESK_SEED).unwrap();
        fixture.write_leaf();
        fixture.write_chain();
        fixture
    }

    fn path(&self, name: &str) -> PathBuf {
        self.dir.path().join(name)
    }

    fn write_account(&self, account: [u8; 32]) {
        auth::write_to(
            &self.path("auth.json"),
            &auth::AuthState {
                account_id: URL_SAFE_NO_PAD.encode(account),
                session_token: "unit-session".into(),
                refresh_token: "unit-refresh".into(),
                expires_at_ns: 100,
                logged_in_ever: true,
                role: None,
            },
        )
        .unwrap();
    }

    fn write_leaf(&self) {
        std::fs::write(self.path("device.cert"), encode(&self.cert)).unwrap();
    }

    fn write_chain(&self) {
        stage_at(
            &self.path("device-chain.json"),
            &encode(&self.cert),
            &encode(&self.intermediate),
        )
        .unwrap()
        .commit()
        .unwrap();
    }

    fn refuse(&self, expected: &str) {
        let error = load_at(self.dir.path()).unwrap_err().to_string();
        assert!(error.contains(expected), "{error}");
        assert!(error.contains("cerulion login"), "{error}");
    }
}

#[test]
fn current_login_chain_preserves_its_signed_fields_and_deterministic_wire() {
    let fixture = Fixture::new();
    let first = load_at(fixture.dir.path()).unwrap();
    let second = load_at(fixture.dir.path()).unwrap();
    assert_eq!(first.version, 1);
    assert_eq!(first.device_cert.cert.account.0, [0x11; 32]);
    assert_eq!(first.device_cert.cert.principal_kind, PrincipalKind::Human);
    assert_eq!(first.device_cert.cert.validity.not_after_ns, 100);
    assert_eq!(first.device_cert, fixture.cert);
    assert_eq!(first.intermediate, fixture.intermediate);
    assert_eq!(first.to_postcard().unwrap(), second.to_postcard().unwrap());
}

#[test]
fn account_switch_and_foreign_device_key_refuse_the_old_chain() {
    let fixture = Fixture::new();
    fixture.write_account([0x55; 32]);
    fixture.refuse("different login account");
    fixture.write_account(ACCOUNT);
    std::fs::write(fixture.path("desk.key"), [0x66; 32]).unwrap();
    fixture.refuse("different device key");
}

#[test]
fn a_rotated_leaf_is_unavailable_until_its_matching_chain_commits() {
    let mut fixture = Fixture::new();
    let old = std::fs::read(fixture.path("device-chain.json")).unwrap();
    fixture.cert.cert.issued_at_ns = 11;
    fixture.cert = fixture
        .cert
        .cert
        .clone()
        .sign(&SigningKey::from_bytes(&[0x22; 32]));
    let staged = stage_at(
        &fixture.path("device-chain.json"),
        &encode(&fixture.cert),
        &encode(&fixture.intermediate),
    )
    .unwrap();
    assert_eq!(
        std::fs::read(fixture.path("device-chain.json")).unwrap(),
        old
    );
    assert!(load_at(fixture.dir.path()).is_ok());
    fixture.write_leaf();
    fixture.refuse("does not match the current certificate");
    staged.commit().unwrap();
    assert_eq!(
        load_at(fixture.dir.path())
            .unwrap()
            .device_cert
            .cert
            .issued_at_ns,
        11
    );
}

#[test]
fn identity_only_or_signed_out_state_never_reuses_a_cached_chain() {
    let fixture = Fixture::new();
    std::fs::remove_file(fixture.path("device.cert")).unwrap();
    fixture.refuse("current device certificate is unavailable");
    fixture.write_leaf();
    std::fs::remove_file(fixture.path("auth.json")).unwrap();
    fixture.refuse("no current login is stored");
    assert!(fixture.path("device-chain.json").exists());
}

#[test]
fn an_unrelated_intermediate_cannot_accompany_the_current_leaf() {
    let mut fixture = Fixture::new();
    fixture.intermediate.cert.intermediate_key = PublicKey([0x77; 32]);
    fixture.write_chain();
    fixture.refuse("different issuers");
}

#[test]
fn absent_corrupt_and_oversized_chain_files_fail_closed() {
    let fixture = Fixture::new();
    let path = fixture.path("device-chain.json");
    std::fs::remove_file(&path).unwrap();
    fixture.refuse("no readable complete device certificate chain");
    std::fs::write(&path, "{").unwrap();
    fixture.refuse("cache is malformed");
    std::fs::write(&path, vec![b' '; MAX_CACHE_BYTES + 1]).unwrap();
    fixture.refuse("no readable complete device certificate chain");
    std::fs::remove_file(&path).unwrap();
    std::fs::create_dir(path).unwrap();
    fixture.refuse("no readable complete device certificate chain");
}

#[test]
fn malformed_and_oversized_device_keys_are_never_recreated() {
    let fixture = Fixture::new();
    let path = fixture.path("desk.key");
    for size in [0, 31, 33] {
        std::fs::write(&path, vec![0; size]).unwrap();
        fixture.refuse("device key");
        assert_eq!(std::fs::read(&path).unwrap().len(), size);
    }
    std::fs::remove_file(&path).unwrap();
    fixture.refuse("device key");
    assert!(!path.exists());
}

#[test]
fn malformed_certificate_encodings_and_trailing_postcard_bytes_are_refused() {
    let fixture = Fixture::new();
    for invalid_cert in ["%".to_string(), URL_SAFE_NO_PAD.encode([0xff])] {
        std::fs::write(fixture.path("device.cert"), &invalid_cert).unwrap();
        stage_at(
            &fixture.path("device-chain.json"),
            &invalid_cert,
            &encode(&fixture.intermediate),
        )
        .unwrap()
        .commit()
        .unwrap();
        fixture.refuse("device certificate could not be decoded");
    }
    fixture.write_leaf();
    let mut extra = postcard::to_stdvec(&fixture.intermediate).unwrap();
    extra.push(0);
    stage_at(
        &fixture.path("device-chain.json"),
        &encode(&fixture.cert),
        &URL_SAFE_NO_PAD.encode(extra),
    )
    .unwrap()
    .commit()
    .unwrap();
    fixture.refuse("intermediate certificate could not be decoded");
}

#[test]
fn abandoned_and_oversized_staging_keep_the_previous_bytes() {
    let fixture = Fixture::new();
    let path = fixture.path("device-chain.json");
    let before = std::fs::read(&path).unwrap();
    drop(stage_at(&path, "next-leaf", "next-issuer").unwrap());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    assert!(stage_at(&path, &"x".repeat(MAX_CACHE_BYTES), "issuer").is_err());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );
    }
}

#[cfg(unix)]
#[test]
fn login_writer_excludes_std_shared_snapshot_lock_on_the_same_sibling() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("auth.json");
    let reader = auth::with_store_lock(&path, || {
        let reader = std::fs::File::open(auth::store_lock_path(&path))?;
        assert!(matches!(
            reader.try_lock_shared(),
            Err(std::fs::TryLockError::WouldBlock)
        ));
        Ok(reader)
    })
    .unwrap();
    reader.try_lock_shared().unwrap();
    assert_eq!(auth::STORE_LOCK_FILE, ".studio-auth.lock");
}

#[cfg(unix)]
fn optional(fixture: &Fixture) -> CliResult<Option<OwnerCertificatePresentationWire>> {
    let auth_path = fixture.path("auth.json");
    let key = PublicKey(
        SigningKey::from_bytes(&DESK_SEED)
            .verifying_key()
            .to_bytes(),
    );
    auth::with_store_lock(&auth_path, || {
        Ok(load_optional_locked(
            fixture.dir.path(),
            &auth_path,
            AccountId(ACCOUNT),
            key,
        ))
    })
    .unwrap()
}

#[cfg(unix)]
#[test]
fn optional_proof_accepts_absence_valid_legacy_leaf_and_expired_complete_chain() {
    let fixture = Fixture::new();
    let proof = optional(&fixture).unwrap().unwrap();
    assert_eq!(proof.device_cert.cert.validity.not_after_ns, 100);
    assert_eq!(proof.device_cert, fixture.cert);
    std::fs::remove_file(fixture.path("device-chain.json")).unwrap();
    assert!(optional(&fixture).unwrap().is_none());
    std::fs::remove_file(fixture.path("device.cert")).unwrap();
    assert!(optional(&fixture).unwrap().is_none());
}

#[cfg(unix)]
#[test]
fn optional_proof_never_treats_partial_corrupt_or_dangling_material_as_absent() {
    let mut fixture = Fixture::new();
    std::fs::remove_file(fixture.path("device.cert")).unwrap();
    assert!(optional(&fixture)
        .unwrap_err()
        .to_string()
        .contains("without its leaf"));
    std::fs::remove_file(fixture.path("device-chain.json")).unwrap();
    std::os::unix::fs::symlink("missing-leaf", fixture.path("device.cert")).unwrap();
    assert!(optional(&fixture).is_err());
    std::fs::remove_file(fixture.path("device.cert")).unwrap();
    fixture.write_leaf();
    std::os::unix::fs::symlink("missing-chain", fixture.path("device-chain.json")).unwrap();
    assert!(optional(&fixture).is_err());
    std::fs::remove_file(fixture.path("device-chain.json")).unwrap();
    std::fs::write(fixture.path("device.cert"), "private-malformed-sentinel").unwrap();
    let error = optional(&fixture).unwrap_err().to_string();
    assert!(!error.contains("private-malformed-sentinel"));
    fixture.cert.cert.account = AccountId([0x55; 32]);
    fixture.write_leaf();
    assert!(optional(&fixture)
        .unwrap_err()
        .to_string()
        .contains("different login account"));
    fixture.cert.cert.account = AccountId(ACCOUNT);
    fixture.cert.cert.device_key = PublicKey([0x77; 32]);
    fixture.write_leaf();
    assert!(optional(&fixture)
        .unwrap_err()
        .to_string()
        .contains("different device key"));
}

#[cfg(unix)]
#[test]
fn optional_and_complete_proofs_reject_invalid_format_intervals_and_unsigned_issuers() {
    let mut fixture = Fixture::new();
    fixture.cert.cert.version = FORMAT_VERSION + 1;
    fixture.write_leaf();
    fixture.write_chain();
    assert!(optional(&fixture).is_err());
    fixture.refuse("invalid format or validity interval");
    fixture.cert.cert.version = FORMAT_VERSION;
    fixture.cert.cert.validity.not_after_ns = fixture.cert.cert.validity.not_before_ns;
    fixture.write_leaf();
    std::fs::remove_file(fixture.path("device-chain.json")).unwrap();
    assert!(optional(&fixture).is_err());
    let mut fixture = Fixture::new();
    fixture.intermediate.signatures.clear();
    fixture.write_chain();
    assert!(optional(&fixture).is_err());
    fixture.refuse("intermediate certificate has an invalid format");
}
