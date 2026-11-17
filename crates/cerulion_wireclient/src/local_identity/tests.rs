// SPDX-License-Identifier: AGPL-3.0-only
use super::*;
use std::cell::RefCell;

const ACCOUNT: &str = "00112233-4455-4677-8899-aabbccddeeff";
const SEED: [u8; 32] = [0x33; 32];

fn fixture() -> tempfile::TempDir {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("auth.lock"), []).unwrap();
    std::fs::write(dir.path().join("desk.key"), SEED).unwrap();
    std::fs::write(
        dir.path().join("auth.json"),
        br#"{"account_id":"00112233-4455-4677-8899-aabbccddeeff","session_token":"expired","refresh_token":"refresh","expires_at_ns":0,"logged_in_ever":true}"#,
    )
    .unwrap();
    dir
}

#[test]
fn identity_only_never_opens_present_invalid_proof_and_returns_the_actual_key() {
    let dir = fixture();
    std::fs::write(dir.path().join("device.cert"), b"invalid leaf").unwrap();
    std::fs::write(dir.path().join("device-chain.json"), b"invalid chain").unwrap();
    let opened = RefCell::new(Vec::new());
    // All files are owned regular fixtures; observe real opens, not substituted reads.
    let open = |path: &Path| {
        opened
            .borrow_mut()
            .push(path.file_name().unwrap().to_owned());
        File::open(path)
    };
    let identity = load_identity(dir.path(), &dir.path().join("auth.lock"), &open).unwrap();
    assert_eq!(identity.auth_account_id(), ACCOUNT);
    assert_eq!(
        identity.account().0,
        [
            0x02, 0x1f, 0x0a, 0x00, 0x68, 0x66, 0xfd, 0xe5, 0x25, 0x6f, 0x57, 0xcf, 0xad, 0x02,
            0xa0, 0x22, 0xfa, 0xe9, 0xe8, 0xe4, 0x73, 0xa6, 0xe4, 0xd3, 0xa4, 0x71, 0xf1, 0xf4,
            0xbd, 0x83, 0xe9, 0x17,
        ]
    );
    assert_eq!(identity.seed(), &SEED);
    assert_eq!(
        identity.device_key(),
        DeviceIdentity::from_seed(&SEED).public_key()
    );
    assert_eq!(
        *opened.borrow(),
        ["auth.lock", "auth.json", "desk.key"].map(std::ffi::OsString::from)
    );
    assert_eq!(
        load_with_proof(dir.path(), &dir.path().join("auth.lock"), &open).err(),
        Some(SnapshotError::InvalidCertificate)
    );
    let lock = File::open(dir.path().join("auth.lock")).unwrap();
    lock.try_lock().unwrap();
    assert_eq!(
        load_identity(dir.path(), &dir.path().join("auth.lock"), &open).err(),
        Some(SnapshotError::Busy)
    );
}

#[test]
fn explicit_lock_path_and_identity_changes_are_observed_without_creating_state() {
    let dir = fixture();
    let missing_lock = dir.path().join("missing.lock");
    assert_eq!(
        load_identity(dir.path(), &missing_lock, &|path| File::open(path)).err(),
        Some(SnapshotError::Lock(LoginProblem::Missing))
    );
    assert!(!missing_lock.exists());
    let lock = dir.path().join("auth.lock");
    std::fs::write(dir.path().join("desk.key"), [0x44; 32]).unwrap();
    let changed = load_identity(dir.path(), &lock, &|path| File::open(path)).unwrap();
    assert_eq!(changed.seed(), &[0x44; 32]);
    assert_ne!(
        changed.device_key(),
        DeviceIdentity::from_seed(&SEED).public_key()
    );
    std::fs::remove_file(dir.path().join("auth.json")).unwrap();
    assert_eq!(
        load_identity(dir.path(), &lock, &|path| File::open(path)).err(),
        Some(SnapshotError::Auth(LoginProblem::Missing))
    );
    assert!(!dir.path().join("auth.json").exists());
}
