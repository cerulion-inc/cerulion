// SPDX-License-Identifier: AGPL-3.0-only
//! Real private files and independent final-witness values.

use std::os::unix::fs::{symlink, PermissionsExt};

use cerulion_ops_proto::ExitReason;

use super::*;

fn witness() -> RecordingWitness {
    RecordingWitness {
        session_id: "session1".into(),
        head: "a".repeat(64),
        events: 4,
    }
}

fn exit() -> SessionExit {
    SessionExit {
        code: Some(23),
        signal: None,
        reason: ExitReason::Exited,
        recording_head: "a".repeat(64),
        events: 4,
        stdout_truncated: false,
        stderr_truncated: false,
    }
}

#[test]
fn final_files_match_the_literal_witness_and_keep_private_permissions() {
    let expected: serde_json::Value = serde_json::from_str(
        r#"{"robot_eid":"robot","session_id":"session1","head":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","events":4,"exit":{"code":23,"signal":null,"reason":"exited","recording_head":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","events":4,"stdout_truncated":false,"stderr_truncated":false}}"#,
    )
    .unwrap();
    let mut previous_bytes = None;
    for _ in 0..2 {
        let parent = tempfile::tempdir().unwrap();
        let directory = parent.path().join("witnesses");
        let store = WitnessStore::open(&directory).unwrap();
        let path = store.persist("robot", &witness(), &exit()).unwrap();
        assert_eq!(path, directory.join("robot-session1.json"));
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(
            serde_json::from_slice::<serde_json::Value>(&bytes).unwrap(),
            expected
        );
        assert!(bytes.ends_with(b"\n"));
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o600
        );
        assert_eq!(
            std::fs::metadata(&directory).unwrap().permissions().mode() & 0o777,
            0o700
        );
        if let Some(previous) = &previous_bytes {
            assert_eq!(&bytes, previous);
        }
        previous_bytes = Some(bytes);
    }
}

#[test]
fn an_existing_final_witness_or_symlink_is_never_overwritten() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    let path = store.persist("robot", &witness(), &exit()).unwrap();
    let before = std::fs::read(&path).unwrap();
    let mut later = exit();
    later.code = Some(0);
    assert!(store.persist("robot", &witness(), &later).is_err());
    assert_eq!(std::fs::read(&path).unwrap(), before);
    let sentinel = parent.path().join("sentinel");
    std::fs::write(&sentinel, b"owned elsewhere").unwrap();
    symlink(&sentinel, store.path.join("another-session1.json")).unwrap();
    assert!(store.persist("another", &witness(), &exit()).is_err());
    assert_eq!(std::fs::read(&sentinel).unwrap(), b"owned elsewhere");
    assert!(
        std::fs::symlink_metadata(store.path.join("another-session1.json"))
            .unwrap()
            .file_type()
            .is_symlink()
    );
}

#[test]
fn private_existing_roots_reopen_but_symlinks_files_and_public_roots_refuse() {
    let parent = tempfile::tempdir().unwrap();
    let path = parent.path().join("witnesses");
    let _first = WitnessStore::open(&path).unwrap();
    let _second = WitnessStore::open(&path).unwrap();
    let link = parent.path().join("link");
    symlink(&path, &link).unwrap();
    assert!(WitnessStore::open(&link).is_err());
    let ordinary = parent.path().join("ordinary");
    std::fs::write(&ordinary, b"ordinary file").unwrap();
    assert!(WitnessStore::open(&ordinary).is_err());
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
    assert!(WitnessStore::open(&path).is_err());
    assert!(WitnessStore::open(&parent.path().join("missing/child")).is_err());
}

#[test]
fn unsafe_filename_components_are_refused_before_creating_an_entry() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    for invalid in [
        "",
        ".",
        "..",
        "../escape",
        "nested/name",
        "/absolute",
        "nul\0id",
        "é",
    ] {
        let mut changed = witness();
        changed.session_id = invalid.into();
        assert!(store.persist("robot", &changed, &exit()).is_err());
        assert!(store.persist(invalid, &witness(), &exit()).is_err());
        assert_eq!(std::fs::read_dir(&store.path).unwrap().count(), 0);
    }
    let mut changed = witness();
    changed.session_id = "s".repeat(129);
    assert!(store.persist("robot", &changed, &exit()).is_err());
    assert!(store
        .persist(&"r".repeat(129), &witness(), &exit())
        .is_err());
    assert_eq!(std::fs::read_dir(&store.path).unwrap().count(), 0);
    changed.session_id = "s".repeat(128);
    assert!(store.persist("robot", &changed, &exit()).is_ok());
    assert!(store.persist(&"r".repeat(128), &witness(), &exit()).is_ok());
}

#[test]
fn contradictory_or_malformed_exit_commitments_make_no_file() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    let mut wrong_head = witness();
    wrong_head.head = "b".repeat(64);
    let mut wrong_count = witness();
    wrong_count.events = 3;
    assert!(store.persist("robot", &wrong_head, &exit()).is_err());
    assert!(store.persist("robot", &wrong_count, &exit()).is_err());
    for invalid_hash in ["bad".into(), "A".repeat(64), "g".repeat(64), "a".repeat(63)] {
        let mut changed_witness = witness();
        changed_witness.head.clone_from(&invalid_hash);
        let mut changed_exit = exit();
        changed_exit.recording_head = invalid_hash;
        assert!(store
            .persist("robot", &changed_witness, &changed_exit)
            .is_err());
    }
    for count in [0, 1] {
        let mut changed_witness = witness();
        changed_witness.events = count;
        let mut changed_exit = exit();
        changed_exit.events = count;
        assert!(store
            .persist("robot", &changed_witness, &changed_exit)
            .is_err());
    }
    let mut changed_exit = exit();
    changed_exit.signal = Some(15);
    assert!(store.persist("robot", &witness(), &changed_exit).is_err());
    assert_eq!(std::fs::read_dir(&store.path).unwrap().count(), 0);
    assert!(store.persist("robot", &witness(), &exit()).is_ok());
}
