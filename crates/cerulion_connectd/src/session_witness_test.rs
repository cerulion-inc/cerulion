// SPDX-License-Identifier: AGPL-3.0-only
//! Real private files and independent final-witness values.

use std::os::unix::fs::{symlink, DirBuilderExt, PermissionsExt};

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
        assert_eq!(path, directory.join("final-ojxwe33u/onsxg43jn5xdc.json"));
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
        assert_eq!(
            std::fs::metadata(path.parent().unwrap())
                .unwrap()
                .permissions()
                .mode()
                & 0o777,
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
    let other = store.path.join("final-mfxg65dimvza");
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(&other)
        .unwrap();
    symlink(&sentinel, other.join("onsxg43jn5xdc.json")).unwrap();
    assert!(store.persist("another", &witness(), &exit()).is_err());
    assert_eq!(std::fs::read(&sentinel).unwrap(), b"owned elsewhere");
    assert!(std::fs::symlink_metadata(other.join("onsxg43jn5xdc.json"))
        .unwrap()
        .file_type()
        .is_symlink());
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

#[test]
fn both_maximum_length_identifiers_fit_as_separate_private_components() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    let robot = "r".repeat(128);
    let mut value = witness();
    value.session_id = "s".repeat(128);
    let path = store.persist(&robot, &value, &exit()).unwrap();
    assert_eq!(path.parent().unwrap().file_name().unwrap().len(), 211);
    assert_eq!(path.file_name().unwrap().len(), 210);
    let saved: serde_json::Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
    assert_eq!(saved["robot_eid"], robot);
    assert_eq!(saved["session_id"], value.session_id);
    assert_eq!(saved["exit"]["code"], 23);
    assert!(store.persist(&robot, &value, &exit()).is_err());
}

#[test]
fn delimiter_ambiguous_pairs_are_distinct_and_keep_older_flat_files_untouched() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    let old = store.path.join("a-b-c.json");
    std::fs::write(&old, b"retained old file").unwrap();
    let mut first = witness();
    first.session_id = "c".into();
    let mut second = witness();
    second.session_id = "b-c".into();
    let a = store.persist("a-b", &first, &exit()).unwrap();
    let b = store.persist("a", &second, &exit()).unwrap();
    assert_eq!(a, store.path.join("final-mewwe/mm.json"));
    assert_eq!(b, store.path.join("final-me/miwwg.json"));
    assert_ne!(a, b);
    for (path, robot, session) in [(a, "a-b", "c"), (b, "a", "b-c")] {
        let value: serde_json::Value =
            serde_json::from_slice(&std::fs::read(path).unwrap()).unwrap();
        assert_eq!(value["robot_eid"], robot);
        assert_eq!(value["session_id"], session);
    }
    assert_eq!(std::fs::read(old).unwrap(), b"retained old file");
}

#[test]
fn robot_subdirectory_replacements_cannot_redirect_or_publicize_witnesses() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    let outside = parent.path().join("outside");
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(&outside)
        .unwrap();
    symlink(&outside, store.path.join("final-nruw423fmq")).unwrap();
    assert!(store.persist("linked", &witness(), &exit()).is_err());
    assert_eq!(std::fs::read_dir(&outside).unwrap().count(), 0);
    let public = store.path.join("final-ob2we3djmm");
    std::fs::create_dir(&public).unwrap();
    std::fs::set_permissions(&public, std::fs::Permissions::from_mode(0o755)).unwrap();
    assert!(store.persist("public", &witness(), &exit()).is_err());
    assert_eq!(std::fs::read_dir(&public).unwrap().count(), 0);
    let ordinary = store.path.join("final-n5zgi2lomfzhs");
    std::fs::write(&ordinary, b"ordinary file").unwrap();
    assert!(store.persist("ordinary", &witness(), &exit()).is_err());
    assert_eq!(std::fs::read(ordinary).unwrap(), b"ordinary file");
}

struct RestorePermissions(PathBuf, std::fs::Permissions);

impl Drop for RestorePermissions {
    fn drop(&mut self) {
        std::fs::set_permissions(&self.0, self.1.clone()).unwrap();
    }
}

fn run_permission_case_as_unprivileged_if_root() -> bool {
    use std::os::unix::process::CommandExt;
    if !rustix::process::geteuid().is_root() {
        return false;
    }
    // A root test process bypasses directory read permission. Run the same real
    // test as an unprivileged child, from a dedicated traversable temporary root.
    let fixture = tempfile::Builder::new()
        .prefix("cerulion-witness-permission-")
        .tempdir_in("/tmp")
        .unwrap();
    std::fs::set_permissions(fixture.path(), std::fs::Permissions::from_mode(0o755)).unwrap();
    let executable = fixture.path().join("permission-test");
    std::fs::copy(std::env::current_exe().unwrap(), &executable).unwrap();
    std::fs::set_permissions(&executable, std::fs::Permissions::from_mode(0o755)).unwrap();
    let child_tmp = fixture.path().join("child-tmp");
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(&child_tmp)
        .unwrap();
    rustix::fs::chown(
        &child_tmp,
        Some(rustix::process::Uid::from_raw(65534)),
        Some(rustix::process::Gid::from_raw(65534)),
    )
    .unwrap();
    let result = std::process::Command::new(executable)
        .args([
            "--exact",
            "session_witness::tests::search_only_ancestors_allow_durable_creation_but_an_unreadable_parent_refuses",
            "--nocapture",
        ])
        .env("TMPDIR", child_tmp)
        .uid(65534)
        .gid(65534)
        .output()
        .unwrap();
    let stdout = String::from_utf8(result.stdout).unwrap();
    assert!(
        result.status.success(),
        "{stdout}\n{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert!(stdout.contains("real unprivileged directory permissions verified"));
    assert!(stdout.contains("1 passed; 0 failed"));
    true
}

#[test]
fn search_only_ancestors_allow_durable_creation_but_an_unreadable_parent_refuses() {
    if run_permission_case_as_unprivileged_if_root() {
        return;
    }
    assert!(!rustix::process::geteuid().is_root());
    let temporary = tempfile::tempdir().unwrap();
    let ancestor = temporary.path().join("search-only");
    let parent = ancestor.join("prepared-parent");
    std::fs::create_dir(&ancestor).unwrap();
    std::fs::DirBuilder::new()
        .mode(0o700)
        .create(&parent)
        .unwrap();
    // The caller prepares and syncs the existing parent before handing it over.
    for path in [&parent, &ancestor, temporary.path()] {
        File::open(path).unwrap().sync_all().unwrap();
    }
    let _restore_ancestor = RestorePermissions(
        ancestor.clone(),
        std::fs::metadata(&ancestor).unwrap().permissions(),
    );
    std::fs::set_permissions(&ancestor, std::fs::Permissions::from_mode(0o111)).unwrap();
    assert_eq!(
        File::open(&ancestor).unwrap_err().kind(),
        io::ErrorKind::PermissionDenied
    );
    let path = parent.join("witnesses");
    let store = WitnessStore::open(&path).unwrap();
    let saved = store.persist("robot", &witness(), &exit()).unwrap();
    assert_eq!(saved, path.join("final-ojxwe33u/onsxg43jn5xdc.json"));
    let expected = std::fs::read(&saved).unwrap();
    let reopened = WitnessStore::open(&path).unwrap();
    assert!(reopened.persist("robot", &witness(), &exit()).is_err());
    assert_eq!(std::fs::read(saved).unwrap(), expected);
    let _restore_parent = RestorePermissions(
        parent.clone(),
        std::fs::metadata(&parent).unwrap().permissions(),
    );
    std::fs::set_permissions(&parent, std::fs::Permissions::from_mode(0o300)).unwrap();
    assert_eq!(
        File::open(&parent).unwrap_err().kind(),
        io::ErrorKind::PermissionDenied
    );
    let unavailable = parent.join("cannot-sync-parent");
    assert!(WitnessStore::open(&unavailable).is_err());
    assert_eq!(
        std::fs::metadata(unavailable).unwrap_err().kind(),
        io::ErrorKind::NotFound
    );
    println!("real unprivileged directory permissions verified");
}

#[test]
fn identifier_encoding_matches_independent_rfc4648_vectors() {
    for (input, expected) in [
        ("", ""),
        ("f", "my"),
        ("fo", "mzxq"),
        ("foo", "mzxw6"),
        ("foob", "mzxw6yq"),
        ("fooba", "mzxw6ytb"),
        ("foobar", "mzxw6ytboi"),
    ] {
        assert_eq!(encoded_id(input), expected);
    }
}

#[test]
fn case_distinct_identifiers_remain_distinct_on_the_real_filesystem() {
    let parent = tempfile::tempdir().unwrap();
    let store = WitnessStore::open(&parent.path().join("witnesses")).unwrap();
    let mut paths = std::collections::BTreeSet::new();
    for (robot, session) in [("a", "a"), ("A", "a"), ("a", "A"), ("A", "A")] {
        let mut value = witness();
        value.session_id = session.into();
        let saved = store.persist(robot, &value, &exit()).unwrap();
        assert!(paths.insert(saved.canonicalize().unwrap()));
        let stored: serde_json::Value =
            serde_json::from_slice(&std::fs::read(saved).unwrap()).unwrap();
        assert_eq!(stored["robot_eid"], robot);
        assert_eq!(stored["session_id"], session);
    }
    assert_eq!(paths.len(), 4);
}
