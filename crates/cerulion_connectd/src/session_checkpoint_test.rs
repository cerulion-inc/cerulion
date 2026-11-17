// SPDX-License-Identifier: AGPL-3.0-only

use std::os::unix::fs::{symlink, PermissionsExt};

use super::*;

fn witness(events: u64) -> RecordingWitness {
    RecordingWitness {
        session_id: "reused-peer-id".into(),
        head: "a".repeat(64),
        events,
    }
}

async fn finish(writer: &mut CheckpointWriter) -> SavedCheckpoint {
    writer.finish();
    tokio::time::timeout(Duration::from_secs(5), async {
        let mut latest = None;
        while !writer.is_done() {
            if let Some(saved) = writer.progress().await.unwrap() {
                latest = Some(saved);
            }
        }
        latest.unwrap()
    })
    .await
    .expect("checkpoint writer must finish")
}

#[tokio::test]
async fn immediate_finish_retains_latest_and_reused_peer_ids_have_separate_private_files() {
    let temp = tempfile::tempdir().unwrap();
    let root = WitnessStore::open(&temp.path().join("witnesses")).unwrap();
    let mut first = CheckpointWriter::new(&root, "robot-one".into()).unwrap();
    let mut second = CheckpointWriter::new(&root, "robot-two".into()).unwrap();
    for events in 1..=32 {
        first.offer(witness(events)).unwrap();
        second.offer(witness(events + 100)).unwrap();
    }
    let (first, second) = tokio::join!(finish(&mut first), finish(&mut second));
    assert_ne!(first.path, second.path);
    for (saved, events, robot) in [(first, 32, "robot-one"), (second, 132, "robot-two")] {
        let value: serde_json::Value =
            serde_json::from_reader(File::open(&saved.path).unwrap()).unwrap();
        assert_eq!(
            value,
            serde_json::json!({
                "kind":"checkpoint", "robot_eid":robot, "session_id":"reused-peer-id",
                "head":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", "events":events,
            })
        );
        assert_eq!(
            std::fs::metadata(&saved.path).unwrap().permissions().mode() & 0o777,
            0o600
        );
        let parent = saved.path.parent().unwrap();
        assert_eq!(
            std::fs::metadata(parent).unwrap().permissions().mode() & 0o777,
            0o700
        );
        assert_eq!(std::fs::read_dir(parent).unwrap().count(), 1);
    }
    assert_eq!(std::fs::read_dir(&root.path).unwrap().count(), 2);
}

#[test]
fn checkpoint_reservation_and_updates_never_replace_foreign_entries() {
    let temp = tempfile::tempdir().unwrap();
    let root = WitnessStore::open(&temp.path().join("witnesses")).unwrap();
    let sentinel = temp.path().join("sentinel");
    std::fs::write(&sentinel, b"owned elsewhere").unwrap();
    symlink(&sentinel, root.path.join("reserved")).unwrap();
    assert!(Store::reserve(&root.dir, root.path.join("reserved"), "reserved").is_err());
    let mut store = Store::create(&root.dir, root.path.clone()).unwrap();
    symlink(&sentinel, store.path.join("witness.json")).unwrap();
    assert!(store.persist("robot", &witness(1)).is_err());
    assert!(!store.path.join("pending.json").exists());
    std::fs::remove_file(store.path.join("witness.json")).unwrap();
    let path = store.persist("robot", &witness(1)).unwrap();
    let first = std::fs::read(&path).unwrap();
    let final_file = root.path.join("immutable-final.json");
    std::fs::write(&final_file, b"immutable final").unwrap();
    std::fs::rename(&path, store.path.join("retained-original")).unwrap();
    std::fs::write(&path, b"replacement").unwrap();
    assert!(store.persist("robot", &witness(2)).is_err());
    assert_eq!(std::fs::read(&path).unwrap(), b"replacement");
    assert_eq!(
        std::fs::read(store.path.join("retained-original")).unwrap(),
        first
    );
    assert_eq!(std::fs::read(&final_file).unwrap(), b"immutable final");
    assert_eq!(std::fs::read(&sentinel).unwrap(), b"owned elsewhere");
}

#[tokio::test]
async fn failed_checkpoint_is_sticky_and_preserves_the_previous_commitment() {
    let temp = tempfile::tempdir().unwrap();
    let root = WitnessStore::open(&temp.path().join("witnesses")).unwrap();
    let mut writer = CheckpointWriter::new(&root, "robot".into()).unwrap();
    writer.offer(witness(1)).unwrap();
    let saved = tokio::time::timeout(Duration::from_secs(5), writer.progress())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    let previous = std::fs::read(&saved.path).unwrap();
    let pending = saved.path.parent().unwrap().join("pending.json");
    std::fs::write(&pending, b"existing temporary file").unwrap();
    writer.offer(witness(2)).unwrap();
    let error = tokio::time::timeout(Duration::from_secs(5), writer.progress())
        .await
        .unwrap()
        .err()
        .unwrap()
        .to_string();
    assert!(error.contains("recording checkpoint failed"), "{error}");
    assert!(writer.is_done());
    assert!(writer.offer(witness(3)).is_err());
    assert_eq!(std::fs::read(saved.path).unwrap(), previous);
    assert_eq!(std::fs::read(pending).unwrap(), b"existing temporary file");
}

#[tokio::test]
async fn dropped_notification_receiver_cannot_skip_the_pending_commitment() {
    let temp = tempfile::tempdir().unwrap();
    let root = WitnessStore::open(&temp.path().join("witnesses")).unwrap();
    let (input, receiver) = watch::channel(None);
    let (status, output) = watch::channel(Status::Pending);
    drop(output);
    input.send(Some(witness(17))).unwrap();
    drop(input);
    tokio::time::timeout(
        Duration::from_secs(5),
        run(
            receiver,
            status,
            root.dir.try_clone().unwrap(),
            root.path.clone(),
            "robot".into(),
        ),
    )
    .await
    .unwrap();
    let directory = std::fs::read_dir(root.path)
        .unwrap()
        .next()
        .unwrap()
        .unwrap()
        .path();
    let value: serde_json::Value =
        serde_json::from_reader(File::open(directory.join("witness.json")).unwrap()).unwrap();
    assert_eq!(value["events"], 17);
    assert_eq!(value["session_id"], "reused-peer-id");
}

#[tokio::test]
async fn invalid_or_backwards_offers_leave_the_latest_pending_commitment_intact() {
    let temp = tempfile::tempdir().unwrap();
    let root = WitnessStore::open(&temp.path().join("witnesses")).unwrap();
    let mut writer = CheckpointWriter::new(&root, "robot".into()).unwrap();
    writer.offer(witness(10)).unwrap();
    writer.offer(witness(10)).unwrap();
    for invalid in [
        RecordingWitness {
            events: 9,
            ..witness(10)
        },
        RecordingWitness {
            head: "b".repeat(64),
            ..witness(10)
        },
        RecordingWitness {
            session_id: "another".into(),
            ..witness(11)
        },
        RecordingWitness {
            session_id: "../escape".into(),
            ..witness(11)
        },
        RecordingWitness {
            head: "A".repeat(64),
            ..witness(11)
        },
        RecordingWitness {
            events: 0,
            ..witness(10)
        },
    ] {
        assert!(writer.offer(invalid).is_err());
        assert_eq!(writer.offered, Some(witness(10)));
    }
    writer.offer(witness(11)).unwrap();
    let saved = finish(&mut writer).await;
    let value: serde_json::Value =
        serde_json::from_reader(File::open(saved.path).unwrap()).unwrap();
    assert_eq!(value["events"], 11);
    assert_eq!(value["session_id"], "reused-peer-id");
    assert_eq!(
        value["head"],
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
    );
    assert!(writer.offer(witness(12)).is_err());
}

#[tokio::test]
async fn empty_finish_and_abort_close_input_without_creating_an_artifact() {
    let temp = tempfile::tempdir().unwrap();
    let root = WitnessStore::open(&temp.path().join("witnesses")).unwrap();
    for invalid in ["", "../escape", "robot/name"] {
        assert!(CheckpointWriter::new(&root, invalid.into()).is_err());
        assert_eq!(std::fs::read_dir(&root.path).unwrap().count(), 0);
    }
    let mut writer = CheckpointWriter::new(&root, "robot".into()).unwrap();
    writer.finish();
    assert!(writer.offer(witness(1)).is_err());
    assert!(
        tokio::time::timeout(Duration::from_secs(5), writer.progress())
            .await
            .unwrap()
            .unwrap()
            .is_none()
    );
    assert!(writer.is_done());
    assert_eq!(std::fs::read_dir(&root.path).unwrap().count(), 0);
    let mut aborted = CheckpointWriter::new(&root, "robot".into()).unwrap();
    aborted.abort();
    assert!(aborted.is_done());
    assert!(aborted.offer(witness(1)).is_err());
    assert_eq!(std::fs::read_dir(root.path).unwrap().count(), 0);
}
