// SPDX-License-Identifier: AGPL-3.0-only

use super::*;

// These event bytes and heads are independent protocol fixtures, not chains
// produced by the recording implementation under test.
const GOLDEN_EXEC_RECORDING: &str =
    include_str!("../../../../cerulion_ops_proto/tests/fixtures/recording_exec_golden.jsonl");
const GOLDEN_SHELL_RECORDING: &str =
    include_str!("../../../../cerulion_ops_proto/tests/fixtures/recording_shell_golden.jsonl");

fn golden_exec_recording_witness() -> RecordingWitness {
    RecordingWitness {
        session_id: "exec-1".into(),
        head: "3f1352e989697121f70af74c842c8cb73833a47a9a19361995af4b77d5a2055c".into(),
        events: 4,
    }
}

fn golden_shell_recording_witness() -> RecordingWitness {
    RecordingWitness {
        session_id: "shell-1".into(),
        head: "7668ad26affbdd0fe5ac260fa107f93ab31b7ff2baacc64204f1c471d6f92232".into(),
        events: 6,
    }
}

#[test]
fn listing_complete_recordings_matches_independent_witnesses_in_session_order() {
    let directory = tempfile::tempdir().unwrap();
    let cases = [
        (
            "a-shell.jsonl",
            GOLDEN_SHELL_RECORDING,
            golden_shell_recording_witness(),
        ),
        (
            "z-exec.jsonl",
            GOLDEN_EXEC_RECORDING,
            golden_exec_recording_witness(),
        ),
    ];
    for (name, bytes, _) in &cases {
        std::fs::write(directory.path().join(name), bytes).unwrap();
    }
    assert_eq!(
        list_recordings(directory.path()).unwrap(),
        vec![
            golden_exec_recording_witness(),
            golden_shell_recording_witness()
        ]
    );
    for (name, bytes, witness) in &cases {
        let path = directory.path().join(name);
        verify_recording_file(&path, witness).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), bytes.as_bytes());
    }
}

#[test]
fn listing_skips_empty_live_and_interrupted_recordings_without_mutation() {
    let lines: Vec<_> = GOLDEN_EXEC_RECORDING.split_inclusive('\n').collect();
    assert_eq!(lines.len(), 4);
    let prefix = lines[..3].concat().into_bytes();
    let mut partial_json = prefix.clone();
    partial_json.extend_from_slice(b"{\"sequence\":3");
    let mut partial_utf8 = prefix.clone();
    partial_utf8.extend_from_slice(b"{\"text\":\"\xf0\x9f");
    let cases = [
        ("empty", Vec::new()),
        ("started", lines[0].as_bytes().to_vec()),
        ("live-output", prefix),
        (
            "torn-started",
            b"{\"sequence\":0,\"payload\":{\"type\":\"started".to_vec(),
        ),
        ("torn-started-utf8", b"{\"session_id\":\"\xf0\x9f".to_vec()),
        ("torn-next-event", partial_json),
        ("torn-next-utf8", partial_utf8),
    ];
    for (name, bytes) in cases {
        let directory = tempfile::tempdir().unwrap();
        let complete = directory.path().join("complete.jsonl");
        let pending = directory.path().join("pending.jsonl");
        std::fs::write(&complete, GOLDEN_EXEC_RECORDING).unwrap();
        std::fs::write(&pending, &bytes).unwrap();
        assert_eq!(
            list_recordings(directory.path()).unwrap(),
            vec![golden_exec_recording_witness()],
            "{name} must not hide a complete recording"
        );
        assert_eq!(std::fs::read(&pending).unwrap(), bytes, "{name}");
        assert_eq!(
            std::fs::read(&complete).unwrap(),
            GOLDEN_EXEC_RECORDING.as_bytes()
        );
        assert!(
            verify_recording_file(&pending, &golden_exec_recording_witness()).is_err(),
            "{name} must not become a verified complete recording"
        );
    }
}

#[test]
fn listing_rejects_corrupt_complete_lines_and_chain_structure() {
    let lines: Vec<_> = GOLDEN_EXEC_RECORDING.split_inclusive('\n').collect();
    let prefix = lines[..3].concat();
    let corrupt_prefix = prefix.replacen("owner-1", "owner-2", 1);
    let cases = [
        ("malformed-first-line", b"not json\n".to_vec()),
        (
            "malformed-complete-line",
            [prefix.as_bytes(), b"not json\n"].concat(),
        ),
        (
            "invalid-utf8-line",
            [prefix.as_bytes(), b"\xf0\x9f\n"].concat(),
        ),
        ("missing-started", lines[1].as_bytes().to_vec()),
        (
            "wrong-hash",
            GOLDEN_EXEC_RECORDING
                .replacen("\"hash\":\"bef11", "\"hash\":\"0ef11", 1)
                .into_bytes(),
        ),
        (
            "wrong-sequence",
            GOLDEN_EXEC_RECORDING
                .replacen("\"sequence\":1", "\"sequence\":9", 1)
                .into_bytes(),
        ),
        (
            "wrong-previous-hash",
            GOLDEN_EXEC_RECORDING
                .replacen("\"previous_hash\":\"bef11", "\"previous_hash\":\"0ef11", 1)
                .into_bytes(),
        ),
        (
            "reordered-events",
            [lines[0], lines[2], lines[1], lines[3]]
                .concat()
                .into_bytes(),
        ),
        (
            "corrupt-unfinished-prefix",
            corrupt_prefix.as_bytes().to_vec(),
        ),
        (
            "corrupt-prefix-and-torn-tail",
            [corrupt_prefix.as_bytes(), b"{\"sequence\":3"].concat(),
        ),
    ];
    for (name, bytes) in cases {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("corrupt.jsonl");
        std::fs::write(&path, &bytes).unwrap();
        assert!(list_recordings(directory.path()).is_err(), "{name}");
        assert_eq!(std::fs::read(&path).unwrap(), bytes, "{name}");
    }
}

#[test]
fn listing_rejects_every_observed_tail_after_exit() {
    let lines: Vec<_> = GOLDEN_EXEC_RECORDING.split_inclusive('\n').collect();
    let tails: [&[u8]; 8] = [
        b" ",
        b"\xf0\x9f",
        b"{\"sequence\":4",
        b"\n",
        b"{}\n",
        lines[0].as_bytes(),
        lines[1].as_bytes(),
        lines[3].as_bytes(),
    ];
    for tail in tails {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("after-exit.jsonl");
        let bytes = [GOLDEN_EXEC_RECORDING.as_bytes(), tail].concat();
        std::fs::write(&path, &bytes).unwrap();
        assert!(list_recordings(directory.path()).is_err(), "tail: {tail:?}");
        assert_eq!(std::fs::read(&path).unwrap(), bytes);
    }
}

#[test]
fn listing_bounds_complete_and_unterminated_event_lines() {
    let started = GOLDEN_EXEC_RECORDING.split_inclusive('\n').next().unwrap();
    for prefix in [b"".as_slice(), started.as_bytes()] {
        for terminated in [false, true] {
            let directory = tempfile::tempdir().unwrap();
            let path = directory.path().join("oversized.jsonl");
            let mut line = vec![b' '; 2 * cerulion_ops_proto::session::MAX_SESSION_FRAME_BYTES + 1];
            if terminated {
                *line.last_mut().unwrap() = b'\n';
            }
            let bytes = [prefix, line.as_slice()].concat();
            std::fs::write(&path, &bytes).unwrap();
            assert_eq!(
                list_recordings(directory.path()).unwrap_err().to_string(),
                "recording event is oversized"
            );
            assert_eq!(std::fs::read(&path).unwrap(), bytes);
        }
    }
}

#[test]
fn listing_propagates_recording_read_errors() {
    let directory = tempfile::tempdir().unwrap();
    let not_a_file = directory.path().join("unreadable.jsonl");
    std::fs::create_dir(&not_a_file).unwrap();
    assert!(list_recordings(directory.path()).is_err());
    assert!(not_a_file.is_dir());
}

#[test]
fn inspector_rejects_invalid_json_and_missing_open_event() {
    let directory = tempfile::tempdir().unwrap();
    let file = directory.path().join("invalid.jsonl");
    std::fs::write(&file, b"not json\n").unwrap();
    assert!(list_recordings(directory.path()).is_err());
    assert!(verify_recording_file(
        &file,
        &RecordingWitness {
            session_id: "x".into(),
            head: "0".repeat(64),
            events: 1
        }
    )
    .is_err());
    assert_eq!(
        io::ErrorKind::NotFound,
        list_recordings(&directory.path().join("missing"))
            .unwrap_err()
            .kind()
    );
    assert_eq!(std::fs::read(&file).unwrap(), b"not json\n");
}

fn golden_prefix_witness() -> RecordingWitness {
    RecordingWitness {
        session_id: "exec-1".into(),
        head: "596f1659d312d1e604d7e0ae2bb93a1eaa09c97a06b3f2ee2833c3a9183f0cfa".into(),
        events: 3,
    }
}

#[test]
fn prefix_verification_checks_the_external_commitment_and_reports_unverified_bytes() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("prefix.jsonl");
    let prefix: String = GOLDEN_EXEC_RECORDING
        .split_inclusive('\n')
        .take(3)
        .collect();
    for _ in 0..2 {
        std::fs::write(&path, &prefix).unwrap();
        assert_eq!(
            verify_recording_prefix_file(&path, &golden_prefix_witness()).unwrap(),
            RecordingPrefixSummary::default()
        );
        let bytes = [prefix.as_bytes(), b"not an event\n\xff\npartial"].concat();
        std::fs::write(&path, &bytes).unwrap();
        assert_eq!(
            verify_recording_prefix_file(&path, &golden_prefix_witness()).unwrap(),
            RecordingPrefixSummary {
                unverified_event_lines: 2,
                unverified_bytes: 22,
                unverified_partial_line: true,
            }
        );
        assert!(verify_recording_file(&path, &golden_exec_recording_witness()).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), bytes);
    }
}

#[test]
fn prefix_verification_rejects_short_corrupt_and_mismatched_commitments() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("prefix.jsonl");
    let prefix: String = GOLDEN_EXEC_RECORDING
        .split_inclusive('\n')
        .take(3)
        .collect();
    let corrupt = prefix.replacen("owner-1", "owner-2", 1);
    for bytes in [
        &prefix.as_bytes()[..prefix.len() - 1],
        prefix.split_inclusive('\n').next().unwrap().as_bytes(),
        corrupt.as_bytes(),
    ] {
        std::fs::write(&path, bytes).unwrap();
        assert!(verify_recording_prefix_file(&path, &golden_prefix_witness()).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), bytes);
    }
    std::fs::write(&path, &prefix).unwrap();
    let witness = golden_prefix_witness();
    for invalid in [
        RecordingWitness {
            events: 0,
            ..witness.clone()
        },
        RecordingWitness {
            events: 4,
            ..witness.clone()
        },
        RecordingWitness {
            head: "0".repeat(64),
            ..witness.clone()
        },
        RecordingWitness {
            session_id: "another-session".into(),
            ..witness
        },
    ] {
        assert!(verify_recording_prefix_file(&path, &invalid).is_err());
    }
    assert_eq!(std::fs::read(&path).unwrap(), prefix.as_bytes());
}

#[test]
fn complete_verification_requires_bounded_newline_terminated_events() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("audit.jsonl");
    let incomplete = &GOLDEN_EXEC_RECORDING.as_bytes()[..GOLDEN_EXEC_RECORDING.len() - 1];
    std::fs::write(&path, incomplete).unwrap();
    assert!(verify_recording_file(&path, &golden_exec_recording_witness()).is_err());
    assert_eq!(std::fs::read(&path).unwrap(), incomplete);
    let oversized = vec![b' '; 2_097_153];
    std::fs::write(&path, &oversized).unwrap();
    assert_eq!(
        verify_recording_file(&path, &golden_exec_recording_witness())
            .unwrap_err()
            .to_string(),
        "recording event is oversized"
    );
    assert_eq!(
        verify_recording_prefix_file(&path, &golden_prefix_witness())
            .unwrap_err()
            .to_string(),
        "recording prefix event is oversized"
    );
    assert_eq!(std::fs::read(&path).unwrap(), oversized);
}

#[test]
fn an_event_at_the_exact_byte_cap_is_accepted_against_the_literal_witness() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("audit.jsonl");
    let (first, rest) = GOLDEN_EXEC_RECORDING.split_once('\n').unwrap();
    let mut bytes = first.as_bytes().to_vec();
    bytes.resize(2_097_151, b' ');
    bytes.push(b'\n');
    bytes.extend_from_slice(rest.as_bytes());
    std::fs::write(&path, &bytes).unwrap();
    verify_recording_file(&path, &golden_exec_recording_witness()).unwrap();
    assert_eq!(
        list_recordings(directory.path()).unwrap(),
        [golden_exec_recording_witness()]
    );
    assert_eq!(std::fs::read(&path).unwrap(), bytes);
}

#[test]
fn listing_reports_incomplete_paths_in_order_and_ignores_other_extensions() {
    let directory = tempfile::tempdir().unwrap();
    let first = directory.path().join("a.jsonl");
    let last = directory.path().join("z.jsonl");
    std::fs::write(&last, b"").unwrap();
    std::fs::write(&first, b"{\"sequence\":").unwrap();
    let ignored = directory.path().join("operator.txt");
    std::fs::write(&ignored, b"not a recording").unwrap();
    for _ in 0..2 {
        let listing = list_recordings_with_incomplete(directory.path()).unwrap();
        assert!(listing.witnesses.is_empty());
        assert_eq!(listing.incomplete, [first.clone(), last.clone()]);
    }
    assert_eq!(std::fs::read(&first).unwrap(), b"{\"sequence\":");
    assert!(std::fs::read(&last).unwrap().is_empty());
    assert_eq!(std::fs::read(&ignored).unwrap(), b"not a recording");
}

#[cfg(unix)]
#[test]
fn inspectors_refuse_symlinks_and_special_files_without_changing_them() {
    use std::os::unix::fs::{symlink, FileTypeExt};
    let directory = tempfile::tempdir().unwrap();
    let target = directory.path().join("source.txt");
    let path = directory.path().join("audit.jsonl");
    std::fs::write(&target, GOLDEN_EXEC_RECORDING).unwrap();
    symlink(&target, &path).unwrap();
    assert!(list_recordings(directory.path()).is_err());
    assert!(verify_recording_file(&path, &golden_exec_recording_witness()).is_err());
    assert!(verify_recording_prefix_file(&path, &golden_prefix_witness()).is_err());
    assert_eq!(
        std::fs::read(&target).unwrap(),
        GOLDEN_EXEC_RECORDING.as_bytes()
    );
    assert_eq!(std::fs::read_link(&path).unwrap(), target);
    let fifo = directory.path().join("pipe.jsonl");
    assert!(std::process::Command::new("mkfifo")
        .arg(&fifo)
        .status()
        .unwrap()
        .success());
    // Metadata must refuse this FIFO before opening it without a writer.
    assert!(open_recording(&fifo).is_err());
    assert!(std::fs::symlink_metadata(&fifo)
        .unwrap()
        .file_type()
        .is_fifo());
}

#[cfg(unix)]
#[test]
fn inspection_refuses_a_replaced_file_between_metadata_and_open() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("audit.jsonl");
    let replacement = directory.path().join("replacement");
    std::fs::write(&path, GOLDEN_EXEC_RECORDING).unwrap();
    std::fs::write(&replacement, GOLDEN_SHELL_RECORDING).unwrap();
    let error = open_recording_with(&path, |path| {
        std::fs::rename(&replacement, path)?;
        std::fs::File::open(path)
    })
    .unwrap_err();
    assert_eq!(error.to_string(), "recording changed while opening");
    assert_eq!(
        std::fs::read(&path).unwrap(),
        GOLDEN_SHELL_RECORDING.as_bytes()
    );
}

#[test]
fn inspection_propagates_a_vanished_file_and_refuses_a_replacement_directory() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("audit.jsonl");
    std::fs::write(&path, GOLDEN_EXEC_RECORDING).unwrap();
    let error = open_recording_with(&path, |path| {
        std::fs::remove_file(path)?;
        std::fs::File::open(path)
    })
    .unwrap_err();
    assert_eq!(error.kind(), io::ErrorKind::NotFound);
    assert!(!path.exists());
    std::fs::write(&path, GOLDEN_EXEC_RECORDING).unwrap();
    assert!(open_recording_with(&path, |path| {
        std::fs::remove_file(path)?;
        std::fs::create_dir(path)?;
        std::fs::File::open(path)
    })
    .is_err());
    assert!(path.is_dir());
    assert_eq!(std::fs::read_dir(&path).unwrap().count(), 0);
}

#[test]
fn complete_heads_with_the_same_session_id_have_a_deterministic_tie_order() {
    // Independent fixture derived from the specified preimage with Python hashlib.
    const OTHER: &str = include_str!("fixtures/same_session_other_owner.jsonl");
    let other = RecordingWitness {
        session_id: "exec-1".into(),
        head: "d5a80e58802b30e672842ddc7f44f17cb51bb10112201e128e21285d4946d085".into(),
        events: 4,
    };
    for (first, second) in [
        (GOLDEN_EXEC_RECORDING, OTHER),
        (OTHER, GOLDEN_EXEC_RECORDING),
    ] {
        let directory = tempfile::tempdir().unwrap();
        let a = directory.path().join("a.jsonl");
        let z = directory.path().join("z.jsonl");
        std::fs::write(&a, first).unwrap();
        std::fs::write(&z, second).unwrap();
        assert_eq!(
            list_recordings(directory.path()).unwrap(),
            vec![golden_exec_recording_witness(), other.clone()]
        );
        assert_eq!(std::fs::read(&a).unwrap(), first.as_bytes());
        assert_eq!(std::fs::read(&z).unwrap(), second.as_bytes());
    }
}
