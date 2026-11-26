// SPDX-License-Identifier: AGPL-3.0-only
//! Real private files, fixed wire vectors, and refusal boundaries.

use cerulion_ops_proto::recording::{RecordingEvent, RecordingPayload};
use cerulion_ops_proto::session::{ExecRequest, ExitReason, OutputStream, SessionKind};

use super::*;

#[test]
fn repeated_real_files_match_the_independent_exec_wire_vector() {
    let oracle =
        include_bytes!("../../cerulion_ops_proto/tests/fixtures/recording_exec_golden.jsonl");
    for _ in 0..2 {
        let directory = private_directory();
        let mut file = RecordingFile::create(directory.path(), "exec-1".into(), 4096).unwrap();
        let events = [
            RecordingPayload::Started {
                session_id: "exec-1".into(),
                principal_id: "owner-1".into(),
                account_id: Some("account-1".into()),
                session: SessionKind::Exec(ExecRequest {
                    argv: vec![
                        "sh".into(),
                        "-c".into(),
                        "printf out; printf err >&2; exit 7".into(),
                    ],
                }),
            },
            RecordingPayload::Output {
                stream: OutputStream::Stdout,
                data: "b3V0".into(),
            },
            RecordingPayload::Output {
                stream: OutputStream::Stderr,
                data: "ZXJy".into(),
            },
            RecordingPayload::Exit {
                code: Some(7),
                signal: None,
                reason: ExitReason::Exited,
            },
        ];
        for (index, event) in events.into_iter().enumerate() {
            assert_eq!(
                file.append(index as u64 * 100, event).unwrap(),
                AppendOutcome::Appended
            );
        }
        assert_eq!(
            std::fs::read(directory.path().join("exec-1.jsonl")).unwrap(),
            oracle
        );
        assert_eq!(
            file.witness(),
            RecordingWitness {
                session_id: "exec-1".into(),
                events: 4,
                head: "3f1352e989697121f70af74c842c8cb73833a47a9a19361995af4b77d5a2055c".into(),
            }
        );
    }
}

#[test]
fn creation_keeps_private_modes_and_never_replaces_existing_files() {
    let parent = private_directory();
    let directory = parent.path().join("recordings");
    let _file = RecordingFile::create(&directory, "new-session".into(), 4096).unwrap();
    let path = directory.join("new-session.jsonl");
    assert_eq!(
        std::fs::metadata(&directory).unwrap().permissions().mode() & 0o777,
        0o700
    );
    assert_eq!(
        std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
        0o600
    );
    std::fs::write(&path, b"existing audit bytes").unwrap();
    assert_eq!(
        RecordingFile::create(&directory, "new-session".into(), 4096)
            .err()
            .unwrap()
            .kind(),
        io::ErrorKind::AlreadyExists
    );
    assert_eq!(std::fs::read(&path).unwrap(), b"existing audit bytes");
}

#[test]
fn path_components_symlinks_and_nonprivate_directories_are_refused() {
    let parent = private_directory();
    let directory = parent.path().join("recordings");
    let absolute = parent.path().join("absolute-session");
    for id in [
        "",
        ".",
        "..",
        "../escape",
        "nested/name",
        "trailing/",
        "nul\0id",
        absolute.to_str().unwrap(),
    ] {
        assert_eq!(
            RecordingFile::create(&directory, id.into(), 4096)
                .err()
                .unwrap()
                .kind(),
            io::ErrorKind::InvalidInput
        );
        assert!(!directory.exists());
    }
    std::fs::create_dir(&directory).unwrap();
    std::fs::set_permissions(&directory, std::fs::Permissions::from_mode(0o755)).unwrap();
    assert_eq!(
        RecordingFile::create(&directory, "session".into(), 4096)
            .err()
            .unwrap()
            .kind(),
        io::ErrorKind::PermissionDenied
    );
    std::fs::set_permissions(&directory, std::fs::Permissions::from_mode(0o700)).unwrap();
    let link = parent.path().join("link");
    std::os::unix::fs::symlink(&directory, &link).unwrap();
    assert_eq!(
        RecordingFile::create(&link, "session".into(), 4096)
            .err()
            .unwrap()
            .kind(),
        io::ErrorKind::PermissionDenied
    );
    let ordinary = parent.path().join("ordinary");
    std::fs::write(&ordinary, b"not a directory").unwrap();
    assert_eq!(
        RecordingFile::create(&ordinary, "session".into(), 4096)
            .err()
            .unwrap()
            .kind(),
        io::ErrorKind::PermissionDenied
    );
    assert_eq!(
        RecordingFile::create(&parent.path().join("missing/child"), "session".into(), 4096)
            .err()
            .unwrap()
            .kind(),
        io::ErrorKind::NotFound
    );
    assert!(!directory.join("session.jsonl").exists());
}

#[test]
fn invalid_start_and_backwards_time_do_not_change_the_commitment() {
    let directory = private_directory();
    let mut file = RecordingFile::create(directory.path(), "test-session".into(), 4096).unwrap();
    let start = |session_id: &str| RecordingPayload::Started {
        session_id: session_id.into(),
        principal_id: "owner".into(),
        account_id: None,
        session: SessionKind::Exec(ExecRequest {
            argv: vec!["/bin/true".into()],
        }),
    };
    assert_eq!(
        file.append(1, start("different-session"))
            .unwrap_err()
            .kind(),
        io::ErrorKind::InvalidInput
    );
    assert_eq!(file.witness().events, 0);
    assert!(std::fs::read(directory.path().join("test-session.jsonl"))
        .unwrap()
        .is_empty());
    file.append(2, start("test-session")).unwrap();
    let witness = file.witness();
    let before = std::fs::read(directory.path().join("test-session.jsonl")).unwrap();
    assert!(file.append(3, start("test-session")).is_err());
    assert!(file.append(1, RecordingPayload::StdinEof).is_err());
    assert_eq!(file.witness(), witness);
    assert_eq!(
        std::fs::read(directory.path().join("test-session.jsonl")).unwrap(),
        before
    );
    file.append(3, exit()).unwrap();
    assert!(file.append(4, RecordingPayload::StdinEof).is_err());
}

#[test]
fn oversized_identity_fields_are_refused_before_any_file_or_chain_change() {
    let directory = private_directory();
    let mut file =
        RecordingFile::create(directory.path(), "test-session".into(), 8 * 1024 * 1024).unwrap();
    let path = directory.path().join("test-session.jsonl");
    let initial = file.witness();
    for (principal_id, account_id) in [
        ("p".repeat(2_097_153), None),
        ("owner".into(), Some("a".repeat(2_097_153))),
        // JSON escaping, not the input string length, determines the line size.
        ("\"".repeat(1_048_576), None),
    ] {
        assert_eq!(
            file.append(
                100,
                RecordingPayload::Started {
                    session_id: "test-session".into(),
                    principal_id,
                    account_id,
                    session: SessionKind::Exec(ExecRequest {
                        argv: vec!["/bin/true".into()],
                    }),
                },
            )
            .unwrap_err()
            .kind(),
            io::ErrorKind::InvalidData
        );
        assert_eq!(file.witness(), initial);
        assert_eq!(file.bytes, 0);
        assert!(std::fs::read(&path).unwrap().is_empty());
    }
    // Refusal is healthy and cannot advance the timestamp.
    file.append(1, start_with_principal("owner".into()))
        .unwrap();
    file.append(2, exit()).unwrap();
    crate::remote_session::verify_recording_file(&path, &file.witness()).unwrap();
}

#[test]
fn exact_event_size_boundary_stays_readable_by_all_file_readers() {
    assert_eq!(MAX_RECORDING_EVENT_BYTES, 2_097_152);
    // Independent wire skeleton: changing either 64-byte hash changes its bytes,
    // never its length. All other event fields are literal, including the newline.
    let prefix = r#"{"sequence":0,"timestamp_micros":1,"payload":{"type":"started","session_id":"test-session","principal_id":""#;
    let suffix = concat!(
        r#"","session":{"type":"exec","argv":["/bin/true"]}},"previous_hash":""#,
        "0000000000000000000000000000000000000000000000000000000000000000",
        r#"","hash":""#,
        "0000000000000000000000000000000000000000000000000000000000000000",
        "\"}\n"
    );
    let exact_principal_bytes = 2_097_152 - prefix.len() - suffix.len();
    for extra in [-1_isize, 0, 1] {
        let directory = private_directory();
        let path = directory.path().join("test-session.jsonl");
        let mut file =
            RecordingFile::create(directory.path(), "test-session".into(), 8 * 1024 * 1024)
                .unwrap();
        let initial = file.witness();
        let principal = "p".repeat(exact_principal_bytes.checked_add_signed(extra).unwrap());
        let result = file.append(1, start_with_principal(principal));
        if extra == 1 {
            assert_eq!(result.unwrap_err().kind(), io::ErrorKind::InvalidData);
            assert_eq!(file.witness(), initial);
            assert!(std::fs::read(&path).unwrap().is_empty());
            continue;
        }
        assert_eq!(result.unwrap(), AppendOutcome::Appended);
        assert_eq!(
            std::fs::metadata(&path).unwrap().len(),
            2_097_152_u64.checked_add_signed(extra as i64).unwrap()
        );
        let prefix_witness = file.witness();
        crate::remote_session::verify_recording_prefix_file(&path, &prefix_witness).unwrap();
        file.append(2, exit()).unwrap();
        crate::remote_session::verify_recording_file(&path, &file.witness()).unwrap();
        assert_eq!(
            crate::remote_session::list_recordings(directory.path()).unwrap(),
            vec![file.witness()]
        );
    }
}

fn start_with_principal(principal_id: String) -> RecordingPayload {
    RecordingPayload::Started {
        session_id: "test-session".into(),
        principal_id,
        account_id: None,
        session: SessionKind::Exec(ExecRequest {
            argv: vec!["/bin/true".into()],
        }),
    }
}

#[test]
fn write_failure_is_sticky_even_after_the_file_handle_becomes_writable() {
    let directory = private_directory();
    let mut file = recorder(directory.path());
    let path = directory.path().join("test-session.jsonl");
    let before = std::fs::read(&path).unwrap();
    let witness = file.witness();
    file.file = Arc::new(File::open(&path).unwrap());
    assert!(file.append(2, RecordingPayload::StdinEof).is_err());
    file.file = Arc::new(OpenOptions::new().append(true).open(&path).unwrap());
    assert_eq!(
        file.append(3, exit()).unwrap_err().to_string(),
        "session recording write failed"
    );
    assert_eq!(file.witness(), witness);
    assert_eq!(std::fs::read(path).unwrap(), before);
}

#[test]
fn byte_count_overflow_refuses_before_writing_and_the_budget_boundary_is_exact() {
    assert!(!exceeds_recording_limit(0, 317, false));
    assert!(exceeds_recording_limit(1, 317, false));
    assert!(!exceeds_recording_limit(100, 418, false));
    assert!(exceeds_recording_limit(101, 418, false));
    assert!(!exceeds_recording_limit(418, 418, true));
    assert!(exceeds_recording_limit(419, 418, true));
    let directory = private_directory();
    let mut file = recorder(directory.path());
    let path = directory.path().join("test-session.jsonl");
    let before = std::fs::read(&path).unwrap();
    let witness = file.witness();
    file.bytes = u64::MAX;
    assert_eq!(
        file.append(2, RecordingPayload::StdinEof)
            .unwrap_err()
            .to_string(),
        "session recording size overflow"
    );
    assert_eq!(file.witness(), witness);
    assert_eq!(std::fs::read(path).unwrap(), before);
}

fn private_directory() -> tempfile::TempDir {
    // RecordingFile requires a private existing directory; create it explicitly
    // rather than inheriting permissive temporary-directory defaults.
    let directory = tempfile::Builder::new()
        .permissions(std::fs::Permissions::from_mode(0o700))
        .tempdir()
        .unwrap();
    assert_eq!(
        std::fs::metadata(directory.path())
            .unwrap()
            .permissions()
            .mode()
            & 0o777,
        0o700
    );
    directory
}

fn recorder(directory: &Path) -> RecordingFile {
    let mut recorder =
        RecordingFile::create(directory, "test-session".into(), 8 * 1024 * 1024).unwrap();
    recorder
        .append(
            1,
            RecordingPayload::Started {
                session_id: "test-session".into(),
                principal_id: "test-owner".into(),
                account_id: None,
                session: SessionKind::Exec(ExecRequest {
                    argv: vec!["/bin/cat".into()],
                }),
            },
        )
        .unwrap();
    recorder
}

fn exit() -> RecordingPayload {
    RecordingPayload::Exit {
        code: Some(0),
        signal: None,
        reason: ExitReason::Exited,
    }
}

fn read_events(directory: &Path) -> Vec<RecordingEvent> {
    std::fs::read_to_string(directory.join("test-session.jsonl"))
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

#[test]
fn exit_reserve_covers_every_wire_reason_and_status_shape() {
    assert_eq!(EXIT_RESERVE_BYTES, 318);
    let reasons = [
        ExitReason::Exited,
        ExitReason::ClientClosed,
        ExitReason::Disconnected,
        ExitReason::Revoked,
        ExitReason::ProtocolError,
        ExitReason::RecordingFailed,
        ExitReason::CleanupFailed,
        ExitReason::SpawnFailed,
        ExitReason::TimedOut,
        ExitReason::ResourceLimit,
    ];
    for reason in reasons {
        // The exhaustive mapping and literal field order are independent of
        // serde's encoder; a new reason must update this wire-size oracle.
        let reason_text = match reason {
            ExitReason::Exited => "exited",
            ExitReason::ClientClosed => "client_closed",
            ExitReason::Disconnected => "disconnected",
            ExitReason::Revoked => "revoked",
            ExitReason::ProtocolError => "protocol_error",
            ExitReason::RecordingFailed => "recording_failed",
            ExitReason::CleanupFailed => "cleanup_failed",
            ExitReason::SpawnFailed => "spawn_failed",
            ExitReason::TimedOut => "timed_out",
            ExitReason::ResourceLimit => "resource_limit",
        };
        for (code, signal) in [
            (None, None),
            (Some(i32::MIN), None),
            (Some(i32::MAX), None),
            (None, Some(i32::MIN)),
            (None, Some(i32::MAX)),
        ] {
            let hash = "f".repeat(64);
            let event = RecordingEvent {
                sequence: u64::MAX,
                timestamp_micros: u64::MAX,
                payload: RecordingPayload::Exit {
                    code,
                    signal,
                    reason,
                },
                previous_hash: hash.clone(),
                hash: hash.clone(),
            };
            let code_text = code.map_or_else(|| "null".into(), |value| value.to_string());
            let signal_text = signal.map_or_else(|| "null".into(), |value| value.to_string());
            let expected = format!(
                "{{\"sequence\":18446744073709551615,\"timestamp_micros\":18446744073709551615,\"payload\":{{\"type\":\"exit\",\"code\":{code_text},\"signal\":{signal_text},\"reason\":\"{reason_text}\"}},\"previous_hash\":\"{hash}\",\"hash\":\"{hash}\"}}\n"
            );
            let mut actual = serde_json::to_vec(&event).unwrap();
            actual.push(b'\n');
            assert_eq!(actual, expected.as_bytes());
            assert!(actual.len() as u64 <= EXIT_RESERVE_BYTES);
            if reason == ExitReason::RecordingFailed
                && (code == Some(i32::MIN) || signal == Some(i32::MIN))
            {
                assert_eq!(actual.len() as u64, EXIT_RESERVE_BYTES);
            }
        }
    }
}

#[test]
fn rejected_append_preserves_file_head_sequence_and_timestamp_for_exit() {
    let directory = private_directory();
    let mut recorder = recorder(directory.path());
    let path = directory.path().join("test-session.jsonl");
    let before = std::fs::read(&path).unwrap();
    let witness = recorder.witness();
    recorder.cap = before.len() as u64 + 318;
    assert_eq!(
        recorder.append(100, RecordingPayload::StdinEof).unwrap(),
        AppendOutcome::LimitReached
    );
    assert_eq!(recorder.witness(), witness);
    assert_eq!(std::fs::read(&path).unwrap(), before);
    // The rejected timestamp cannot prevent an earlier valid final timestamp.
    assert_eq!(recorder.append(2, exit()).unwrap(), AppendOutcome::Appended);
    let events = read_events(directory.path());
    assert_eq!(events.len(), 2);
    assert_eq!(events[1].sequence, 1);
    assert_eq!(events[1].timestamp_micros, 2);
    assert_eq!(events[1].previous_hash, witness.head);
    assert_eq!(events[1].payload, exit());
    assert!(std::fs::metadata(path).unwrap().len() <= recorder.cap);
    cerulion_ops_proto::verify_recording(&events, &recorder.witness()).unwrap();
}

#[test]
fn recording_boundaries_that_do_not_fit_leave_file_and_chain_unchanged() {
    let directory = private_directory();
    let mut recorder = recorder(directory.path());
    let path = directory.path().join("test-session.jsonl");
    let before = std::fs::read(&path).unwrap();
    let witness = recorder.witness();
    recorder.cap = before.len() as u64;
    assert!(recorder.append(2, exit()).is_err());
    assert_eq!(recorder.witness(), witness);
    assert_eq!(std::fs::read(&path).unwrap(), before);

    let empty = private_directory();
    let mut recorder = RecordingFile::create(empty.path(), "test-session".into(), 1).unwrap();
    assert!(recorder
        .append(
            1,
            RecordingPayload::Started {
                session_id: "test-session".into(),
                principal_id: "owner".into(),
                account_id: None,
                session: SessionKind::Exec(ExecRequest {
                    argv: vec!["/bin/true".into()],
                }),
            },
        )
        .is_err());
    assert_eq!(recorder.witness().events, 0);
    assert_eq!(recorder.witness().head, "0".repeat(64));
    assert!(std::fs::read(empty.path().join("test-session.jsonl"))
        .unwrap()
        .is_empty());
}
