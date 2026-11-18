// SPDX-License-Identifier: MIT OR Apache-2.0
//! Recording oracles: fixtures are hand-written event JSON hashed independently
//! with Python's hashlib, without calling the Rust builder or verifier.

use cerulion_ops_proto::{
    verify_recording, verify_recording_prefix, RecordingChain, RecordingEvent, RecordingPayload,
    RecordingWitness,
};

const SHELL: &str = include_str!("fixtures/recording_shell_golden.jsonl");
const EXEC: &str = include_str!("fixtures/recording_exec_golden.jsonl");

fn events(fixture: &str) -> Vec<RecordingEvent> {
    fixture
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect()
}

fn witness() -> RecordingWitness {
    RecordingWitness {
        session_id: "shell-1".into(),
        head: "7668ad26affbdd0fe5ac260fa107f93ab31b7ff2baacc64204f1c471d6f92232".into(),
        events: 6,
    }
}

#[test]
fn full_shell_and_exec_recordings_match_independent_hash_oracles() {
    for (fixture, expected) in [
        (SHELL, witness()),
        (
            EXEC,
            RecordingWitness {
                session_id: "exec-1".into(),
                head: "3f1352e989697121f70af74c842c8cb73833a47a9a19361995af4b77d5a2055c".into(),
                events: 4,
            },
        ),
    ] {
        let records = events(fixture);
        verify_recording(&records, &expected).unwrap();
        // Both runs are pinned to an independent oracle, not merely each other.
        for _ in 0..2 {
            let mut chain = RecordingChain::new();
            for record in &records {
                let appended = chain
                    .append(record.timestamp_micros, record.payload.clone())
                    .unwrap();
                assert_eq!(&appended, record);
            }
            assert_eq!(chain.head(), expected.head);
            assert_eq!(chain.events(), expected.events);
        }
    }
}

#[test]
fn live_prefixes_match_independent_heads_without_claiming_a_complete_session() {
    let records = events(SHELL);
    for (count, head) in [
        (
            1,
            "d78f2237b17c59bb9fb87b7dcf41b6b3473800ef66a0a52e3f5e16ab6ddfefca",
        ),
        (
            3,
            "2bae8e5fb4febfdba9e1e88a912868724a6eba818f0276485b67b7a02efe1994",
        ),
    ] {
        let checkpoint = RecordingWitness {
            session_id: "shell-1".into(),
            head: head.into(),
            events: count as u64,
        };
        for _ in 0..2 {
            verify_recording_prefix(&records[..count], &checkpoint).unwrap();
        }
        assert!(verify_recording(&records[..count], &checkpoint).is_err());
        assert!(verify_recording_prefix(&records[..count - 1], &checkpoint).is_err());
        assert!(verify_recording_prefix(&records[..count + 1], &checkpoint).is_err());
        let mut changed = records[..count].to_vec();
        changed[count - 1].timestamp_micros += 1;
        assert!(verify_recording_prefix(&changed, &checkpoint).is_err());
        for invalid in [
            RecordingWitness {
                session_id: String::new(),
                ..checkpoint.clone()
            },
            RecordingWitness {
                session_id: "different".into(),
                ..checkpoint.clone()
            },
            RecordingWitness {
                head: "z".repeat(64),
                ..checkpoint.clone()
            },
            RecordingWitness {
                head: "0".repeat(64),
                ..checkpoint.clone()
            },
            RecordingWitness {
                events: 0,
                ..checkpoint.clone()
            },
            RecordingWitness {
                events: u64::MAX,
                ..checkpoint.clone()
            },
        ] {
            assert!(verify_recording_prefix(&records[..count], &invalid).is_err());
        }
    }
    verify_recording_prefix(&records, &witness()).unwrap();
}

#[test]
fn partial_edits_reordering_and_truncation_are_detected() {
    let original = events(SHELL);
    for index in 0..original.len() {
        let mut changed = original.clone();
        changed[index].timestamp_micros += 1;
        assert!(verify_recording(&changed, &witness()).is_err());
        let mut missing = original.clone();
        missing.remove(index);
        assert!(verify_recording(&missing, &witness()).is_err());
    }
    let mut reversed = original.clone();
    reversed.swap(1, 2);
    assert!(verify_recording(&reversed, &witness()).is_err());
    let mut corrupt_hash = original;
    corrupt_hash[1].hash = "0".repeat(64);
    assert!(verify_recording(&corrupt_hash, &witness()).is_err());
}

#[test]
fn self_consistent_forgery_is_rejected_by_required_off_robot_head() {
    let mut chain = RecordingChain::new();
    let forged: Vec<_> = events(SHELL)
        .into_iter()
        .map(|record| {
            let payload = match record.payload {
                RecordingPayload::Stdin { .. } => RecordingPayload::Stdin {
                    data: "d2hvYW1pCg==".into(),
                },
                payload => payload,
            };
            chain.append(record.timestamp_micros, payload).unwrap()
        })
        .collect();
    // A locally rewritten chain verifies against its own replacement head.
    let replacement = RecordingWitness {
        head: chain.head().into(),
        ..witness()
    };
    verify_recording(&forged, &replacement).unwrap();
    // The retained external head detects a fully rewritten local chain.
    assert!(verify_recording(&forged, &witness()).is_err());
}

#[test]
fn witness_identity_count_and_encoding_are_all_required() {
    let records = events(SHELL);
    for invalid in [
        RecordingWitness {
            session_id: "another-session".into(),
            ..witness()
        },
        RecordingWitness {
            head: String::new(),
            ..witness()
        },
        RecordingWitness {
            head: "z".repeat(64),
            ..witness()
        },
        RecordingWitness {
            events: 5,
            ..witness()
        },
        RecordingWitness {
            events: 0,
            ..witness()
        },
    ] {
        assert!(verify_recording(&records, &invalid).is_err());
    }
    assert!(verify_recording(&[], &witness()).is_err());
}

#[test]
fn builder_rejects_invalid_order_and_retains_last_valid_state() {
    let records = events(SHELL);
    let mut chain = RecordingChain::default();
    assert!(chain.append(0, RecordingPayload::StdinEof).is_err());
    chain.append(100, records[0].payload.clone()).unwrap();
    let head = chain.head().to_owned();
    assert!(chain.append(100, records[0].payload.clone()).is_err());
    assert!(chain.append(99, RecordingPayload::StdinEof).is_err());
    assert!(chain
        .append(100, RecordingPayload::Stdin { data: "???".into() })
        .is_err());
    assert_eq!(chain.head(), head);
    assert_eq!(chain.events(), 1);
    chain
        .append(101, records.last().unwrap().payload.clone())
        .unwrap();
    assert!(chain.append(102, RecordingPayload::StdinEof).is_err());
}

#[test]
fn audit_json_rejects_uncommitted_unknown_fields() {
    let json = SHELL.lines().next().unwrap();
    let modified = format!("{},\"uncommitted\":true}}", &json[..json.len() - 1]);
    assert!(serde_json::from_str::<RecordingEvent>(&modified).is_err());
    assert!(
        serde_json::from_str::<RecordingPayload>(r#"{"type":"stdin_eof","uncommitted":true}"#)
            .is_err()
    );
    let eof = SHELL
        .lines()
        .find(|line| line.contains("stdin_eof"))
        .unwrap();
    let modified = eof.replace(
        r#""type":"stdin_eof""#,
        r#""type":"stdin_eof","uncommitted":true"#,
    );
    assert!(serde_json::from_str::<RecordingEvent>(&modified).is_err());
}

#[test]
fn recording_commits_to_authenticated_principal_and_account_metadata() {
    let mut records = events(EXEC);
    let expected = RecordingWitness {
        session_id: "exec-1".into(),
        head: "3f1352e989697121f70af74c842c8cb73833a47a9a19361995af4b77d5a2055c".into(),
        events: 4,
    };
    if let RecordingPayload::Started { account_id, .. } = &mut records[0].payload {
        *account_id = Some("different-account".into());
    }
    assert!(verify_recording(&records, &expected).is_err());
    let mut records = events(EXEC);
    if let RecordingPayload::Started { principal_id, .. } = &mut records[0].payload {
        *principal_id = "different-device".into();
    }
    assert!(verify_recording(&records, &expected).is_err());
}

#[test]
fn invalid_payloads_preserve_the_last_valid_commitment() {
    use cerulion_ops_proto::{ExecRequest, ExitReason, OutputStream, SessionKind, WindowSize};
    let records = events(SHELL);
    let timestamp = records[0].timestamp_micros;
    for payload in [
        RecordingPayload::Output {
            stream: OutputStream::Pty,
            data: "!!!!".into(),
        },
        RecordingPayload::Resize {
            size: WindowSize {
                rows: 0,
                cols: 80,
                pixel_width: 0,
                pixel_height: 0,
            },
        },
        RecordingPayload::Exit {
            code: Some(0),
            signal: Some(9),
            reason: ExitReason::Exited,
        },
    ] {
        let mut chain = RecordingChain::new();
        chain.append(timestamp, records[0].payload.clone()).unwrap();
        assert!(chain.append(timestamp + 1, payload).is_err());
        assert_eq!(chain.head(), records[0].hash);
        assert_eq!(chain.events(), 1);
        assert_eq!(
            chain
                .append(timestamp, RecordingPayload::StdinEof)
                .unwrap()
                .sequence,
            1
        );
    }
    let RecordingPayload::Started { session, .. } = &records[0].payload else {
        panic!("started oracle");
    };
    for (session_id, principal_id) in [("", "device-1"), ("shell-1", "")] {
        let mut chain = RecordingChain::new();
        assert!(chain
            .append(
                100,
                RecordingPayload::Started {
                    session_id: session_id.into(),
                    principal_id: principal_id.into(),
                    account_id: None,
                    session: session.clone(),
                }
            )
            .is_err());
        assert_eq!(chain.events(), 0);
        assert_eq!(
            chain.head(),
            "0000000000000000000000000000000000000000000000000000000000000000"
        );
    }
    for session in [
        SessionKind::Ops,
        SessionKind::Exec(ExecRequest { argv: vec![] }),
    ] {
        let mut chain = RecordingChain::new();
        assert!(chain
            .append(
                100,
                RecordingPayload::Started {
                    session_id: "shell-1".into(),
                    principal_id: "device-1".into(),
                    account_id: None,
                    session,
                }
            )
            .is_err());
        assert_eq!(chain.events(), 0);
        assert_eq!(
            chain.head(),
            "0000000000000000000000000000000000000000000000000000000000000000"
        );
    }
}
