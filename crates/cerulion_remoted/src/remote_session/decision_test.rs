// SPDX-License-Identifier: AGPL-3.0-only
//! Literal boundary oracles for session cancellation, output and receipts.

use super::{delivered_count, final_receipt_outcome, forward_drained_output, termination_reason};
use cerud::receipt::ReceiptOutcome;
use cerulion_ops_proto::session::ExitReason;
use std::time::Duration;

const REASONS: [(ExitReason, &str); 10] = [
    (ExitReason::Exited, "Exited"),
    (ExitReason::ClientClosed, "ClientClosed"),
    (ExitReason::Disconnected, "Disconnected"),
    (ExitReason::Revoked, "Revoked"),
    (ExitReason::ProtocolError, "ProtocolError"),
    (ExitReason::RecordingFailed, "RecordingFailed"),
    (ExitReason::CleanupFailed, "CleanupFailed"),
    (ExitReason::SpawnFailed, "SpawnFailed"),
    (ExitReason::TimedOut, "TimedOut"),
    (ExitReason::ResourceLimit, "ResourceLimit"),
];

#[test]
fn reaching_a_duration_or_idle_limit_takes_precedence_over_revocation() {
    let cases = [
        (true, 9, 7, None, None, None),
        (false, 9, 7, None, None, Some(ExitReason::Revoked)),
        (true, 999, 9, Some(1000), Some(10), None),
        (
            true,
            1000,
            9,
            Some(1000),
            Some(10),
            Some(ExitReason::TimedOut),
        ),
        (
            false,
            1000,
            9,
            Some(1000),
            Some(10),
            Some(ExitReason::TimedOut),
        ),
        (
            true,
            999,
            10,
            Some(1000),
            Some(10),
            Some(ExitReason::TimedOut),
        ),
        (
            false,
            999,
            10,
            Some(1000),
            Some(10),
            Some(ExitReason::TimedOut),
        ),
        (
            false,
            999,
            9,
            Some(1000),
            Some(10),
            Some(ExitReason::Revoked),
        ),
        (true, 0, 0, Some(0), None, Some(ExitReason::TimedOut)),
        (true, 0, 0, None, Some(0), Some(ExitReason::TimedOut)),
        (true, u64::MAX, u64::MAX, None, None, None),
    ];
    for _ in 0..2 {
        for (allowed, elapsed, idle, duration, idle_limit, expected) in cases {
            assert_eq!(
                termination_reason(
                    allowed,
                    Duration::from_millis(elapsed),
                    Duration::from_millis(idle),
                    duration.map(Duration::from_millis),
                    idle_limit.map(Duration::from_millis),
                ),
                expected
            );
        }
    }
}

#[test]
fn output_cap_counts_all_observed_bytes_and_keeps_truncation_sticky() {
    for _ in 0..2 {
        let mut bytes = 0;
        let mut truncated = false;
        for (incoming, delivered, observed, clipped) in [
            (3, 3, 3, false),
            (6, 5, 9, true),
            (0, 0, 9, true),
            (2, 0, 11, true),
        ] {
            assert_eq!(
                delivered_count(&mut bytes, &mut truncated, 8, incoming),
                delivered
            );
            assert_eq!((bytes, truncated), (observed, clipped));
        }
    }
}

#[test]
fn empty_input_and_an_exact_cap_do_not_claim_truncation() {
    let mut bytes = 0;
    let mut truncated = false;
    assert_eq!(delivered_count(&mut bytes, &mut truncated, 0, 0), 0);
    assert_eq!((bytes, truncated), (0, false));
    assert_eq!(delivered_count(&mut bytes, &mut truncated, 8, 8), 8);
    assert_eq!((bytes, truncated), (8, false));
    assert_eq!(delivered_count(&mut bytes, &mut truncated, 8, 0), 0);
    assert_eq!((bytes, truncated), (8, false));
    assert_eq!(delivered_count(&mut bytes, &mut truncated, 8, 1), 0);
    assert_eq!((bytes, truncated), (9, true));
}

#[test]
fn counters_saturate_and_a_lowered_cap_never_wraps_into_an_allowance() {
    let mut bytes = u64::MAX - 1;
    let mut truncated = false;
    assert_eq!(delivered_count(&mut bytes, &mut truncated, u64::MAX, 7), 1);
    assert_eq!((bytes, truncated), (u64::MAX, true));
    assert_eq!(delivered_count(&mut bytes, &mut truncated, u64::MAX, 1), 0);
    assert_eq!((bytes, truncated), (u64::MAX, true));

    let mut bytes = 17;
    let mut truncated = false;
    assert_eq!(delivered_count(&mut bytes, &mut truncated, 3, 4), 0);
    assert_eq!((bytes, truncated), (21, true));
}

#[test]
fn stdout_and_stderr_keep_independent_output_allowances() {
    let (mut stdout, mut stderr) = (0, 0);
    let (mut stdout_truncated, mut stderr_truncated) = (false, false);
    assert_eq!(delivered_count(&mut stdout, &mut stdout_truncated, 2, 3), 2);
    assert_eq!(delivered_count(&mut stderr, &mut stderr_truncated, 7, 5), 5);
    assert_eq!((stdout, stdout_truncated), (3, true));
    assert_eq!((stderr, stderr_truncated), (5, false));
}

#[test]
fn stopped_sessions_never_resume_output_when_leader_exit_arrives() {
    let cases = [
        (ExitReason::Exited, true),
        (ExitReason::CleanupFailed, true),
        (ExitReason::ClientClosed, false),
        (ExitReason::Disconnected, false),
        (ExitReason::Revoked, false),
        (ExitReason::ProtocolError, false),
        (ExitReason::RecordingFailed, false),
        (ExitReason::SpawnFailed, false),
        (ExitReason::TimedOut, false),
        (ExitReason::ResourceLimit, false),
    ];
    for (reason, expected) in cases {
        assert_eq!(forward_drained_output(reason, None), expected);
        for (stopped, _) in cases {
            assert!(!forward_drained_output(reason, Some(stopped)));
        }
    }
}

#[test]
fn witness_delivery_failure_dominates_every_recorded_exit_reason() {
    for (reason, _) in REASONS {
        let ReceiptOutcome::Error(message) = final_receipt_outcome(reason, true) else {
            panic!("a failed witness must never receive an OK receipt");
        };
        assert_eq!(message, "recording_witness_failed");
    }
}

#[test]
fn successful_witness_delivery_preserves_the_exact_failure_class() {
    for _ in 0..2 {
        for (reason, expected) in REASONS {
            match final_receipt_outcome(reason, false) {
                ReceiptOutcome::Ok => assert_eq!(reason, ExitReason::Exited),
                ReceiptOutcome::Error(message) => {
                    assert_ne!(reason, ExitReason::Exited);
                    assert_eq!(message, expected);
                }
                other => panic!("unexpected receipt outcome: {other:?}"),
            }
        }
    }
}
