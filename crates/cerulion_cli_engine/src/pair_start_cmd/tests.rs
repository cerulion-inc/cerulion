// SPDX-License-Identifier: AGPL-3.0-only
//! Oracles for the robot side of code pairing: every decision of
//! [`Machine::next_step`] from hand-written observations, and every text.

use super::*;

#[cfg(unix)]
mod host;
#[cfg(unix)]
mod runs;

use cerulion_pairing::pair_code::{
    REFUSED_MALFORMED, REFUSED_UNCONFIRMED, REFUSED_UNREADABLE, WRITER_LOCK_FILE,
};

const NONCE: PairCodeNonce = PairCodeNonce::from_bytes([7; 16]);
const NONCE_HEX: &str = "07070707070707070707070707070707";
const OTHER_HEX: &str = "08080808080808080808080808080808";
/// 22:13:20 UTC.
const UNIX: u64 = 1_700_000_000;
const CODE: &str = "48213907";

fn eid_hex() -> String {
    "e1".repeat(32)
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

/// A result line, parsed from its hand-written bytes.
fn parsed(nonce: &str, status: &str, reason: &str, failed: u8, desk: &str) -> PairCodeResult {
    let line = format!(
        "cerulion-pair-code-result/1 {nonce} {status} {reason} {} 600 3 {failed} {desk}\n",
        eid_hex()
    );
    PairCodeResult::parse(line.as_bytes()).unwrap()
}

fn seen(nonce: &str, status: &str, failed: u8) -> Option<ResultSeen> {
    Some(ResultSeen::Line(parsed(nonce, status, "-", failed, "- -")))
}

fn refused(nonce: &str, reason: &str) -> Option<ResultSeen> {
    Some(ResultSeen::Line(parsed(nonce, "refused", reason, 0, "- -")))
}

fn paired_seen(failed: u8) -> Option<ResultSeen> {
    Some(ResultSeen::Line(parsed(
        NONCE_HEX,
        "paired",
        "-",
        failed,
        &format!("{} lab-desk", "04".repeat(32)),
    )))
}

fn at(now_ms: u64) -> Observation {
    Observation::at(ms(now_ms), UNIX + now_ms / 1000)
}

fn with_dir(now_ms: u64, dir: DirState) -> Observation {
    Observation {
        dir: Some(dir),
        ..at(now_ms)
    }
}

fn with_lock(now_ms: u64, lock: LockTry) -> Observation {
    Observation {
        lock: Some(lock),
        ..at(now_ms)
    }
}

fn files(now_ms: u64, request_present: bool, result: Option<ResultSeen>) -> Observation {
    Observation {
        request_present: Some(request_present),
        result: result.or(Some(ResultSeen::Gone)),
        ..at(now_ms)
    }
}

fn probed(now_ms: u64, result: Option<ResultSeen>, probe: NetdProbe) -> Observation {
    Observation {
        probe: Some(probe),
        ..files(now_ms, false, result)
    }
}

/// The armed facts of [`armed`]: its window counted from the second of the
/// look at 500 ms that last found the request on disk.
const FACTS: ArmedFacts = ArmedFacts {
    endpoint_id: [0xE1; 32],
    ttl_s: 600,
    tries: 3,
    expires_unix_s: UNIX + 600,
};

/// `refusal` as one before the arm.
fn pre(refusal: PairCodeRefusal) -> PreArmRefusal {
    PreArmRefusal::new(refusal).expect("a refusal before the arm")
}

/// A machine that left its request at 250 ms.
fn left() -> Machine {
    let mut m = Machine::new(NONCE, READY_WAIT);
    assert_eq!(m.next_step(&with_dir(0, DirState::Ready)), Step::Wait);
    assert!(m.wants(ms(0)).lock);
    assert_eq!(m.next_step(&with_lock(250, LockTry::Taken)), Step::Leave);
    assert!(m.holds_lock());
    m
}

/// A machine whose code was shown at 1000 ms.
fn armed() -> Machine {
    let mut m = left();
    assert_eq!(m.next_step(&files(500, true, None)), Step::Wait);
    assert_eq!(
        m.next_step(&files(1000, false, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(FACTS)
    );
    assert!(
        !m.holds_lock(),
        "the lock is released once the code is shown"
    );
    m
}

#[test]
fn a_code_is_the_first_accepted_draw_reduced_and_zero_padded() {
    assert_eq!(code_from_u32s([4_199_999_999]).as_deref(), Some("99999999"));
    assert_eq!(
        code_from_u32s([4_200_000_000, 7]).as_deref(),
        Some("00000007")
    );
    assert_eq!(code_from_u32s([100_000_000]).as_deref(), Some("00000000"));
    let late = std::iter::repeat_n(u32::MAX, 15).chain([12_345_678]);
    assert_eq!(code_from_u32s(late).as_deref(), Some("12345678"));
    assert_eq!(code_from_u32s(std::iter::repeat_n(u32::MAX, 16)), None);
    assert_eq!(code_from_u32s(std::iter::repeat_n(u32::MAX, 100)), None);
    assert_eq!(code_from_u32s([]), None);
    // Two runs over the same draws make the same code.
    assert_eq!(code_from_u32s([31_415_926]), code_from_u32s([31_415_926]));
    // Every code is one the grammar takes.
    for draw in [0, 1, 99_999_999, 4_199_999_999] {
        let code = code_from_u32s([draw]).unwrap();
        cerulion_pairing::pair_code::PairCodeRequest::new(NONCE, code).unwrap();
    }
}

#[test]
fn times_and_counts_are_rendered_in_words() {
    assert_eq!(utc_hh_mm(0), "00:00");
    assert_eq!(utc_hh_mm(86_399), "23:59");
    assert_eq!(utc_hh_mm(1_700_000_000), "22:13");
    assert_eq!(utc_hh_mm(1_800_000_000), "08:00");
    assert_eq!(span_words(600), "10 minutes");
    assert_eq!(span_words(60), "1 minute");
    assert_eq!(span_words(90), "90 seconds");
    assert_eq!(span_words(1), "1 second");
    assert_eq!(count_words(1, "try", "tries"), "1 try");
    assert_eq!(count_words(3, "try", "tries"), "3 tries");
    assert_eq!(HOLD_TTL_S, 790);
    // The numbers `cerulion help pair` states for the robot side.
    assert_eq!(cerulion_pairing::pair_code::CODE_LEN, 8);
    assert_eq!(cerulion_pairing::pair_code::PAIR_CODE_TRIES, 3);
    assert_eq!(PAIR_CODE_WINDOW.as_secs(), 600);
    assert_eq!(RELEAVE_AFTER, ms(2000));
}

#[test]
fn the_kill_switch_refuses_before_netd_is_asked() {
    assert_eq!(network_refusal_from(None), None);
    assert_eq!(network_refusal_from(Some(" \t".into())), None);
    assert_eq!(
        network_refusal_from(Some("off".into())).as_deref(),
        Some(
            "cerulion pair --start makes this robot serve pairing over the network, which \
             CERULION_NETWORK=off disables; unset CERULION_NETWORK to start code pairing"
        )
    );
    assert_eq!(
        network_refusal_from(Some(" OFF ".into())),
        network_refusal_from(Some("off".into()))
    );
    assert_eq!(
        network_refusal_from(Some("typo".into())).as_deref(),
        Some(
            "cerulion pair --start makes this robot serve pairing over the network, which \
             CERULION_NETWORK=typo (an unrecognized value, which fails closed to local-only) \
             disables; unset CERULION_NETWORK to start code pairing"
        )
    );
}

#[test]
fn a_result_file_is_read_only_when_it_is_the_daemons_kind() {
    let good = FileFacts {
        is_file: true,
        nlink: 1,
        uid: 501,
        mode: 0o600,
        len: 512,
    };
    assert_eq!(result_verdict(&good, 501), Verdict::Read);
    let cases = [
        (FileFacts { nlink: 0, ..good }, Verdict::Gone),
        (
            FileFacts {
                nlink: 0,
                mode: 0o644,
                ..good
            },
            Verdict::Gone,
        ),
        (
            FileFacts { nlink: 2, ..good },
            Verdict::Unreadable("it has another hard link"),
        ),
        (
            FileFacts { uid: 502, ..good },
            Verdict::Unreadable("it is not owned by this user"),
        ),
        (
            FileFacts {
                mode: 0o640,
                ..good
            },
            Verdict::Unreadable("it is not owner-only"),
        ),
        (
            FileFacts {
                is_file: false,
                ..good
            },
            Verdict::Unreadable("it is not a regular file"),
        ),
        (
            FileFacts { len: 513, ..good },
            Verdict::Unreadable("it is longer than 512 bytes"),
        ),
    ];
    for (facts, verdict) in cases {
        assert_eq!(result_verdict(&facts, 501), verdict, "{facts:?}");
    }
}

#[test]
fn the_directory_is_waited_for_and_a_wrong_one_refused() {
    let mut m = Machine::new(NONCE, READY_WAIT);
    assert_eq!(
        m.wants(ms(0)),
        Wants {
            dir: true,
            ..Wants::default()
        }
    );
    for now in [0, 250, 500] {
        assert_eq!(m.next_step(&with_dir(now, DirState::Missing)), Step::Wait);
    }
    assert_eq!(m.next_step(&with_dir(1999, DirState::Missing)), Step::Wait);
    assert_eq!(m.next_step(&with_dir(2000, DirState::Missing)), Step::Note);
    assert_eq!(
        m.next_step(&with_dir(2250, DirState::Missing)),
        Step::Wait,
        "once"
    );
    assert_eq!(
        m.next_step(&with_dir(150_000, DirState::Missing)),
        Step::End(Outcome::NeverReady { waited: READY_WAIT })
    );

    let mut m = Machine::new(NONCE, READY_WAIT);
    assert_eq!(
        m.next_step(&with_dir(0, DirState::NotPrivate)),
        Step::End(Outcome::StateDir)
    );
    assert_eq!(m.wants(ms(0)), Wants::default(), "an end wants nothing");
}

#[test]
fn a_held_lock_is_waited_for_and_the_note_shows_once() {
    let mut m = Machine::new(NONCE, READY_WAIT);
    assert_eq!(m.next_step(&with_dir(0, DirState::Ready)), Step::Wait);
    assert!(!m.holds_lock());
    assert_eq!(m.next_step(&with_lock(1000, LockTry::Busy)), Step::Wait);
    assert_eq!(m.next_step(&with_lock(2000, LockTry::Busy)), Step::Note);
    assert_eq!(m.next_step(&with_lock(2250, LockTry::Busy)), Step::Wait);
    assert_eq!(m.next_step(&with_lock(2500, LockTry::Taken)), Step::Leave);
    assert_eq!(
        m.wants(ms(2500)),
        Wants {
            files: true,
            ..Wants::default()
        }
    );
    // The note shown while the lock was busy is not shown again.
    assert_eq!(m.next_step(&files(4000, true, None)), Step::Wait);

    let mut m = Machine::new(NONCE, READY_WAIT);
    m.next_step(&with_dir(0, DirState::Ready));
    assert_eq!(
        m.next_step(&with_lock(150_000, LockTry::Busy)),
        Step::End(Outcome::NeverReady { waited: READY_WAIT })
    );
    let mut m = Machine::new(NONCE, READY_WAIT);
    m.next_step(&with_dir(0, DirState::Ready));
    let fault = Observation {
        fault: Some("Permission denied (os error 13)".into()),
        ..with_lock(250, LockTry::Busy)
    };
    assert_eq!(
        m.next_step(&fault),
        Step::End(Outcome::HandoffFailed {
            error: "Permission denied (os error 13)".into()
        })
    );
}

#[test]
fn a_request_gone_without_an_answer_is_left_again_after_two_polls() {
    let mut m = left();
    assert_eq!(m.next_step(&files(500, true, None)), Step::Wait);
    assert_eq!(m.next_step(&files(1000, false, None)), Step::Wait);
    // The waiting note comes due first, once.
    assert_eq!(m.next_step(&files(2999, false, None)), Step::Note);
    assert_eq!(m.next_step(&files(2999, false, None)), Step::Wait);
    assert_eq!(m.next_step(&files(3000, false, None)), Step::LeaveAgain);
    // The count starts again from the next time it is seen gone.
    assert_eq!(m.next_step(&files(3250, true, None)), Step::Wait);
    assert_eq!(m.next_step(&files(3500, false, None)), Step::Wait);
    assert_eq!(m.next_step(&files(5499, false, None)), Step::Wait);
    assert_eq!(m.next_step(&files(5500, false, None)), Step::LeaveAgain);
    assert!(m.holds_lock());
}

#[test]
fn a_dash_answer_is_taken_only_before_the_own_nonce() {
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, refused("-", "malformed"))),
        Step::End(Outcome::Refused(pre(PairCodeRefusal::Malformed)))
    );
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, refused("-", "unreadable"))),
        Step::End(Outcome::Refused(pre(PairCodeRefusal::Unreadable)))
    );
    // A request still on disk is taken back first.
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, true, refused("-", "malformed"))),
        Step::PullBack
    );
    let back = Observation {
        pulled: Some(Pulled::Back),
        ..files(1000, false, refused("-", "malformed"))
    };
    assert_eq!(
        m.next_step(&back),
        Step::End(Outcome::Refused(pre(PairCodeRefusal::Malformed)))
    );
    // After the code is shown, a `-` answer is a withdrawal.
    let mut m = armed();
    assert_eq!(
        m.next_step(&files(1250, false, refused("-", "malformed"))),
        Step::End(Outcome::Withdrawn)
    );
}

#[test]
fn another_nonce_before_the_own_is_a_replaced_codes_last_result() {
    let mut m = left();
    for status in ["armed", "paired", "expired"] {
        let result = if status == "paired" {
            Some(ResultSeen::Line(parsed(
                OTHER_HEX,
                "paired",
                "-",
                0,
                &format!("{} desk", "04".repeat(32)),
            )))
        } else {
            seen(OTHER_HEX, status, 0)
        };
        assert_eq!(
            m.next_step(&files(500, true, result)),
            Step::Wait,
            "{status}"
        );
    }
    assert_eq!(
        m.next_step(&files(750, true, refused(OTHER_HEX, "stopping"))),
        Step::Wait
    );
    assert_eq!(
        m.next_step(&files(1000, false, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(FACTS)
    );
}

#[test]
fn an_own_answer_with_the_request_left_again_pulls_it_back_before_showing() {
    let mut m = left();
    // The daemon claimed the first copy, the writer left it again, and the
    // answer to the first copy came while the second is on disk.
    assert_eq!(m.next_step(&files(1000, false, None)), Step::Wait);
    assert_eq!(m.next_step(&files(3000, false, None)), Step::LeaveAgain);
    assert_eq!(
        m.next_step(&files(3250, true, seen(NONCE_HEX, "armed", 0))),
        Step::PullBack
    );
    assert!(m.holds_lock(), "the lock stays held through the pull-back");
    for pulled in [Pulled::Back, Pulled::Claimed] {
        let mut m = m.clone();
        let after = Observation {
            pulled: Some(pulled),
            ..files(3250, false, seen(NONCE_HEX, "armed", 0))
        };
        // The answer is to the copy claimed by 1000 ms, so the window counts
        // from the leave at 250 ms, not from this look at 3250 ms.
        assert_eq!(m.next_step(&after), Step::ShowArmed(FACTS), "{pulled:?}");
    }
    // A request that could not be taken back is not pulled again.
    let stuck = Observation {
        pulled: Some(Pulled::Claimed),
        ..files(3250, true, seen(NONCE_HEX, "armed", 0))
    };
    assert!(matches!(m.next_step(&stuck), Step::ShowArmed(_)));
}

#[test]
fn an_own_refusal_before_the_arm_is_refused_and_stopping_settles() {
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, refused(NONCE_HEX, "unconfirmed"))),
        Step::End(Outcome::Refused(pre(PairCodeRefusal::Unconfirmed)))
    );
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, refused(NONCE_HEX, "stopping"))),
        Step::Wait
    );
    assert!(m.wants(ms(1000)).probe, "a settling run probes at once");
    assert_eq!(
        m.next_step(&probed(1000, None, NetdProbe::Unreachable)),
        Step::End(Outcome::NetdStopped)
    );
}

#[test]
fn an_unreadable_result_ends_the_run() {
    let unreadable = Some(ResultSeen::Unreadable("it is not owner-only".into()));
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, unreadable.clone())),
        Step::End(Outcome::UnreadableResult {
            error: "it is not owner-only".into()
        })
    );
    let mut m = armed();
    assert_eq!(
        m.next_step(&files(1250, false, unreadable)),
        Step::End(Outcome::UnreadableResult {
            error: "it is not owner-only".into()
        })
    );
}

#[test]
fn an_armed_code_shows_each_failed_try_then_its_end() {
    let mut m = armed();
    assert_eq!(
        m.wants(ms(1000)),
        Wants {
            files: true,
            probe: true,
            ..Wants::default()
        }
    );
    assert_eq!(
        m.next_step(&files(1250, false, seen(NONCE_HEX, "armed", 1))),
        Step::TriesFailed {
            from: 1,
            to: 1,
            tries: 3
        }
    );
    assert_eq!(
        m.next_step(&files(1250, false, seen(NONCE_HEX, "armed", 1))),
        Step::Wait
    );
    assert_eq!(
        m.next_step(&files(1500, false, None)),
        Step::Wait,
        "gone is no end"
    );
    // A paired result whose try count moved shows the try first.
    assert_eq!(
        m.next_step(&files(1750, false, paired_seen(2))),
        Step::TriesFailed {
            from: 2,
            to: 2,
            tries: 3
        }
    );
    assert_eq!(
        m.next_step(&files(1750, false, paired_seen(2))),
        Step::End(Outcome::Paired {
            label: "lab-desk".into(),
            account: [4; 32]
        })
    );

    // The exhausted line itself announces the last failed try.
    let mut m = armed();
    m.next_step(&files(1250, false, seen(NONCE_HEX, "armed", 2)));
    assert_eq!(
        m.next_step(&files(1500, false, seen(NONCE_HEX, "exhausted", 3))),
        Step::End(Outcome::Exhausted {
            failed: 3,
            tries: 3
        })
    );
    let mut m = armed();
    assert_eq!(
        m.next_step(&files(1500, false, seen(NONCE_HEX, "exhausted", 3))),
        Step::TriesFailed {
            from: 1,
            to: 2,
            tries: 3
        }
    );
    assert_eq!(
        m.next_step(&files(1500, false, seen(NONCE_HEX, "exhausted", 3))),
        Step::End(Outcome::Exhausted {
            failed: 3,
            tries: 3
        })
    );
    let mut m = armed();
    assert_eq!(
        m.next_step(&files(600_000, false, seen(NONCE_HEX, "expired", 0))),
        Step::End(Outcome::Expired {
            failed: 0,
            tries: 3,
            ttl_s: 600
        })
    );
}

#[test]
fn an_armed_code_ends_replaced_withdrawn_or_disarmed() {
    let cases = [
        (seen(OTHER_HEX, "armed", 0), Outcome::Replaced),
        (seen(OTHER_HEX, "expired", 1), Outcome::Replaced),
        (refused(OTHER_HEX, "stopping"), Outcome::Replaced),
        (refused(OTHER_HEX, "unconfirmed"), Outcome::Withdrawn),
        (refused("-", "unreadable"), Outcome::Withdrawn),
        (refused(NONCE_HEX, "unconfirmed"), Outcome::Disarmed),
        (refused(NONCE_HEX, "malformed"), Outcome::Disarmed),
    ];
    for (result, outcome) in cases {
        let mut m = armed();
        assert_eq!(
            m.next_step(&files(1250, false, result.clone())),
            Step::End(outcome),
            "{result:?}"
        );
    }
    // A failed count below the one shown resets the baseline silently.
    let mut m = armed();
    m.next_step(&files(1250, false, seen(NONCE_HEX, "armed", 2)));
    assert_eq!(
        m.next_step(&files(1500, false, seen(NONCE_HEX, "armed", 1))),
        Step::Wait
    );
    assert_eq!(
        m.next_step(&files(1750, false, seen(NONCE_HEX, "armed", 2))),
        Step::TriesFailed {
            from: 2,
            to: 2,
            tries: 3
        }
    );
}

#[test]
fn a_startup_error_after_a_clear_probe_is_remoted_stopping() {
    let ok = NetdProbe::Status {
        startup_error: false,
    };
    let failed = NetdProbe::Status {
        startup_error: true,
    };
    let armed_seen = || seen(NONCE_HEX, "armed", 0);
    // An error before any clear probe is a stale one.
    let mut m = armed();
    assert_eq!(m.next_step(&probed(1000, armed_seen(), failed)), Step::Wait);
    assert!(!m.wants(ms(2999)).probe);
    assert!(m.wants(ms(3000)).probe);
    assert_eq!(m.next_step(&probed(3000, armed_seen(), ok)), Step::Wait);
    assert_eq!(
        m.next_step(&probed(5000, armed_seen(), failed)),
        Step::End(Outcome::RemotedStopped)
    );
    // An unreachable netd alone ends nothing: an orphan remoted serves on.
    let mut m = armed();
    for now in [1000, 3000, 5000, 7000] {
        assert_eq!(
            m.next_step(&probed(now, armed_seen(), NetdProbe::Unreachable)),
            Step::Wait
        );
    }
    // A result end read with the same look wins.
    let mut m = armed();
    m.next_step(&probed(1000, armed_seen(), ok));
    assert_eq!(
        m.next_step(&probed(3000, paired_seen(0), failed)),
        Step::End(Outcome::Paired {
            label: "lab-desk".into(),
            account: [4; 32]
        })
    );
}

#[test]
fn a_stopping_result_settles_on_which_daemon_stopped() {
    let stopping = || refused(NONCE_HEX, "stopping");
    let settle = || {
        let mut m = armed();
        assert_eq!(m.next_step(&files(2000, false, stopping())), Step::Wait);
        assert!(m.wants(ms(2000)).probe);
        assert!(!m.wants(ms(2000)).files);
        m
    };
    let mut m = settle();
    assert_eq!(
        m.next_step(&probed(2000, None, NetdProbe::Unreachable)),
        Step::End(Outcome::NetdStopped)
    );
    let mut m = settle();
    assert_eq!(
        m.next_step(&probed(
            2000,
            None,
            NetdProbe::Status {
                startup_error: true
            }
        )),
        Step::End(Outcome::RemotedStopped)
    );
    let mut m = settle();
    assert_eq!(
        m.next_step(&probed(
            2000,
            None,
            NetdProbe::Status {
                startup_error: false
            }
        )),
        Step::Wait
    );
    assert!(!m.wants(ms(2499)).probe);
    assert!(m.wants(ms(2500)).probe);
    assert_eq!(m.next_step(&at(3999)), Step::Wait);
    assert_eq!(m.next_step(&at(4000)), Step::End(Outcome::RemotedStopped));
}

#[test]
fn a_code_with_no_end_by_its_window_and_margin_ends_the_run() {
    let mut m = armed();
    // Armed at 1 s: the window ends at 601 s, the margin at 631 s.
    assert_eq!(m.next_step(&files(630_999, false, None)), Step::Wait);
    assert_eq!(
        m.next_step(&files(631_000, false, None)),
        Step::End(Outcome::NoEnd)
    );
}

#[test]
fn an_interrupt_takes_the_request_back_and_waits_only_for_a_claimed_one() {
    let interrupted = |now, present, result| Observation {
        interrupted: true,
        ..files(now, present, result)
    };
    // Never claimed: taken back, and the run stops at once.
    let mut m = left();
    assert_eq!(m.next_step(&interrupted(500, true, None)), Step::PullBack);
    let back = Observation {
        pulled: Some(Pulled::Back),
        ..interrupted(500, false, None)
    };
    assert_eq!(m.next_step(&back), Step::End(Outcome::InterruptedBeforeArm));

    // Claimed: the answer may still come, and an armed one is shown.
    let mut m = left();
    assert_eq!(m.next_step(&files(1000, false, None)), Step::Wait);
    assert_eq!(m.next_step(&interrupted(1250, false, None)), Step::Wait);
    assert!(m.holds_lock());
    let shown = m.next_step(&interrupted(1500, false, seen(NONCE_HEX, "armed", 0)));
    assert!(matches!(shown, Step::ShowArmed(_)), "{shown:?}");
    assert_eq!(
        m.next_step(&interrupted(1750, false, seen(NONCE_HEX, "armed", 0))),
        Step::End(Outcome::InterruptedArmed {
            until_unix_s: UNIX + 600
        })
    );

    // Claimed while the pull-back ran, and no answer within the wait.
    let mut m = left();
    assert_eq!(m.next_step(&interrupted(500, true, None)), Step::PullBack);
    let claimed = Observation {
        pulled: Some(Pulled::Claimed),
        ..interrupted(500, false, None)
    };
    assert_eq!(m.next_step(&claimed), Step::Wait);
    assert_eq!(m.next_step(&interrupted(2499, false, None)), Step::Wait);
    assert_eq!(
        m.next_step(&interrupted(2500, false, None)),
        Step::End(Outcome::InterruptedBeforeArm)
    );

    // Before the lock, and while armed.
    let mut m = Machine::new(NONCE, READY_WAIT);
    let early = Observation {
        interrupted: true,
        ..with_dir(0, DirState::Missing)
    };
    assert_eq!(
        m.next_step(&early),
        Step::End(Outcome::InterruptedBeforeArm)
    );
    let mut m = armed();
    assert_eq!(
        m.next_step(&interrupted(5000, false, seen(NONCE_HEX, "armed", 0))),
        Step::End(Outcome::InterruptedArmed {
            until_unix_s: UNIX + 600
        })
    );
}

#[test]
fn the_ready_deadline_takes_the_request_back_and_uses_a_late_answer() {
    // Never claimed: no answer can come.
    let mut m = left();
    assert_eq!(m.next_step(&files(149_999, true, None)), Step::Note);
    assert_eq!(m.next_step(&files(150_000, true, None)), Step::PullBack);
    let back = Observation {
        pulled: Some(Pulled::Back),
        ..files(150_000, false, None)
    };
    assert_eq!(
        m.next_step(&back),
        Step::End(Outcome::NeverReady { waited: READY_WAIT })
    );
    // Claimed: an answer within the closing wait is used.
    let mut m = left();
    m.next_step(&files(149_000, false, None));
    assert_eq!(m.next_step(&files(150_000, false, None)), Step::Wait);
    assert!(matches!(
        m.next_step(&files(151_000, false, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(_)
    ));
    // A shorter hold shortens the wait, and the text says so.
    let mut m = Machine::new(NONCE, Duration::from_secs(60));
    assert_eq!(
        m.next_step(&with_dir(60_000, DirState::Missing)),
        Step::End(Outcome::NeverReady {
            waited: Duration::from_secs(60)
        })
    );
    assert!(never_ready_msg(Duration::from_secs(60)).contains("within 60 s"));
}

/// A machine in its closing wait, its request seen gone at 1000 ms: entered
/// on an interrupt at 1250 ms when `interrupted`, else on the ready deadline
/// at 150 s. Returns it with the end of the wait in milliseconds.
fn closing_after(interrupted: bool) -> (Machine, u64) {
    let mut m = left();
    assert_eq!(m.next_step(&files(1000, false, None)), Step::Wait);
    let now = if interrupted { 1250 } else { 150_000 };
    let entered = Observation {
        interrupted,
        ..files(now, false, None)
    };
    assert_eq!(m.next_step(&entered), Step::Wait);
    assert!(m.holds_lock(), "the lock is held through the closing wait");
    (m, now + 2000)
}

#[test]
fn a_closing_wait_ends_on_each_answer_as_the_reason_it_closed_says() {
    for interrupted in [true, false] {
        let (base, until) = closing_after(interrupted);
        let obs = |now: u64, result: Option<ResultSeen>| Observation {
            interrupted,
            ..files(now, false, result)
        };
        let unarmed = |refusal| {
            if interrupted {
                Outcome::InterruptedBeforeArm
            } else {
                Outcome::Refused(pre(refusal))
            }
        };
        let case = format!("interrupted={interrupted}");
        // An own refusal before the arm.
        let mut m = base.clone();
        assert_eq!(
            m.next_step(&obs(until - 250, refused(NONCE_HEX, "unconfirmed"))),
            Step::End(unarmed(PairCodeRefusal::Unconfirmed)),
            "{case}"
        );
        // A refused answer naming `-`.
        let mut m = base.clone();
        assert_eq!(
            m.next_step(&obs(until - 250, refused("-", "malformed"))),
            Step::End(unarmed(PairCodeRefusal::Malformed)),
            "{case}"
        );
        // Another nonce's result is passed over; the wait still ends at its
        // end.
        let mut m = base.clone();
        assert_eq!(
            m.next_step(&obs(until - 250, seen(OTHER_HEX, "armed", 0))),
            Step::Wait,
            "{case}"
        );
        assert!(m.holds_lock(), "{case}");
        assert_eq!(m.next_step(&obs(until - 1, None)), Step::Wait, "{case}");
        assert_eq!(
            m.next_step(&obs(until, None)),
            Step::End(if interrupted {
                Outcome::InterruptedBeforeArm
            } else {
                Outcome::NeverReady { waited: READY_WAIT }
            }),
            "{case}"
        );
        // An own `stopping`: the code was armed and its daemon stopped, so
        // the run settles on which daemon stopped.
        let mut m = base.clone();
        assert_eq!(
            m.next_step(&obs(until - 250, refused(NONCE_HEX, "stopping"))),
            Step::Wait,
            "{case}"
        );
        assert!(!m.holds_lock(), "{case}");
        assert_eq!(
            m.wants(ms(until - 250)),
            Wants {
                probe: true,
                ..Wants::default()
            },
            "{case}"
        );
        let gone = Observation {
            interrupted,
            probe: Some(NetdProbe::Unreachable),
            ..at(until - 250)
        };
        assert_eq!(
            m.next_step(&gone),
            Step::End(Outcome::NetdStopped),
            "{case}"
        );
    }
}

#[test]
fn an_interrupt_during_a_closing_wait_the_deadline_began_ends_the_run_interrupted() {
    let (base, until) = closing_after(false);
    let interrupted = |now: u64, result: Option<ResultSeen>| Observation {
        interrupted: true,
        ..files(now, false, result)
    };
    // No answer by the end of the wait.
    let mut m = base.clone();
    assert_eq!(m.next_step(&interrupted(until - 1000, None)), Step::Wait);
    assert_eq!(
        m.next_step(&interrupted(until, None)),
        Step::End(Outcome::InterruptedBeforeArm)
    );
    // A refusal read after the interrupt.
    let mut m = base.clone();
    assert_eq!(m.next_step(&interrupted(until - 1000, None)), Step::Wait);
    assert_eq!(
        m.next_step(&files(
            until - 750,
            false,
            refused(NONCE_HEX, "unconfirmed")
        )),
        Step::End(Outcome::InterruptedBeforeArm)
    );
    // An armed answer is shown, and the next look ends the run interrupted.
    let mut m = base;
    assert!(matches!(
        m.next_step(&interrupted(until - 1000, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(_)
    ));
    assert_eq!(
        m.next_step(&interrupted(until - 750, seen(NONCE_HEX, "armed", 0))),
        Step::End(Outcome::InterruptedArmed {
            until_unix_s: UNIX + 600
        })
    );
}

#[test]
fn an_unreadable_answer_in_a_closing_wait_ends_the_run_unreadable() {
    for interrupted in [true, false] {
        let (mut m, until) = closing_after(interrupted);
        let obs = Observation {
            interrupted,
            ..files(
                until - 250,
                false,
                Some(ResultSeen::Unreadable("it is not owner-only".into())),
            )
        };
        assert_eq!(
            m.next_step(&obs),
            Step::End(Outcome::UnreadableResult {
                error: "it is not owner-only".into()
            }),
            "interrupted={interrupted}"
        );
    }
}

#[test]
fn the_window_shown_counts_from_the_last_look_that_found_the_request_before_its_claim() {
    // The first copy is seen on disk at 1000 ms (second UNIX + 1) and gone at
    // 1250 ms; no answer comes, so it is left again at 3250 ms, and that copy
    // is seen on disk at 4500 ms. The answer at 9000 ms is the code the first
    // copy armed, no earlier than 1000 ms.
    let mut m = left();
    assert_eq!(m.next_step(&files(1000, true, None)), Step::Wait);
    assert_eq!(m.next_step(&files(1250, false, None)), Step::Wait);
    assert_eq!(m.next_step(&files(2000, false, None)), Step::Note);
    assert_eq!(m.next_step(&files(3250, false, None)), Step::LeaveAgain);
    assert_eq!(m.next_step(&files(4500, true, None)), Step::Wait);
    let facts = ArmedFacts {
        expires_unix_s: UNIX + 1 + 600,
        ..FACTS
    };
    assert_eq!(
        m.next_step(&files(9000, false, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(facts)
    );
    let interrupted = Observation {
        interrupted: true,
        ..files(9250, false, seen(NONCE_HEX, "armed", 0))
    };
    assert_eq!(
        m.next_step(&interrupted),
        Step::End(Outcome::InterruptedArmed {
            until_unix_s: UNIX + 1 + 600
        })
    );
    // Claimed before the next look: the window counts from the leave.
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1999, false, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(FACTS)
    );
}

#[test]
fn an_end_read_in_the_same_look_as_an_interrupt_is_the_runs_end() {
    let interrupted = |now: u64, result: Option<ResultSeen>| Observation {
        interrupted: true,
        ..files(now, false, result)
    };
    let cases = [
        (
            paired_seen(0),
            Outcome::Paired {
                label: "lab-desk".into(),
                account: [4; 32],
            },
        ),
        (
            seen(NONCE_HEX, "expired", 0),
            Outcome::Expired {
                failed: 0,
                tries: 3,
                ttl_s: 600,
            },
        ),
        (seen(OTHER_HEX, "armed", 0), Outcome::Replaced),
        (refused("-", "malformed"), Outcome::Withdrawn),
        (refused(NONCE_HEX, "unconfirmed"), Outcome::Disarmed),
    ];
    for (result, outcome) in cases {
        let mut m = armed();
        assert_eq!(
            m.next_step(&interrupted(1250, result)),
            Step::End(outcome.clone()),
            "{outcome:?}"
        );
    }
    // Failed tries read in that look are shown first, then the interrupt.
    let mut m = armed();
    assert_eq!(
        m.next_step(&interrupted(1250, seen(NONCE_HEX, "armed", 1))),
        Step::TriesFailed {
            from: 1,
            to: 1,
            tries: 3
        }
    );
    assert_eq!(
        m.next_step(&interrupted(1500, seen(NONCE_HEX, "armed", 1))),
        Step::End(Outcome::InterruptedArmed {
            until_unix_s: UNIX + 600
        })
    );
    // A startup error after a clear probe, in the look of the interrupt.
    let mut m = armed();
    let clear = probed(
        1000,
        None,
        NetdProbe::Status {
            startup_error: false,
        },
    );
    assert_eq!(m.next_step(&clear), Step::Wait);
    let failed = Observation {
        interrupted: true,
        ..probed(
            3000,
            None,
            NetdProbe::Status {
                startup_error: true,
            },
        )
    };
    assert_eq!(m.next_step(&failed), Step::End(Outcome::RemotedStopped));
    // The window and its margin passed with no end read, in the look of the
    // interrupt: armed at 1 s, the margin ends at 631 s.
    let mut m = armed();
    assert_eq!(
        m.next_step(&interrupted(631_000, None)),
        Step::End(Outcome::NoEnd)
    );
    // An answer that cannot be read is no end: the interrupt in its look
    // ends the run.
    let mut m = armed();
    let unreadable = Some(ResultSeen::Unreadable("it is not owner-only".into()));
    assert_eq!(
        m.next_step(&interrupted(1250, unreadable)),
        Step::End(Outcome::InterruptedArmed {
            until_unix_s: UNIX + 600
        })
    );
}

#[test]
fn every_step_after_an_end_is_that_end() {
    let mut m = armed();
    let paired = Outcome::Paired {
        label: "lab-desk".into(),
        account: [4; 32],
    };
    assert_eq!(
        m.next_step(&files(1250, false, paired_seen(0))),
        Step::End(paired.clone())
    );
    assert_eq!(m.wants(ms(1500)), Wants::default());
    for obs in [at(1500), files(1750, false, seen(OTHER_HEX, "armed", 0))] {
        assert_eq!(m.next_step(&obs), Step::End(paired.clone()));
    }
    assert!(!m.holds_lock());
}

#[test]
fn a_look_that_did_not_read_the_files_never_counts_the_request_gone() {
    let mut m = left();
    // Looks with no file facts: the request is neither claimed nor left
    // again, however long they go on.
    for now in [500, 1000, 3000, 6000] {
        let step = m.next_step(&at(now));
        assert!(matches!(step, Step::Wait | Step::Note), "{now}: {step:?}");
    }
    // A deadline with no request ever seen gone takes no closing wait.
    let late = Observation {
        interrupted: true,
        ..at(6250)
    };
    assert_eq!(m.next_step(&late), Step::End(Outcome::InterruptedBeforeArm));
}

#[test]
fn a_refusal_before_the_arm_is_any_refusal_but_stopping() {
    for refusal in [
        PairCodeRefusal::Unreadable,
        PairCodeRefusal::Malformed,
        PairCodeRefusal::Unconfirmed,
    ] {
        assert_eq!(
            PreArmRefusal::new(refusal).map(PreArmRefusal::get),
            Some(refusal)
        );
    }
    assert_eq!(PreArmRefusal::new(PairCodeRefusal::Stopping), None);
}

#[test]
fn an_interrupt_while_settling_ends_the_run_remoted_stopped_unless_netd_is_gone() {
    let clear = NetdProbe::Status {
        startup_error: false,
    };
    let cases = [
        (None, Outcome::RemotedStopped),
        (Some(clear), Outcome::RemotedStopped),
        (Some(NetdProbe::Unreachable), Outcome::NetdStopped),
    ];
    for (probe, outcome) in cases {
        let mut m = armed();
        assert_eq!(
            m.next_step(&files(2000, false, refused(NONCE_HEX, "stopping"))),
            Step::Wait
        );
        // Well before the settle ends at 4000 ms.
        let obs = Observation {
            interrupted: true,
            probe,
            ..at(2250)
        };
        assert_eq!(m.next_step(&obs), Step::End(outcome), "{probe:?}");
    }
}

#[test]
fn an_interrupt_while_the_lock_is_tried_ends_the_run_before_any_request_is_left() {
    for lock in [LockTry::Taken, LockTry::Busy] {
        let mut m = Machine::new(NONCE, READY_WAIT);
        assert_eq!(m.next_step(&with_dir(0, DirState::Ready)), Step::Wait);
        let obs = Observation {
            interrupted: true,
            ..with_lock(250, lock)
        };
        assert_eq!(
            m.next_step(&obs),
            Step::End(Outcome::InterruptedBeforeArm),
            "{lock:?}"
        );
        assert!(!m.holds_lock(), "{lock:?}");
    }
}

#[test]
fn a_first_own_answer_that_is_already_an_end_shows_the_code_its_tries_then_the_end() {
    let exhausted = || seen(NONCE_HEX, "exhausted", 3);
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, exhausted())),
        Step::ShowArmed(FACTS)
    );
    assert_eq!(
        m.next_step(&files(1250, false, exhausted())),
        Step::TriesFailed {
            from: 1,
            to: 2,
            tries: 3
        }
    );
    assert_eq!(
        m.next_step(&files(1500, false, exhausted())),
        Step::End(Outcome::Exhausted {
            failed: 3,
            tries: 3
        })
    );

    let expired = || seen(NONCE_HEX, "expired", 1);
    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, expired())),
        Step::ShowArmed(FACTS)
    );
    assert_eq!(
        m.next_step(&files(1250, false, expired())),
        Step::TriesFailed {
            from: 1,
            to: 1,
            tries: 3
        }
    );
    assert_eq!(
        m.next_step(&files(1500, false, expired())),
        Step::End(Outcome::Expired {
            failed: 1,
            tries: 3,
            ttl_s: 600
        })
    );

    let mut m = left();
    assert_eq!(
        m.next_step(&files(1000, false, paired_seen(0))),
        Step::ShowArmed(FACTS)
    );
    assert_eq!(
        m.next_step(&files(1250, false, paired_seen(0))),
        Step::End(Outcome::Paired {
            label: "lab-desk".into(),
            account: [4; 32]
        })
    );
}

#[test]
fn a_failed_leave_ends_the_handshake_at_once_after_any_request_on_disk_is_taken_back() {
    const ENOSPC: &str = "No space left on device (os error 28)";
    let failed = |now: u64, present: bool, result: Option<ResultSeen>| Observation {
        fault: Some(ENOSPC.into()),
        ..files(now, present, result)
    };
    let handoff_failed = Step::End(Outcome::HandoffFailed {
        error: ENOSPC.into(),
    });
    // The first leave failed: nothing was left.
    let mut m = left();
    assert_eq!(m.next_step(&failed(500, false, None)), handoff_failed);
    assert!(!m.holds_lock());
    // A leave again failed after the first copy was seen gone: no closing
    // wait follows, as no code it might arm is shown.
    let mut m = left();
    assert_eq!(m.next_step(&files(1000, false, None)), Step::Wait);
    assert_eq!(m.next_step(&files(3000, false, None)), Step::LeaveAgain);
    assert_eq!(m.next_step(&failed(3250, false, None)), handoff_failed);
    // A request on disk is taken back first, and the failure is still the
    // end once it is.
    let mut m = left();
    assert_eq!(m.next_step(&failed(500, true, None)), Step::PullBack);
    assert!(m.holds_lock(), "the lock stays held through the pull-back");
    for pulled in [Pulled::Back, Pulled::Claimed] {
        let mut m = m.clone();
        let after = Observation {
            pulled: Some(pulled),
            ..files(500, false, None)
        };
        assert_eq!(m.next_step(&after), handoff_failed, "{pulled:?}");
    }
    // An own answer read in the same look is the answer.
    let mut m = left();
    assert_eq!(
        m.next_step(&failed(1000, false, seen(NONCE_HEX, "armed", 0))),
        Step::ShowArmed(FACTS)
    );
}

/// The success run: every step and its lines, in order.
fn success_run() -> (Report, u8) {
    let mut report = Report::default();
    let mut add = |part: Report| {
        report.stdout.push_str(&part.stdout);
        report.stderr.push_str(&part.stderr);
    };
    add(Report {
        stdout: String::new(),
        stderr: format!("{ASKING_LINE}\n"),
    });
    let mut m = left();
    let Step::ShowArmed(facts) = m.next_step(&files(1000, false, seen(NONCE_HEX, "armed", 0)))
    else {
        panic!("expected the armed lines");
    };
    add(armed_report(CODE, &facts, Some(41234)));
    let Step::End(outcome) = m.next_step(&files(5000, false, paired_seen(0))) else {
        panic!("expected the end");
    };
    add(outcome_report(&outcome, Path::new("/s/remoted")));
    (report, outcome.exit_code())
}

#[test]
fn the_success_transcript_is_byte_exact_and_identical_on_two_runs() {
    let (report, exit) = success_run();
    assert_eq!(exit, 0);
    let eid = eid_hex();
    let account = "04".repeat(32);
    assert_eq!(
        report.stdout,
        format!(
            "pair-code: code=48213907 eid={eid} port=41234 expires_in_s=600 tries=3\n\
             paired: desk=lab-desk account={account}\n"
        )
    );
    assert_eq!(
        report.stderr,
        format!(
            "Asking cerulion-netd to serve this robot for code pairing.\n\
             Pairing code: 48213907\n\
             It pairs one desk and expires at 22:23 UTC (in 10 minutes). All desks share 3 \
             tries.\n\
             On the desk, run `cerulion pair --eid {eid}` and type the code. Add `--addr \
             <robot-ip>:41234` when the desk has no internet.\n\
             Waiting for a desk. Ctrl-C stops waiting; the code keeps working until it is used \
             or expires.\n\
             Paired with desk lab-desk (account {account}). The code is used up.\n"
        )
    );
    assert_eq!(success_run(), (report, exit));
}

#[test]
fn the_exhausted_and_expired_transcripts_are_byte_exact() {
    let dir = Path::new("/s/remoted");
    let mut out = tries_failed_report(1, 2, 3);
    let end = outcome_report(
        &Outcome::Exhausted {
            failed: 3,
            tries: 3,
        },
        dir,
    );
    out.stdout.push_str(&end.stdout);
    out.stderr.push_str(&end.stderr);
    assert_eq!(
        out.stdout,
        "try-failed: failed=1 tries=3\ntry-failed: failed=2 tries=3\nexhausted: failed=3 tries=3\n"
    );
    assert_eq!(
        out.stderr,
        "A pairing try failed: 1 of 3 tries failed.\n\
         A pairing try failed: 2 of 3 tries failed.\n\
         The code was used up by 3 failed tries and no desk paired. Run `cerulion pair --start` \
         again for a new code.\n"
    );
    let expired = outcome_report(
        &Outcome::Expired {
            failed: 0,
            tries: 3,
            ttl_s: 600,
        },
        dir,
    );
    assert_eq!(expired.stdout, "expired: failed=0 tries=3\n");
    assert_eq!(
        expired.stderr,
        "The code expired after 10 minutes and no desk paired. Run `cerulion pair --start` \
         again for a new code.\n"
    );
    assert_eq!(
        armed_report(CODE, &FACTS, None).stdout,
        format!(
            "pair-code: code=48213907 eid={} port=- expires_in_s=600 tries=3\n",
            eid_hex()
        )
    );
}

#[test]
fn every_outcome_has_its_lines_and_exit_code() {
    let dir = Path::new("/s/remoted");
    let cases: Vec<(Outcome, &str, String, u8)> = vec![
        (
            Outcome::Replaced,
            "",
            "Error: a newer `cerulion pair --start` replaced this code, so it no longer works."
                .into(),
            2,
        ),
        (
            Outcome::Withdrawn,
            "",
            "Error: a newer `cerulion pair --start` withdrew this code and the robot armed none \
             for it, so a desk that types this code is told no code is armed. Run `cerulion \
             pair --start` again for a new code."
                .into(),
            2,
        ),
        (
            Outcome::Disarmed,
            "",
            "Error: cerulion-remoted withdrew this code after it was armed, so it no longer \
             works. Run `cerulion pair --start` again for a new code."
                .into(),
            2,
        ),
        (
            Outcome::RemotedStopped,
            "",
            "Error: cerulion-remoted stopped while the code was armed, so the code no longer \
             works. Run `cerulion pair --start` again for a new code."
                .into(),
            3,
        ),
        (
            Outcome::NetdStopped,
            "",
            "Error: cerulion-netd stopped while the code was armed, so the code no longer works. \
             Run `cerulion pair --start` again for a new code."
                .into(),
            3,
        ),
        (
            Outcome::NoEnd,
            "",
            "Error: cerulion-remoted did not say how the code ended by the end of its window, so \
             the code no longer works. Run `cerulion pair --start` again for a new code."
                .into(),
            3,
        ),
        (
            Outcome::NeverReady { waited: READY_WAIT },
            "",
            "Error: cerulion-remoted did not start serving within 150 s, so no code was made. \
             The cerulion-netd log says why it could not start cerulion-remoted; run `cerulion \
             pair --start` again once it can."
                .into(),
            3,
        ),
        (
            Outcome::Refused(pre(PairCodeRefusal::Unconfirmed)),
            "",
            "Error: the robot's daemon did not arm the pairing code: it could not write the \
             result that confirms it. No code was made."
                .into(),
            1,
        ),
        (
            Outcome::Refused(pre(PairCodeRefusal::Malformed)),
            "",
            "Error: cerulion-remoted refused the pairing code request: it is not a well-formed \
             request of the version this daemon reads; use cerulion and cerulion-remoted from \
             the same release. No code was made."
                .into(),
            1,
        ),
        (
            Outcome::StateDir,
            "",
            "Error: the robot state directory /s/remoted is not a private directory of this \
             user, so no code was made: run `cerulion pair --start` as the user cerulion-netd \
             runs as, and keep the directory owner-only."
                .into(),
            1,
        ),
        (
            Outcome::UnreadableResult {
                error: "it is not owner-only".into(),
            },
            "",
            "Error: could not read the answer cerulion-remoted left in \
             /s/remoted/pair-code.result: it is not owner-only. Use cerulion and \
             cerulion-remoted from the same release."
                .into(),
            1,
        ),
        (
            Outcome::HandoffFailed {
                error: "No space left on device (os error 28)".into(),
            },
            "",
            "Error: could not leave the pairing code request in /s/remoted: No space left on \
             device (os error 28). No code was made."
                .into(),
            1,
        ),
        (
            Outcome::InterruptedBeforeArm,
            "",
            "Stopped waiting. No code was made.".into(),
            130,
        ),
        (
            Outcome::InterruptedArmed {
                until_unix_s: UNIX + 600,
            },
            "",
            "Stopped waiting. The code keeps working until 22:23 UTC unless a desk uses it \
             first."
                .into(),
            130,
        ),
        (
            Outcome::Paired {
                label: "lab-desk".into(),
                account: [4; 32],
            },
            "paired: desk=lab-desk account=0404040404040404040404040404040404040404040404040404040404040404",
            "Paired with desk lab-desk (account \
             0404040404040404040404040404040404040404040404040404040404040404). The code is used \
             up."
                .into(),
            0,
        ),
        (
            Outcome::Exhausted { failed: 3, tries: 3 },
            "exhausted: failed=3 tries=3",
            "The code was used up by 3 failed tries and no desk paired. Run `cerulion pair \
             --start` again for a new code."
                .into(),
            4,
        ),
        (
            Outcome::Expired {
                failed: 1,
                tries: 3,
                ttl_s: 60,
            },
            "expired: failed=1 tries=3",
            "The code expired after 1 minute and no desk paired. Run `cerulion pair --start` \
             again for a new code."
                .into(),
            2,
        ),
    ];
    for (outcome, stdout, stderr, exit) in cases {
        let report = outcome_report(&outcome, dir);
        let stdout = if stdout.is_empty() {
            String::new()
        } else {
            format!("{stdout}\n")
        };
        assert_eq!(report.stdout, stdout, "{outcome:?}");
        assert_eq!(report.stderr, format!("{stderr}\n"), "{outcome:?}");
        assert_eq!(outcome.exit_code(), exit, "{outcome:?}");
    }
}

#[test]
fn the_texts_outside_the_machine_are_byte_exact() {
    assert_eq!(
        state_root_mismatch_msg(
            Path::new("/srv/robot"),
            Path::new("/home/r/.cerulion/robot-state")
        ),
        "Error: cerulion-netd serves the robot state at /srv/robot but this command uses \
         /home/r/.cerulion/robot-state; run both with the same CERULION_STATE_ROOT or \
         CERULION_HOME. No code was made."
    );
    assert_eq!(
        NO_STATE_ROOT_MSG,
        "Error: no robot state root is available for this command, so no code was made; set \
         CERULION_STATE_ROOT, CERULION_HOME or HOME."
    );
    assert_eq!(
        no_random_msg("entropy source unavailable"),
        "Error: could not draw a random pairing code: entropy source unavailable. No code was \
         made."
    );
    assert_eq!(
        serving_authority_msg("serving the network requires a prior login; run `cerulion login`"),
        "Error: serving the network requires a prior login; run `cerulion login`. No code was \
         made."
    );
    assert_eq!(
        WAITING_NOTE,
        "Waiting for cerulion-remoted to start serving. A first start on this robot registers \
         it with your account and can take up to 2 minutes."
    );
}

#[cfg(unix)]
mod hold_refusals {
    use std::io;
    use std::path::PathBuf;

    use cerulion_netd::client::{ClientError, ConnectAttempt};
    use cerulion_netd::egress::{
        NO_STATE_ROOT, REMOTE_SERVING_UNAVAILABLE, STATE_ROOT_NOT_ABSOLUTE, STATE_ROOT_NOT_UTF8,
    };
    use cerulion_netd::protocol::SERVING_HOLD_TTL_REFUSAL;

    use super::super::{hold_refusal, HoldRefusal};

    fn text(error: &ClientError, protocol: Option<u32>) -> (String, u8) {
        let refusal = hold_refusal(error, protocol, Some(std::path::Path::new("/x")));
        (refusal.text(), refusal.exit_code())
    }

    #[test]
    fn a_ladder_miss_never_quotes_netds_own_text() {
        let error = ClientError::Spawn {
            bin: PathBuf::from("cerulion-netd"),
            source: io::Error::new(
                io::ErrorKind::NotFound,
                "no `cerulion-netd` found for `/x/cerulion` \u{2014} tried, in order: (1) \
                 /x/cerulion-netd. Install it beside the binary that spawns it, or set \
                 CERULION_NETD_BIN to its path.",
            ),
        };
        assert_eq!(
            text(&error, None),
            (
                "Error: could not ask cerulion-netd to serve this robot: no cerulion-netd was \
                 found beside this cerulion in /x. No code was made. Install cerulion-netd \
                 beside cerulion or set CERULION_NETD_BIN to its path."
                    .into(),
                3
            )
        );
        assert_eq!(
            hold_refusal(&error, None, None).text(),
            "Error: could not ask cerulion-netd to serve this robot: no cerulion-netd was found \
             beside this cerulion. No code was made. Install cerulion-netd beside cerulion or \
             set CERULION_NETD_BIN to its path."
        );
    }

    #[test]
    fn each_netd_failure_has_its_text_and_exit_code() {
        let cases: Vec<(ClientError, Option<u32>, &str, u8)> = vec![
            (
                ClientError::Spawn {
                    bin: PathBuf::from("/opt/netd"),
                    source: io::Error::from_raw_os_error(libc::ENOENT),
                },
                None,
                "Error: could not ask cerulion-netd to serve this robot: could not start \
                 cerulion-netd from /opt/netd: No such file or directory (os error 2). No code \
                 was made. Install cerulion-netd beside cerulion or set CERULION_NETD_BIN to \
                 its path.",
                3,
            ),
            (
                ClientError::Connect {
                    socket: PathBuf::from("/run/n.sock"),
                    attempt: ConnectAttempt::ConnectOrSpawn,
                    source: io::Error::from_raw_os_error(libc::EACCES),
                },
                None,
                "Error: could not ask cerulion-netd to serve this robot: could not reach the \
                 cerulion-netd daemon at /run/n.sock: Permission denied (os error 13). No code \
                 was made.",
                3,
            ),
            (
                ClientError::Connect {
                    socket: PathBuf::from("/run/n.sock"),
                    attempt: ConnectAttempt::ConnectOrSpawn,
                    source: io::Error::new(
                        io::ErrorKind::TimedOut,
                        "waited 10.0s \u{2014} the liveness is UNKNOWN",
                    ),
                },
                None,
                "Error: could not ask cerulion-netd to serve this robot: cerulion-netd was \
                 started but did not open its control socket at /run/n.sock; its startup output \
                 is in /run/n.log. No code was made.",
                3,
            ),
            (
                ClientError::Io(io::Error::from_raw_os_error(libc::EPIPE)),
                None,
                "Error: could not ask cerulion-netd to serve this robot: the connection to \
                 cerulion-netd failed: Broken pipe (os error 32). No code was made.",
                3,
            ),
            (
                ClientError::Io(io::Error::other("closed early")),
                None,
                "Error: could not ask cerulion-netd to serve this robot: the connection to \
                 cerulion-netd failed. No code was made.",
                3,
            ),
            (
                ClientError::Netd {
                    error: REMOTE_SERVING_UNAVAILABLE.into(),
                    robot: None,
                    topic: None,
                },
                Some(12),
                "Error: cerulion-netd will not serve this robot: this cerulion-netd does not \
                 serve remote access: its network is off or it was built without remote \
                 serving; remove CERULION_NETD_NETWORK=off or rebuild it with the default wan \
                 feature, then restart cerulion-netd. No code was made.",
                1,
            ),
            (
                ClientError::Netd {
                    error: cerulion_netd::serving_login::REFUSAL.into(),
                    robot: None,
                    topic: None,
                },
                Some(12),
                "Error: cerulion-netd will not serve this robot: serving the network requires a \
                 prior login; run `cerulion login`. No code was made.",
                1,
            ),
            (
                ClientError::Netd {
                    error: "remote server startup failed: x.".into(),
                    robot: None,
                    topic: None,
                },
                Some(12),
                "Error: cerulion-netd could not serve this robot: remote server startup failed: \
                 x. No code was made.",
                3,
            ),
            (
                ClientError::Protocol("the connected cerulion-netd speaks protocol v11".into()),
                Some(11),
                "Error: could not ask cerulion-netd to serve this robot: the connected \
                 cerulion-netd speaks protocol v11 but the 'hold_serving' verb needs protocol \
                 v12 (this client speaks v12). Upgrade cerulion-netd to use this feature. \
                 Restart cerulion-netd after the upgrade.",
                1,
            ),
            (
                ClientError::Protocol("expected a serving hold response".into()),
                Some(12),
                "Error: could not ask cerulion-netd to serve this robot: its answer does not \
                 follow the protocol this cerulion speaks. Use cerulion and cerulion-netd from \
                 the same release and restart cerulion-netd. No code was made.",
                1,
            ),
            (
                ClientError::Protocol("bad hello".into()),
                None,
                "Error: could not ask cerulion-netd to serve this robot: its answer does not \
                 follow the protocol this cerulion speaks. Use cerulion and cerulion-netd from \
                 the same release and restart cerulion-netd. No code was made.",
                1,
            ),
        ];
        for (error, protocol, expected, exit) in cases {
            assert_eq!(
                text(&error, protocol),
                (expected.to_string(), exit),
                "{error:?}"
            );
        }
        assert!(matches!(
            hold_refusal(
                &ClientError::Netd {
                    error: "x".into(),
                    robot: None,
                    topic: None
                },
                None,
                None
            ),
            HoldRefusal::NetdFailed { .. }
        ));
    }

    #[test]
    fn each_refusal_netd_gives_a_hold_is_will_not_serve_with_exit_1() {
        let cases = [
            (
                REMOTE_SERVING_UNAVAILABLE,
                "Error: cerulion-netd will not serve this robot: this cerulion-netd does not \
                 serve remote access: its network is off or it was built without remote \
                 serving; remove CERULION_NETD_NETWORK=off or rebuild it with the default wan \
                 feature, then restart cerulion-netd. No code was made.",
            ),
            (
                NO_STATE_ROOT,
                "Error: cerulion-netd will not serve this robot: no robot state root is \
                 available, so cerulion-netd cannot serve remote access; set \
                 CERULION_STATE_ROOT, CERULION_HOME or HOME and restart cerulion-netd. No code \
                 was made.",
            ),
            (
                STATE_ROOT_NOT_ABSOLUTE,
                "Error: cerulion-netd will not serve this robot: the robot state root must be \
                 an absolute path with no '.' or '..' components; set CERULION_STATE_ROOT to \
                 such a path and restart cerulion-netd. No code was made.",
            ),
            (
                STATE_ROOT_NOT_UTF8,
                "Error: cerulion-netd will not serve this robot: the robot state root is not a \
                 UTF-8 path, so cerulion-netd cannot report it; set CERULION_STATE_ROOT to a \
                 UTF-8 path and restart cerulion-netd. No code was made.",
            ),
            (
                cerulion_netd::serving_login::REFUSAL,
                "Error: cerulion-netd will not serve this robot: serving the network requires a \
                 prior login; run `cerulion login`. No code was made.",
            ),
            (
                SERVING_HOLD_TTL_REFUSAL,
                "Error: cerulion-netd will not serve this robot: hold_serving needs ttl_s from \
                 1 to 900 seconds. No code was made.",
            ),
        ];
        for (netd, expected) in cases {
            let error = ClientError::Netd {
                error: netd.into(),
                robot: None,
                topic: None,
            };
            assert_eq!(
                hold_refusal(&error, Some(12), None),
                HoldRefusal::WillNotServe { netd: netd.into() },
                "{netd}"
            );
            assert_eq!(text(&error, Some(12)), (expected.to_string(), 1), "{netd}");
        }
        // A retired mirror is no answer to a hold.
        let lost = ClientError::MirrorLost(cerulion_netd::protocol::MirrorLost {
            event: "mirror_lost".into(),
            robot: "arm".into(),
            topic: "/vector".into(),
        });
        assert_eq!(
            hold_refusal(&lost, Some(12), None),
            HoldRefusal::NetdProtocol
        );
        assert_eq!(text(&lost, Some(12)).1, 1);
    }

    /// Every hold refusal's text, for the sweep below.
    pub(super) fn every_text() -> Vec<String> {
        [
            HoldRefusal::NotFound { exe_dir: None },
            HoldRefusal::NotFound {
                exe_dir: Some(PathBuf::from("/x")),
            },
            HoldRefusal::CannotStart {
                bin: PathBuf::from("/opt/netd"),
                os: "No such file or directory (os error 2)".into(),
            },
            HoldRefusal::Unreachable {
                socket: PathBuf::from("/run/n.sock"),
                os: "Permission denied (os error 13)".into(),
            },
            HoldRefusal::NoSocket {
                socket: PathBuf::from("/run/n.sock"),
                log: PathBuf::from("/run/n.log"),
            },
            HoldRefusal::ConnectionFailed { os: None },
            HoldRefusal::ConnectionFailed {
                os: Some("Broken pipe (os error 32)".into()),
            },
            HoldRefusal::WillNotServe {
                netd: REMOTE_SERVING_UNAVAILABLE.into(),
            },
            HoldRefusal::WillNotServe {
                netd: NO_STATE_ROOT.into(),
            },
            HoldRefusal::WillNotServe {
                netd: STATE_ROOT_NOT_ABSOLUTE.into(),
            },
            HoldRefusal::WillNotServe {
                netd: STATE_ROOT_NOT_UTF8.into(),
            },
            HoldRefusal::WillNotServe {
                netd: cerulion_netd::serving_login::REFUSAL.into(),
            },
            HoldRefusal::WillNotServe {
                netd: SERVING_HOLD_TTL_REFUSAL.into(),
            },
            HoldRefusal::NetdFailed { error: "x".into() },
            HoldRefusal::TooOld { daemon: 11 },
            HoldRefusal::NetdProtocol,
        ]
        .iter()
        .map(HoldRefusal::text)
        .collect()
    }
}

/// Every text this module renders, with sample fields.
fn every_text() -> Vec<String> {
    let dir = Path::new("/s/remoted");
    let mut texts: Vec<String> = [
        ASKING_LINE,
        WAITING_NOTE,
        WAITING_FOR_DESK_LINE,
        REPLACED_MSG,
        WITHDRAWN_MSG,
        DISARMED_MSG,
        NETD_STOPPED_MSG,
        NO_END_MSG,
        NO_STATE_ROOT_MSG,
        INTERRUPTED_BEFORE_ARM_LINE,
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    texts.push(remoted_stopped_msg());
    texts.push(never_ready_msg(READY_WAIT));
    for refusal in [
        PairCodeRefusal::Unreadable,
        PairCodeRefusal::Malformed,
        PairCodeRefusal::Unconfirmed,
    ] {
        texts.push(refused_msg(pre(refusal)));
    }
    texts.push(state_root_mismatch_msg(Path::new("/a"), Path::new("/b")));
    texts.push(state_dir_msg(dir));
    texts.push(unreadable_result_msg(
        Path::new("/s/remoted/pair-code.result"),
        "e",
    ));
    texts.push(handoff_failed_msg(dir, "e"));
    texts.push(no_random_msg("e"));
    texts.push(serving_authority_msg(
        "serving the network requires a prior login; run `cerulion login`",
    ));
    texts.extend(network_refusal_from(Some("off".into())));
    texts.extend(network_refusal_from(Some("typo".into())));
    let armed = armed_report(CODE, &FACTS, Some(1));
    texts.push(armed.stdout);
    texts.push(armed.stderr);
    let tried = tries_failed_report(1, 2, 3);
    texts.push(tried.stdout);
    texts.push(tried.stderr);
    for outcome in [
        Outcome::Paired {
            label: "d".into(),
            account: [4; 32],
        },
        Outcome::Exhausted {
            failed: 3,
            tries: 3,
        },
        Outcome::Expired {
            failed: 0,
            tries: 3,
            ttl_s: 600,
        },
        Outcome::InterruptedArmed { until_unix_s: UNIX },
    ] {
        let report = outcome_report(&outcome, dir);
        texts.push(report.stdout);
        texts.push(report.stderr);
    }
    #[cfg(unix)]
    texts.extend(hold_refusals::every_text());
    texts
}

/// Whether `text` ends a sentence twice: a `..` before a space, a line end or
/// the end of the text. A quoted `'..'` path component is not one.
fn ends_a_sentence_twice(text: &str) -> bool {
    text.contains(".. ") || text.contains("..\n") || text.ends_with("..")
}

#[test]
fn a_doubled_sentence_end_is_told_from_a_quoted_parent_component() {
    for doubled in [
        "restart cerulion-netd.. No code was made.",
        "No code was made..",
        "No code was made..\nRun it again.",
    ] {
        assert!(ends_a_sentence_twice(doubled), "{doubled}");
    }
    for single in [
        "no '.' or '..' components; set CERULION_STATE_ROOT",
        "No code was made.\n",
        "No code was made.",
    ] {
        assert!(!ends_a_sentence_twice(single), "{single}");
    }
}

#[test]
fn every_text_is_plain_ascii_with_one_remedy_at_most() {
    for text in every_text() {
        assert!(text.is_ascii(), "{text}");
        assert!(!ends_a_sentence_twice(&text), "{text}");
        assert!(text.matches("Install cerulion-netd").count() <= 1, "{text}");
        assert!(!text.contains("  "), "{text}");
    }
    for text in [REFUSED_UNREADABLE, REFUSED_MALFORMED, REFUSED_UNCONFIRMED] {
        assert!(text.is_ascii(), "{text}");
    }
}

#[test]
fn every_command_a_text_names_is_listed() {
    let mut named = std::collections::BTreeSet::new();
    for text in every_text() {
        for (i, span) in text.split('`').enumerate() {
            if i % 2 == 1 && span.starts_with("cerulion ") {
                // The robot's endpoint id is filled in at run time.
                named.insert(span.replace(&eid_hex(), "<eid>"));
            }
        }
    }
    let listed: std::collections::BTreeSet<String> =
        COMMANDS_NAMED.iter().map(|s| s.to_string()).collect();
    assert_eq!(named, listed);
}

#[test]
fn the_desk_hint_names_the_endpoint_and_the_port_when_known() {
    let eid = eid_hex();
    assert_eq!(
        desk_hint_line(&[0xE1; 32], Some(41234)),
        format!(
            "On the desk, run `cerulion pair --eid {eid}` and type the code. Add `--addr \
             <robot-ip>:41234` when the desk has no internet."
        )
    );
    assert_eq!(
        desk_hint_line(&[0xE1; 32], None),
        format!("On the desk, run `cerulion pair --eid {eid}` and type the code.")
    );
    let lines: Vec<String> = armed_report(CODE, &FACTS, None)
        .stderr
        .lines()
        .map(str::to_string)
        .collect();
    assert_eq!(
        lines,
        vec![
            code_line(CODE),
            expiry_line(&FACTS),
            desk_hint_line(&FACTS.endpoint_id, None),
            WAITING_FOR_DESK_LINE.to_string(),
        ],
        "the hint goes between the expiry and the waiting lines"
    );
}

#[test]
fn the_lock_file_is_not_a_handoff_file() {
    assert!(!cerulion_pairing::pair_code::is_handoff_file(
        WRITER_LOCK_FILE
    ));
}
