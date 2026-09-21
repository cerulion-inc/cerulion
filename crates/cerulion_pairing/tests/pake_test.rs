// SPDX-License-Identifier: MIT OR Apache-2.0
//! CPace ceremony tests. The PAKE uses fresh ephemeral randomness per session,
//! so the oracle is the *protocol invariant* (both sides agree iff the code and
//! bound identities match), not a fixed byte value — never a self-compare of one
//! run against itself.

mod common;
use common::{acct, pk, sk};

use cerulion_pairing::format::PrincipalKind;
use cerulion_pairing::pake::*;
use cerulion_pairing::PairingError;

const CODE_ACCT: u8 = 0x0A;

fn ids() -> PakeIdentities {
    PakeIdentities::new(pk(&sk(20)), pk(&sk(21)), b"tls-exporter-context".to_vec())
}

/// Run one full attempt; returns the initiator keys + the responder's witness.
fn run_attempt(
    resp: &mut CpaceResponder,
    init: &CpaceInitiator,
    now: u64,
) -> Result<(PairingKeys, CpaceConfirmed), PairingError> {
    let (attempt, msg1) = init.begin().unwrap();
    let r = resp.respond(&msg1, now)?;
    let (init_keys, init_confirm) = attempt.finish(&r.msg2, &r.responder_confirm)?;
    let confirmed = resp.finish(&init_confirm, now, acct(CODE_ACCT), PrincipalKind::Human)?;
    Ok((init_keys, confirmed))
}

#[test]
fn matching_code_derives_the_same_key_on_both_sides() {
    let mut resp = CpaceResponder::new("428193", ids(), CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("428193", ids());
    let (ik, rk) = run_attempt(&mut resp, &init, 0).unwrap();
    assert_eq!(ik.session_key, rk.keys().session_key);
    assert_eq!(ik.confirm_key, rk.keys().confirm_key);
    // A real derived key is not all-zero.
    assert_ne!(ik.session_key, [0u8; 32]);
    // The witness binds the account + identities the ceremony proved.
    assert_eq!(rk.account(), acct(CODE_ACCT));
    assert_eq!(rk.initiator_key(), pk(&sk(20)));
    assert_eq!(rk.responder_key(), pk(&sk(21)));
    assert_eq!(resp.state(), CeremonyState::Succeeded);
}

#[test]
fn two_independent_ceremonies_with_the_same_code_both_succeed_with_distinct_keys() {
    // Fresh randomness => the two sessions agree internally but differ from each
    // other (proves the keys are ephemeral, not a hardcoded constant).
    let mut r1 = CpaceResponder::new("111111", ids(), CeremonyConfig::default(), 0);
    let i1 = CpaceInitiator::new("111111", ids());
    let (a1, b1) = run_attempt(&mut r1, &i1, 0).unwrap();
    assert_eq!(a1.session_key, b1.keys().session_key);

    let mut r2 = CpaceResponder::new("111111", ids(), CeremonyConfig::default(), 0);
    let i2 = CpaceInitiator::new("111111", ids());
    let (a2, _b2) = run_attempt(&mut r2, &i2, 0).unwrap();
    assert_ne!(a1.session_key, a2.session_key);
}

#[test]
fn wrong_code_is_detected_by_key_confirmation() {
    let mut resp = CpaceResponder::new("000000", ids(), CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("999999", ids()); // wrong code
                                                     // The initiator detects the responder-confirm mismatch first.
    let err = run_attempt(&mut resp, &init, 0).unwrap_err();
    assert!(matches!(err, PairingError::ConfirmationFailed));
}

#[test]
fn mismatched_channel_binding_context_fails_confirmation() {
    // Same code, but the two sides bound DIFFERENT channel-binding contexts.
    let resp_ids = PakeIdentities::new(pk(&sk(20)), pk(&sk(21)), b"context-A".to_vec());
    let init_ids = PakeIdentities::new(pk(&sk(20)), pk(&sk(21)), b"context-B".to_vec());
    let mut resp = CpaceResponder::new("123456", resp_ids, CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("123456", init_ids);
    let err = run_attempt(&mut resp, &init, 0).unwrap_err();
    assert!(matches!(err, PairingError::ConfirmationFailed));
}

#[test]
fn mismatched_transport_identity_fails_confirmation() {
    // Same code + context, but the responder key the initiator bound is wrong.
    let resp_ids = PakeIdentities::new(pk(&sk(20)), pk(&sk(21)), b"ctx".to_vec());
    let init_ids = PakeIdentities::new(pk(&sk(20)), pk(&sk(99)), b"ctx".to_vec());
    let mut resp = CpaceResponder::new("123456", resp_ids, CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("123456", init_ids);
    let err = run_attempt(&mut resp, &init, 0).unwrap_err();
    assert!(matches!(err, PairingError::ConfirmationFailed));
}

#[test]
fn initiator_key_only_mismatch_fails_confirmation() {
    // Only the INITIATOR key differs between the two sides (responder + context
    // identical) — the binding is on both identities, so this still fails.
    let resp_ids = PakeIdentities::new(pk(&sk(20)), pk(&sk(21)), b"ctx".to_vec());
    let init_ids = PakeIdentities::new(pk(&sk(99)), pk(&sk(21)), b"ctx".to_vec());
    let mut resp = CpaceResponder::new("123456", resp_ids, CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("123456", init_ids);
    let err = run_attempt(&mut resp, &init, 0).unwrap_err();
    assert!(matches!(err, PairingError::ConfirmationFailed));
}

#[test]
fn swapping_id_a_and_id_b_between_the_sides_fails_confirmation() {
    // Order sensitivity: the responder binds (initiator=A, responder=B); the
    // initiator binds them SWAPPED (initiator=B, responder=A). The transcript
    // encodes id_a then id_b length-prefixed, so a swap yields a different
    // generator → different keys → confirmation fails.
    let a = pk(&sk(20));
    let b = pk(&sk(21));
    let resp_ids = PakeIdentities::new(a, b, b"ctx".to_vec());
    let init_ids = PakeIdentities::new(b, a, b"ctx".to_vec());
    let mut resp = CpaceResponder::new("123456", resp_ids, CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("123456", init_ids);
    let err = run_attempt(&mut resp, &init, 0).unwrap_err();
    assert!(matches!(err, PairingError::ConfirmationFailed));
}

#[test]
fn attempts_burn_after_the_configured_maximum() {
    let cfg = CeremonyConfig::new(3, DEFAULT_TTL_NS).unwrap();
    let mut resp = CpaceResponder::new("secret", ids(), cfg, 0);
    let init = CpaceInitiator::new("secret", ids());

    // Three failed attempts (an adversary sends a garbage confirmation tag).
    for i in 0..3 {
        let (_, msg1) = init.begin().unwrap();
        resp.respond(&msg1, 0).unwrap();
        assert_eq!(resp.attempts_used(), i + 1);
        let err = resp
            .finish(&[0u8; 32], 0, acct(CODE_ACCT), PrincipalKind::Human)
            .unwrap_err();
        assert!(matches!(err, PairingError::ConfirmationFailed));
    }
    assert_eq!(resp.state(), CeremonyState::Burned);
    assert_eq!(resp.attempts_remaining(), 0);

    // Any further attempt is refused — the code is burned.
    let (_, msg1) = init.begin().unwrap();
    let err = resp.respond(&msg1, 0).unwrap_err();
    assert!(matches!(err, PairingError::AttemptsExhausted { max: 3 }));
}

#[test]
fn respond_consumes_an_attempt_even_without_finish() {
    // Anti-spam: an attacker cannot avoid the counter by never calling finish.
    let cfg = CeremonyConfig::new(3, DEFAULT_TTL_NS).unwrap();
    let mut resp = CpaceResponder::new("secret", ids(), cfg, 0);
    let init = CpaceInitiator::new("secret", ids());
    for _ in 0..3 {
        let (_, msg1) = init.begin().unwrap();
        resp.respond(&msg1, 0).unwrap();
    }
    assert_eq!(resp.attempts_used(), 3);
    let (_, msg1) = init.begin().unwrap();
    assert!(matches!(
        resp.respond(&msg1, 0),
        Err(PairingError::AttemptsExhausted { max: 3 })
    ));
}

#[test]
fn ceremony_expires_after_ttl() {
    let cfg = CeremonyConfig::new(3, 100).unwrap(); // 100ns TTL
    let mut resp = CpaceResponder::new("secret", ids(), cfg, 0);
    let init = CpaceInitiator::new("secret", ids());
    let (_, msg1) = init.begin().unwrap();
    // now = 200 > created(0) + ttl(100)
    let err = resp.respond(&msg1, 200).unwrap_err();
    assert!(matches!(err, PairingError::PakeExpired));
    assert_eq!(resp.state(), CeremonyState::Expired);
}

#[test]
fn single_session_is_consumed_after_success() {
    let mut resp = CpaceResponder::new("secret", ids(), CeremonyConfig::default(), 0);
    let init = CpaceInitiator::new("secret", ids());
    run_attempt(&mut resp, &init, 0).unwrap();
    assert_eq!(resp.state(), CeremonyState::Succeeded);
    // A second use of the same session is refused.
    let (_, msg1) = init.begin().unwrap();
    assert!(matches!(
        resp.respond(&msg1, 0),
        Err(PairingError::CeremonyConsumed)
    ));
}

#[test]
fn config_validation() {
    assert!(matches!(
        CeremonyConfig::new(0, 1),
        Err(PairingError::InvalidConfig(_))
    ));
    assert!(matches!(
        CeremonyConfig::new(6, 1),
        Err(PairingError::InvalidConfig(_))
    ));
    assert!(matches!(
        CeremonyConfig::new(3, 0),
        Err(PairingError::InvalidConfig(_))
    ));
    assert!(CeremonyConfig::new(5, DEFAULT_TTL_NS).is_ok());
    let d = CeremonyConfig::default();
    assert_eq!(d.max_attempts, 3);
    assert_eq!(d.ttl_ns, DEFAULT_TTL_NS);
}

#[test]
fn a_recovered_attempt_after_a_failure_still_succeeds() {
    // Fail twice, then the correct code on the third attempt succeeds.
    let cfg = CeremonyConfig::new(3, DEFAULT_TTL_NS).unwrap();
    let mut resp = CpaceResponder::new("goodcode", ids(), cfg, 0);
    let wrong = CpaceInitiator::new("badcode", ids());
    let right = CpaceInitiator::new("goodcode", ids());

    for _ in 0..2 {
        let (_, msg1) = wrong.begin().unwrap();
        resp.respond(&msg1, 0).unwrap();
        let _ = resp.finish(&[0u8; 32], 0, acct(CODE_ACCT), PrincipalKind::Human);
    }
    assert_eq!(resp.state(), CeremonyState::Active);
    // Third attempt with the right code succeeds.
    let (ik, rk) = run_attempt(&mut resp, &right, 0).unwrap();
    assert_eq!(ik.session_key, rk.keys().session_key);
    assert_eq!(resp.state(), CeremonyState::Succeeded);
}
