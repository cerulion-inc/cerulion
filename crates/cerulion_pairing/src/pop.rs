// SPDX-License-Identifier: MIT OR Apache-2.0
//! Proof-of-possession — the device↔account binding gesture.
//!
//! A caller proves it holds the PRIVATE half of a public transport key by signing
//! a **server-issued challenge** with that key. The account service ([issuer]) hands
//! out a fresh, single-use, account-bound challenge; the desk/robot signs a
//! domain-separated message binding `(purpose, account, public_key, challenge)` and
//! presents `(public_key, signature)`; the service recomputes the message and
//! verifies the signature against the presented key ([`verify_pop`]).
//!
//! This closes the **registration-squatting** hole flagged at A2: without a PoP,
//! `POST /v1/robots` bound a PUBLIC transport key with no proof the caller held the
//! private half, so a caller could burn the `one key = one robot owner` slot for a
//! key it did not control. With the PoP the signature is only producible by the key
//! holder, so a key can only be registered by whoever actually owns it (invariant
//! I1 — accounts authorize, device keys authenticate).
//!
//! **One gesture, two sides.** [`sign_pop`] is the client half (the desk/robot CLI),
//! [`verify_pop`] the issuer half (the account service). Both build the SAME bytes
//! via [`pop_signing_message`], so this crate is the single source of the encoding —
//! no drift between the signer and the verifier.
//!
//! [issuer]: crate

use ed25519_dalek::SigningKey;

use crate::crypto::{ed25519_sign, ed25519_verify, VerifyResult};
use crate::format::{PublicKey, Signature};

/// Domain-separation tag for the PoP signing message. Bumped only on a breaking
/// change to the message layout (a signer/verifier compatibility boundary).
pub const POP_DOMAIN: &[u8] = b"cerulion/device-pop/v1";

/// The `purpose` string for a robot-ownership registration PoP (`POST /v1/robots`).
/// Binding the purpose into the signed message means a challenge signed for one
/// context can never be replayed into another (future) PoP-gated endpoint.
pub const PURPOSE_ROBOT_REGISTRATION: &str = "robot-registration";

/// The `purpose` string for a device-key registration PoP (`POST /v1/devices`,
/// A3). DISTINCT from [`PURPOSE_ROBOT_REGISTRATION`] so the two PoP-gated
/// endpoints have separate signing domains: a signature produced for one can never
/// be replayed into the other, even though both consume the same account-bound,
/// single-use challenge from `POST /v1/devices/challenge`.
pub const PURPOSE_DEVICE_REGISTRATION: &str = "device-registration";

/// The outcome of verifying a PoP signature — a structurally invalid key is
/// distinguished from a bad signature so the caller can surface the precise reason.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PopVerification {
    /// The signature is valid for the claimed key over the challenge.
    Ok,
    /// The claimed public key is not a valid ed25519 point (non-canonical /
    /// small-order). Never a valid transport key.
    BadKey,
    /// The key is valid but the signature does not verify — the caller does NOT
    /// hold the private half (or signed a different message).
    BadSignature,
}

impl PopVerification {
    /// Whether the proof succeeded.
    pub fn is_ok(self) -> bool {
        matches!(self, PopVerification::Ok)
    }
}

/// Build the canonical, domain-separated PoP signing message for
/// `(purpose, account, public_key, challenge)`.
///
/// Every field is length-prefixed (u32 little-endian length + bytes) so no field
/// boundary is ambiguous even if a future field is variable-length — a signer and a
/// verifier that agree on the four inputs produce byte-identical output. The
/// `challenge` is the opaque bearer string EXACTLY as the service issued it (its
/// raw UTF-8 bytes are signed — no re-encode, no decode step to disagree on).
pub fn pop_signing_message(
    purpose: &str,
    account: &[u8; 32],
    public_key: &[u8; 32],
    challenge: &str,
) -> Vec<u8> {
    let mut msg =
        Vec::with_capacity(POP_DOMAIN.len() + purpose.len() + 32 + 32 + challenge.len() + 5 * 4);
    write_field(&mut msg, POP_DOMAIN);
    write_field(&mut msg, purpose.as_bytes());
    write_field(&mut msg, account);
    write_field(&mut msg, public_key);
    write_field(&mut msg, challenge.as_bytes());
    msg
}

/// Append a length-prefixed field (u32-LE length, then the bytes).
fn write_field(buf: &mut Vec<u8>, field: &[u8]) {
    buf.extend_from_slice(&(field.len() as u32).to_le_bytes());
    buf.extend_from_slice(field);
}

/// Client half — sign a PoP challenge with the device signing key. The public half
/// of `signing_key` MUST equal `public_key` (the key whose possession is proven);
/// the returned signature is what the caller presents to the PoP-gated endpoint.
pub fn sign_pop(
    signing_key: &SigningKey,
    purpose: &str,
    account: &[u8; 32],
    public_key: &[u8; 32],
    challenge: &str,
) -> Signature {
    ed25519_sign(
        signing_key,
        &pop_signing_message(purpose, account, public_key, challenge),
    )
}

/// Issuer half — verify a PoP signature against the claimed `public_key`. Strict
/// ed25519 verification (rejects non-canonical encodings + small-order keys), over
/// the message rebuilt from the SAME `(purpose, account, challenge)` and the claimed
/// key. A `PopVerification::Ok` proves the caller holds the private half of
/// `public_key`.
pub fn verify_pop(
    public_key: &PublicKey,
    purpose: &str,
    account: &[u8; 32],
    challenge: &str,
    signature: &Signature,
) -> PopVerification {
    let msg = pop_signing_message(purpose, account, &public_key.0, challenge);
    match ed25519_verify(public_key, &msg, signature) {
        VerifyResult::Ok => PopVerification::Ok,
        VerifyResult::BadKey => PopVerification::BadKey,
        VerifyResult::BadSig => PopVerification::BadSignature,
    }
}

/// Domain-separation tag for a ROBOT's proof of possession of its registered
/// transport key. Distinct from [`POP_DOMAIN`], so no device-registration or
/// robot-registration signature is ever a robot proof, and the reverse.
pub const ROBOT_POP_DOMAIN: &[u8] = b"cerulion/robot-pop/v1";

/// The `purpose` of a robot proof that spends a challenge to read the robot's
/// own access epoch (`POST /v1/robots/{robot_id}/access`).
pub const PURPOSE_ROBOT_ACCESS: &str = "robot-access";

/// The `purpose` of a robot proof that asks for a challenge (`POST
/// /v1/robots/{robot_id}/access/challenge`). Its last field is the request
/// time, Unix nanoseconds as decimal digits, in place of a challenge.
pub const PURPOSE_ROBOT_ACCESS_CHALLENGE: &str = "robot-access-challenge";

/// The signing message of a robot proof: the [`pop_signing_message`] layout
/// under [`ROBOT_POP_DOMAIN`], with the robot id where a device proof has the
/// account. Every field is length-prefixed (u32 little-endian); the challenge is
/// its exact UTF-8 bytes as the service issued it.
pub fn robot_pop_signing_message(
    purpose: &str,
    robot_id: &[u8; 32],
    transport_key: &[u8; 32],
    challenge: &str,
) -> Vec<u8> {
    let mut msg = Vec::with_capacity(
        ROBOT_POP_DOMAIN.len() + purpose.len() + 32 + 32 + challenge.len() + 5 * 4,
    );
    write_field(&mut msg, ROBOT_POP_DOMAIN);
    write_field(&mut msg, purpose.as_bytes());
    write_field(&mut msg, robot_id);
    write_field(&mut msg, transport_key);
    write_field(&mut msg, challenge.as_bytes());
    msg
}

/// Robot half: sign a robot access challenge with the transport key whose
/// public half is `transport_key`.
pub fn sign_robot_pop(
    signing_key: &SigningKey,
    purpose: &str,
    robot_id: &[u8; 32],
    transport_key: &[u8; 32],
    challenge: &str,
) -> Signature {
    ed25519_sign(
        signing_key,
        &robot_pop_signing_message(purpose, robot_id, transport_key, challenge),
    )
}

/// Service half of a robot proof: strict ed25519 over the rebuilt message.
pub fn verify_robot_pop(
    transport_key: &PublicKey,
    purpose: &str,
    robot_id: &[u8; 32],
    challenge: &str,
    signature: &Signature,
) -> PopVerification {
    let msg = robot_pop_signing_message(purpose, robot_id, &transport_key.0, challenge);
    match ed25519_verify(transport_key, &msg, signature) {
        VerifyResult::Ok => PopVerification::Ok,
        VerifyResult::BadKey => PopVerification::BadKey,
        VerifyResult::BadSig => PopVerification::BadSignature,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(seed: u8) -> (SigningKey, [u8; 32]) {
        let sk = SigningKey::from_bytes(&[seed; 32]);
        let pk = sk.verifying_key().to_bytes();
        (sk, pk)
    }

    const ACCOUNT: [u8; 32] = [0x11; 32];
    const CHALLENGE: &str = "Zm9vLWJhci1jaGFsbGVuZ2U";

    #[test]
    fn signing_message_is_the_length_prefixed_hand_oracle() {
        // Hand-build the exact bytes the layout must produce (NOT a self-compare):
        // [len(domain)|domain][len(purpose)|purpose][32|account][32|pk][len|challenge].
        let purpose = "robot-registration";
        let account = [0x11u8; 32];
        let pk = [0x22u8; 32];
        let challenge = "abc";
        let mut oracle = Vec::new();
        // Spell the domain-separation tag LITERALLY, not via POP_DOMAIN (a
        // tautology-guard): a silent edit to the constant must make THIS oracle
        // DISAGREE with the function under test, not silently follow along. The
        // literal is the cross-protocol signer/verifier compatibility boundary.
        let domain_literal: &[u8] = b"cerulion/device-pop/v1";
        oracle.extend_from_slice(&(domain_literal.len() as u32).to_le_bytes());
        oracle.extend_from_slice(domain_literal);
        oracle.extend_from_slice(&(purpose.len() as u32).to_le_bytes());
        oracle.extend_from_slice(purpose.as_bytes());
        oracle.extend_from_slice(&32u32.to_le_bytes());
        oracle.extend_from_slice(&account);
        oracle.extend_from_slice(&32u32.to_le_bytes());
        oracle.extend_from_slice(&pk);
        oracle.extend_from_slice(&(challenge.len() as u32).to_le_bytes());
        oracle.extend_from_slice(challenge.as_bytes());
        assert_eq!(
            pop_signing_message(purpose, &account, &pk, challenge),
            oracle
        );
    }

    #[test]
    fn pop_domain_is_the_pinned_literal_tag() {
        // The domain tag is a signer/verifier cross-protocol compatibility boundary
        // (a device key may be reused across protocols) — pin the EXACT bytes so a
        // rename/typo that collides with another protocol's signing domain is a LOUD
        // test failure, never a silent break. Companion to the literal-oracle guard
        // in `signing_message_is_the_length_prefixed_hand_oracle`.
        assert_eq!(POP_DOMAIN, b"cerulion/device-pop/v1");
    }

    #[test]
    fn the_two_registration_purposes_are_distinct() {
        // The device- and robot-registration PoP purposes MUST differ, or a signature
        // for one endpoint could be replayed into the other (both consume the same
        // account-bound single-use challenge — only the purpose separates them).
        assert_ne!(PURPOSE_DEVICE_REGISTRATION, PURPOSE_ROBOT_REGISTRATION);
    }

    #[test]
    fn a_valid_signature_verifies() {
        let (sk, pk) = key(7);
        let sig = sign_pop(&sk, PURPOSE_ROBOT_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        assert_eq!(
            verify_pop(
                &PublicKey(pk),
                PURPOSE_ROBOT_REGISTRATION,
                &ACCOUNT,
                CHALLENGE,
                &sig
            ),
            PopVerification::Ok
        );
    }

    #[test]
    fn a_signature_from_a_different_key_is_rejected() {
        // The squatting attack: sign with a key you DO hold, present it as a
        // DIFFERENT key you do not. verify against the presented (unheld) key fails.
        let (attacker_sk, _attacker_pk) = key(1);
        let (_victim_sk, victim_pk) = key(2);
        let sig = sign_pop(
            &attacker_sk,
            PURPOSE_ROBOT_REGISTRATION,
            &ACCOUNT,
            &victim_pk, // the attacker signs a message CLAIMING the victim key...
            CHALLENGE,
        );
        // ...but the signature was made by the attacker's key, so verifying against
        // the victim key (which the attacker does not hold) fails.
        assert_eq!(
            verify_pop(
                &PublicKey(victim_pk),
                PURPOSE_ROBOT_REGISTRATION,
                &ACCOUNT,
                CHALLENGE,
                &sig
            ),
            PopVerification::BadSignature
        );
    }

    #[test]
    fn a_tampered_challenge_is_rejected() {
        let (sk, pk) = key(7);
        let sig = sign_pop(&sk, PURPOSE_ROBOT_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        assert_eq!(
            verify_pop(
                &PublicKey(pk),
                PURPOSE_ROBOT_REGISTRATION,
                &ACCOUNT,
                "a-different-challenge",
                &sig
            ),
            PopVerification::BadSignature
        );
    }

    #[test]
    fn a_different_account_is_rejected() {
        // The account is bound into the message — a signature made for account A
        // cannot be presented under account B (defense-in-depth over the
        // account-scoped, single-use challenge the service also enforces).
        let (sk, pk) = key(7);
        let sig = sign_pop(&sk, PURPOSE_ROBOT_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        assert_eq!(
            verify_pop(
                &PublicKey(pk),
                PURPOSE_ROBOT_REGISTRATION,
                &[0x99; 32],
                CHALLENGE,
                &sig
            ),
            PopVerification::BadSignature
        );
    }

    #[test]
    fn a_different_purpose_is_rejected() {
        // A challenge signed for one purpose cannot be replayed into another: the
        // SAME account-bound challenge signed for robot-registration does NOT verify
        // when presented for device-registration (the domain split is by design).
        let (sk, pk) = key(7);
        let sig = sign_pop(&sk, PURPOSE_ROBOT_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        assert_eq!(
            verify_pop(
                &PublicKey(pk),
                PURPOSE_DEVICE_REGISTRATION,
                &ACCOUNT,
                CHALLENGE,
                &sig
            ),
            PopVerification::BadSignature
        );
        // ...and the reverse: a device-registration signature does not verify for
        // robot-registration (the separation holds both directions).
        let dev_sig = sign_pop(&sk, PURPOSE_DEVICE_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        assert_eq!(
            verify_pop(
                &PublicKey(pk),
                PURPOSE_ROBOT_REGISTRATION,
                &ACCOUNT,
                CHALLENGE,
                &dev_sig
            ),
            PopVerification::BadSignature
        );
    }

    #[test]
    fn a_small_order_key_is_rejected() {
        // The all-zero key is a small-order point: `from_bytes` ACCEPTS its
        // (canonical) encoding, but `verify_strict` rejects small-order keys — so a
        // signature over it never verifies. The PoP contract is Ok-vs-rejected; this
        // pins that a degenerate key cannot yield a passing proof (whichever precise
        // rejection variant the primitive returns).
        let sig = Signature([0u8; 64]);
        let v = verify_pop(
            &PublicKey([0u8; 32]),
            PURPOSE_ROBOT_REGISTRATION,
            &ACCOUNT,
            CHALLENGE,
            &sig,
        );
        assert!(!v.is_ok(), "a small-order key must not yield a passing PoP");
    }

    #[test]
    fn a_structurally_invalid_key_encoding_is_bad_key() {
        // A non-decompressible point encoding is rejected at `from_bytes` → BadKey,
        // the distinct "not even a valid key" signal (vs a mere signature mismatch).
        // Search for such an encoding deterministically so the test never guesses.
        let mut bad: Option<[u8; 32]> = None;
        for b in 1u8..=255 {
            let mut k = [0u8; 32];
            k[0] = b;
            k[31] = b;
            if ed25519_dalek::VerifyingKey::from_bytes(&k).is_err() {
                bad = Some(k);
                break;
            }
        }
        let bad = bad.expect("some 32-byte pattern must be a non-decompressible point");
        assert_eq!(
            verify_pop(
                &PublicKey(bad),
                PURPOSE_ROBOT_REGISTRATION,
                &ACCOUNT,
                CHALLENGE,
                &Signature([0u8; 64]),
            ),
            PopVerification::BadKey
        );
    }

    #[test]
    fn determinism_two_signs_of_the_same_inputs_are_byte_identical() {
        // ed25519 signing is deterministic (RFC 8032) — the same inputs sign to the
        // same 64 bytes, so a signer + verifier never disagree on a transient nonce.
        let (sk, pk) = key(7);
        let a = sign_pop(&sk, PURPOSE_ROBOT_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        let b = sign_pop(&sk, PURPOSE_ROBOT_REGISTRATION, &ACCOUNT, &pk, CHALLENGE);
        assert_eq!(a.0, b.0);
    }

    /// The vector the hosted account service's own tests pin byte for byte: the
    /// transport key is the RFC 8032 seed 00..1f, the robot id 0x44 x 32.
    const ROBOT_VECTOR_SEED: [u8; 32] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29, 30, 31,
    ];
    const ROBOT_VECTOR_KEY_HEX: &str =
        "03a107bff3ce10be1d70dd18e74bc09967e4d6309ba50d5f1ddc8664125531b8";
    const ROBOT_VECTOR_CHALLENGE: &str = "cm9ib3QtcG9wLWdvbGRlbi12ZWN0b3ItY2hhbGxlbmdl";
    const ROBOT_VECTOR_MESSAGE_HEX: &str = "15000000636572756c696f6e2f726f626f742d706f702f76310c000000726f626f742d6163636573732000000044444444444444444444444444444444444444444444444444444444444444442000000003a107bff3ce10be1d70dd18e74bc09967e4d6309ba50d5f1ddc8664125531b82c000000636d396962335174634739774c5764766247526c626931325a574e3062334974593268686247786c626d646c";
    const ROBOT_VECTOR_SIGNATURE_HEX: &str = "08a47ebe0dbfadd6f1a96210ccfac013b0a92331181d01698548f2e3ff291fea89999715e87d3b896ec21cbf03ad7f0a8158d1e756c5bf568d4137a43a17250f";

    const ROBOT_VECTOR_REQUEST_MESSAGE_HEX: &str = "15000000636572756c696f6e2f726f626f742d706f702f763116000000726f626f742d6163636573732d6368616c6c656e67652000000044444444444444444444444444444444444444444444444444444444444444442000000003a107bff3ce10be1d70dd18e74bc09967e4d6309ba50d5f1ddc8664125531b81300000031373932393732383030303030303030303030";
    const ROBOT_VECTOR_REQUEST_SIGNATURE_HEX: &str = "80cc6d309cece71a44d589d3dcc3e6af14f41fe23d4cb4ba514e2d82dee28b7d46efb3e11b212013d060da15a8e382358384887a0acf127d49eeb11ddc096e02";

    fn unhex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn robot_pop_matches_the_shared_app_vector() {
        let sk = SigningKey::from_bytes(&ROBOT_VECTOR_SEED);
        let pk = sk.verifying_key().to_bytes();
        assert_eq!(pk.to_vec(), unhex(ROBOT_VECTOR_KEY_HEX));
        let robot = [0x44u8; 32];
        // The domain and purpose spelled literally, not through the constants.
        assert_eq!(ROBOT_POP_DOMAIN, b"cerulion/robot-pop/v1");
        assert_eq!(PURPOSE_ROBOT_ACCESS, "robot-access");
        let msg = robot_pop_signing_message("robot-access", &robot, &pk, ROBOT_VECTOR_CHALLENGE);
        assert_eq!(msg, unhex(ROBOT_VECTOR_MESSAGE_HEX));
        let sig = sign_robot_pop(
            &sk,
            PURPOSE_ROBOT_ACCESS,
            &robot,
            &pk,
            ROBOT_VECTOR_CHALLENGE,
        );
        assert_eq!(sig.0.to_vec(), unhex(ROBOT_VECTOR_SIGNATURE_HEX));
        assert_eq!(
            verify_robot_pop(
                &PublicKey(pk),
                PURPOSE_ROBOT_ACCESS,
                &robot,
                ROBOT_VECTOR_CHALLENGE,
                &sig
            ),
            PopVerification::Ok
        );
    }

    #[test]
    fn a_robot_challenge_request_matches_the_shared_app_vector() {
        let sk = SigningKey::from_bytes(&ROBOT_VECTOR_SEED);
        let pk = sk.verifying_key().to_bytes();
        let robot = [0x44u8; 32];
        assert_eq!(PURPOSE_ROBOT_ACCESS_CHALLENGE, "robot-access-challenge");
        let requested = "1792972800000000000";
        let msg = robot_pop_signing_message("robot-access-challenge", &robot, &pk, requested);
        assert_eq!(msg, unhex(ROBOT_VECTOR_REQUEST_MESSAGE_HEX));
        let sig = sign_robot_pop(&sk, PURPOSE_ROBOT_ACCESS_CHALLENGE, &robot, &pk, requested);
        assert_eq!(sig.0.to_vec(), unhex(ROBOT_VECTOR_REQUEST_SIGNATURE_HEX));
        // A request signature is never an access proof over the same text.
        assert!(!verify_robot_pop(
            &PublicKey(pk),
            PURPOSE_ROBOT_ACCESS,
            &robot,
            requested,
            &sig
        )
        .is_ok());
    }

    #[test]
    fn robot_and_device_proofs_never_verify_as_each_other() {
        let (sk, pk) = key(7);
        let robot = [0x44u8; 32];
        let robot_sig = sign_robot_pop(&sk, PURPOSE_ROBOT_ACCESS, &robot, &pk, CHALLENGE);
        // The same 32 bytes as an account, the same purpose string: a device proof.
        assert!(!verify_pop(
            &PublicKey(pk),
            PURPOSE_ROBOT_ACCESS,
            &robot,
            CHALLENGE,
            &robot_sig
        )
        .is_ok());
        let device_sig = sign_pop(&sk, PURPOSE_DEVICE_REGISTRATION, &robot, &pk, CHALLENGE);
        assert!(!verify_robot_pop(
            &PublicKey(pk),
            PURPOSE_DEVICE_REGISTRATION,
            &robot,
            CHALLENGE,
            &device_sig
        )
        .is_ok());
        // Another robot id or challenge does not verify either.
        assert!(!verify_robot_pop(
            &PublicKey(pk),
            PURPOSE_ROBOT_ACCESS,
            &[0x55; 32],
            CHALLENGE,
            &robot_sig
        )
        .is_ok());
        assert!(!verify_robot_pop(
            &PublicKey(pk),
            PURPOSE_ROBOT_ACCESS,
            &robot,
            "other",
            &robot_sig
        )
        .is_ok());
    }
}
