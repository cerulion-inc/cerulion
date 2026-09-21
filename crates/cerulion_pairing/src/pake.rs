// SPDX-License-Identifier: MIT OR Apache-2.0
//! The CPace fallback pairing ceremony.
//!
//! [CPace](https://datatracker.ietf.org/doc/draft-irtf-cfrg-cpace/) is the
//! CFRG-selected **balanced** PAKE. This module wraps the `pake-cpace` crate
//! (a faithful libsodium port over ristretto255 + HMAC-SHA512) in a state machine
//! that enforces the ceremony policy: **bounded attempts, code-burn, a short
//! TTL, and a single session**, with **both transport identities transcript-bound**
//! and a **key-confirmation** step that detects a wrong code.
//!
//! The ceremony is meant to run *inside* an already-authenticated channel. It is
//! **never auto-negotiated** after a strong-path failure — the higher layer must
//! make that a deliberate user choice; this module only enforces the mechanics.
//!
//! ## Roles
//!
//! - [`CpaceResponder`] (the **robot**) owns the attempt/TTL/burn policy: it is the
//!   single, stateful session. `respond` consumes an attempt; `finish` confirms.
//! - [`CpaceInitiator`] (the **client**) is per-attempt: `begin` then
//!   [`InitiatorAttempt::finish`]. The client re-`begin`s to retry; the robot's
//!   counter is authoritative.
//!
//! ## Message flow (one attempt)
//!
//! ```text
//! initiator.begin()                 -> msg1
//! responder.respond(msg1)           -> msg2, responder_confirm   (attempt consumed)
//! initiator_attempt.finish(msg2, responder_confirm)
//!                                   -> keys, initiator_confirm
//! responder.finish(initiator_confirm)
//!                                   -> keys                        (success)
//! ```
//! A wrong code yields different derived keys on the two sides, so the
//! confirmation tags mismatch → [`PairingError::ConfirmationFailed`].

use pake_cpace::{CPace, Step1Out, STEP1_PACKET_BYTES, STEP2_PACKET_BYTES};

use zeroize::Zeroize;

use crate::crypto;
use crate::error::PairingError;
use crate::format::{AccountId, PrincipalKind, PublicKey};

const CONFIRM_RESPONDER: &[u8] = b"cerulion-pairing:confirm:responder:v1";
const CONFIRM_INITIATOR: &[u8] = b"cerulion-pairing:confirm:initiator:v1";

/// Default ceremony TTL: two minutes.
pub const DEFAULT_TTL_NS: u64 = 120_000_000_000;

/// Ceremony policy knobs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CeremonyConfig {
    /// Attempts before the code is burned. The product SHOULD use 3–5.
    pub max_attempts: u8,
    /// Time-to-live from creation (ns).
    pub ttl_ns: u64,
}

impl Default for CeremonyConfig {
    fn default() -> Self {
        CeremonyConfig {
            max_attempts: 3,
            ttl_ns: DEFAULT_TTL_NS,
        }
    }
}

impl CeremonyConfig {
    /// Validate and build a config. `max_attempts` must be in `1..=5` (the design
    /// range is 3–5) and `ttl_ns` must be non-zero.
    pub fn new(max_attempts: u8, ttl_ns: u64) -> Result<Self, PairingError> {
        if !(1..=5).contains(&max_attempts) {
            return Err(PairingError::InvalidConfig("max_attempts must be in 1..=5"));
        }
        if ttl_ns == 0 {
            return Err(PairingError::InvalidConfig("ttl_ns must be > 0"));
        }
        Ok(CeremonyConfig {
            max_attempts,
            ttl_ns,
        })
    }
}

/// Both transport identities plus a channel-binding context, transcript-bound
/// into every attempt.
#[derive(Clone, Debug)]
pub struct PakeIdentities {
    /// The initiator's (client's) transport key.
    pub initiator_key: PublicKey,
    /// The responder's (robot's) transport key.
    pub responder_key: PublicKey,
    /// Channel-binding material (e.g. a TLS exporter). May be empty.
    pub context: Vec<u8>,
}

impl PakeIdentities {
    /// Bind both transport identities and a channel-binding context.
    pub fn new(initiator_key: PublicKey, responder_key: PublicKey, context: Vec<u8>) -> Self {
        PakeIdentities {
            initiator_key,
            responder_key,
            context,
        }
    }

    fn id_a(&self) -> String {
        crypto::to_hex(&self.initiator_key.0)
    }
    fn id_b(&self) -> String {
        crypto::to_hex(&self.responder_key.0)
    }
    fn ad(&self) -> &[u8] {
        &self.context
    }
}

/// The derived keys from a successful ceremony.
///
/// NOT `Copy` — the key bytes are zeroized on drop (including on the
/// `ConfirmationFailed` path and when a pending attempt is dropped), so the
/// secret does not linger in freed memory.
#[derive(Clone)]
pub struct PairingKeys {
    /// The primary shared session key.
    pub session_key: [u8; 32],
    /// The secondary key (used here for key confirmation; available to the caller
    /// for a second channel direction).
    pub confirm_key: [u8; 32],
}

impl Drop for PairingKeys {
    fn drop(&mut self) {
        self.session_key.zeroize();
        self.confirm_key.zeroize();
    }
}

impl core::fmt::Debug for PairingKeys {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Never print key bytes.
        f.write_str("PairingKeys(<redacted>)")
    }
}

/// An **unforgeable proof** that a CPace ceremony completed successfully for a
/// specific account, minted **only** by [`CpaceResponder::finish`]. Its fields
/// are private, so no caller can hand-construct one to persist a code-paired row
/// without actually running the ceremony (mirrors
/// [`crate::verify::VerifiedPairing`]). It binds the proven `account` and both
/// transport identities, so a confirmation for account A cannot be replayed to
/// persist account B (see [`crate::verify::TrustStore::establish_code_pairing`]):
///
/// ```compile_fail
/// use cerulion_pairing::pake::{CpaceConfirmed, PairingKeys};
/// use cerulion_pairing::format::{AccountId, PublicKey, PrincipalKind};
/// // This literal is COMPLETE (all five fields supplied, and `PairingKeys` is
/// // externally constructible), so its ONLY possible compile error is that
/// // `CpaceConfirmed`'s fields are private. The guard therefore flips to a real
/// // test FAILURE the moment any field is made `pub` — i.e. it actually guards
/// // unforgeability, rather than failing for an unrelated "missing field" reason.
/// let _forged = CpaceConfirmed {
///     account: AccountId([0; 32]),
///     principal_kind: PrincipalKind::Human,
///     initiator_key: PublicKey([0; 32]),
///     responder_key: PublicKey([0; 32]),
///     keys: PairingKeys { session_key: [0u8; 32], confirm_key: [0u8; 32] },
/// };
/// ```
pub struct CpaceConfirmed {
    account: AccountId,
    principal_kind: PrincipalKind,
    initiator_key: PublicKey,
    responder_key: PublicKey,
    keys: PairingKeys,
}

impl CpaceConfirmed {
    /// The account this ceremony was confirmed FOR.
    pub fn account(&self) -> AccountId {
        self.account
    }
    /// Human vs. machine principal.
    pub fn principal_kind(&self) -> PrincipalKind {
        self.principal_kind
    }
    /// The initiator (client) transport key that was proven.
    pub fn initiator_key(&self) -> PublicKey {
        self.initiator_key
    }
    /// The responder (robot) transport key that was proven.
    pub fn responder_key(&self) -> PublicKey {
        self.responder_key
    }
    /// The derived session/confirm keys (zeroized on drop).
    pub fn keys(&self) -> &PairingKeys {
        &self.keys
    }
}

impl core::fmt::Debug for CpaceConfirmed {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CpaceConfirmed")
            .field("account", &self.account)
            .field("principal_kind", &self.principal_kind)
            .finish_non_exhaustive()
    }
}

/// The responder's per-attempt output.
#[derive(Clone, Debug)]
pub struct ResponderResponse {
    /// The CPace step-2 packet to send to the initiator.
    pub msg2: Vec<u8>,
    /// The responder's key-confirmation tag.
    pub responder_confirm: [u8; 32],
}

/// The lifecycle state of a ceremony.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CeremonyState {
    /// Accepting attempts.
    Active,
    /// Completed successfully (terminal, single-session).
    Succeeded,
    /// All attempts exhausted — the code is burned (terminal).
    Burned,
    /// TTL elapsed (terminal).
    Expired,
}

fn map_cpace(_e: pake_cpace::Error) -> PairingError {
    PairingError::PakeProtocol("CPace rejected a message")
}

fn confirm_tag(k2: &[u8; 32], label: &[u8], id: &PakeIdentities) -> [u8; 32] {
    let mut data = Vec::with_capacity(label.len() + 32 + 32 + id.context.len());
    data.extend_from_slice(label);
    data.extend_from_slice(&id.initiator_key.0);
    data.extend_from_slice(&id.responder_key.0);
    data.extend_from_slice(&id.context);
    crypto::hmac_sha256(k2, &data)
}

/// The robot-side stateful ceremony: the single session that enforces the
/// attempt/TTL/burn policy.
pub struct CpaceResponder {
    config: CeremonyConfig,
    identities: PakeIdentities,
    code: String,
    created_at_ns: u64,
    attempts_used: u8,
    state: CeremonyState,
    pending: Option<PairingKeys>,
}

impl CpaceResponder {
    /// Start a new single-session ceremony with the shared `code`.
    pub fn new(
        code: impl Into<String>,
        identities: PakeIdentities,
        config: CeremonyConfig,
        now_ns: u64,
    ) -> Self {
        CpaceResponder {
            config,
            identities,
            code: code.into(),
            created_at_ns: now_ns,
            attempts_used: 0,
            state: CeremonyState::Active,
            pending: None,
        }
    }

    /// The current ceremony state.
    pub fn state(&self) -> CeremonyState {
        self.state
    }
    /// Attempts consumed so far.
    pub fn attempts_used(&self) -> u8 {
        self.attempts_used
    }
    /// Attempts remaining before burn.
    pub fn attempts_remaining(&self) -> u8 {
        self.config.max_attempts.saturating_sub(self.attempts_used)
    }

    /// Process the initiator's `msg1`. **Consumes one attempt** (anti-spam:
    /// counting at `respond` prevents evading the counter by never calling
    /// `finish`). Returns the step-2 packet and the responder confirmation tag.
    pub fn respond(&mut self, msg1: &[u8], now_ns: u64) -> Result<ResponderResponse, PairingError> {
        self.check_live(now_ns)?;
        if self.attempts_used >= self.config.max_attempts {
            self.state = CeremonyState::Burned;
            return Err(PairingError::AttemptsExhausted {
                max: self.config.max_attempts,
            });
        }
        self.attempts_used += 1;

        let packet: &[u8; STEP1_PACKET_BYTES] = msg1
            .try_into()
            .map_err(|_| PairingError::PakeProtocol("bad step1 packet length"))?;
        let step2 = CPace::step2(
            packet,
            &self.code,
            &self.identities.id_a(),
            &self.identities.id_b(),
            Some(self.identities.ad()),
        )
        .map_err(map_cpace)?;
        let mut shared = step2.shared_keys();
        let responder_confirm = confirm_tag(&shared.k2, CONFIRM_RESPONDER, &self.identities);
        // Move the key material into a zeroize-on-drop `PairingKeys` and wipe the
        // transient `SharedKeys` copy immediately (best-effort — pake-cpace's own
        // internal copy is outside our control; see the crate-level zeroize note).
        let keys = PairingKeys {
            session_key: shared.k1,
            confirm_key: shared.k2,
        };
        shared.k1.zeroize();
        shared.k2.zeroize();
        self.pending = Some(keys);
        Ok(ResponderResponse {
            msg2: step2.packet().to_vec(),
            responder_confirm,
        })
    }

    /// Verify the initiator's confirmation tag. On success the ceremony is done
    /// (single session) and mints an unforgeable [`CpaceConfirmed`] proof bound to
    /// `account` + `principal_kind` — the ONLY way to obtain the witness that
    /// [`crate::verify::TrustStore::establish_code_pairing`] requires. On failure
    /// (wrong code) the attempt — already counted at `respond` — stands, the
    /// pending key material is dropped (zeroized), and the ceremony burns if that
    /// was the last one.
    pub fn finish(
        &mut self,
        initiator_confirm: &[u8],
        now_ns: u64,
        account: AccountId,
        principal_kind: PrincipalKind,
    ) -> Result<CpaceConfirmed, PairingError> {
        self.check_live(now_ns)?;
        // On any early return below, `keys` drops here → zeroized.
        let keys = self
            .pending
            .take()
            .ok_or(PairingError::PakeProtocol("finish with no pending attempt"))?;
        let expected = confirm_tag(&keys.confirm_key, CONFIRM_INITIATOR, &self.identities);
        if !crypto::ct_eq(&expected, initiator_confirm) {
            if self.attempts_used >= self.config.max_attempts {
                self.state = CeremonyState::Burned;
            }
            return Err(PairingError::ConfirmationFailed);
        }
        self.state = CeremonyState::Succeeded;
        Ok(CpaceConfirmed {
            account,
            principal_kind,
            initiator_key: self.identities.initiator_key,
            responder_key: self.identities.responder_key,
            keys,
        })
    }

    fn check_live(&mut self, now_ns: u64) -> Result<(), PairingError> {
        match self.state {
            CeremonyState::Succeeded => return Err(PairingError::CeremonyConsumed),
            CeremonyState::Burned => {
                return Err(PairingError::AttemptsExhausted {
                    max: self.config.max_attempts,
                })
            }
            CeremonyState::Expired => return Err(PairingError::PakeExpired),
            CeremonyState::Active => {}
        }
        if now_ns.saturating_sub(self.created_at_ns) > self.config.ttl_ns {
            self.state = CeremonyState::Expired;
            return Err(PairingError::PakeExpired);
        }
        Ok(())
    }
}

/// The client-side initiator: mostly stateless, driven per attempt.
pub struct CpaceInitiator {
    identities: PakeIdentities,
    code: String,
}

impl CpaceInitiator {
    /// Create an initiator for the shared `code`.
    pub fn new(code: impl Into<String>, identities: PakeIdentities) -> Self {
        CpaceInitiator {
            identities,
            code: code.into(),
        }
    }

    /// Begin an attempt: returns the in-flight attempt handle and `msg1` to send.
    /// Each call uses fresh randomness — call again to retry.
    pub fn begin(&self) -> Result<(InitiatorAttempt, Vec<u8>), PairingError> {
        let step1 = CPace::step1(
            &self.code,
            &self.identities.id_a(),
            &self.identities.id_b(),
            Some(self.identities.ad()),
        )
        .map_err(map_cpace)?;
        let msg1 = step1.packet().to_vec();
        Ok((
            InitiatorAttempt {
                step1,
                identities: self.identities.clone(),
            },
            msg1,
        ))
    }
}

/// An in-flight initiator attempt (holds the CPace step-1 context).
pub struct InitiatorAttempt {
    step1: Step1Out,
    identities: PakeIdentities,
}

impl InitiatorAttempt {
    /// Complete the attempt: derive the keys from `msg2`, verify the responder's
    /// confirmation tag (a mismatch is a wrong code), and return the keys plus the
    /// initiator confirmation tag to send back.
    pub fn finish(
        self,
        msg2: &[u8],
        responder_confirm: &[u8],
    ) -> Result<(PairingKeys, Vec<u8>), PairingError> {
        let packet: &[u8; STEP2_PACKET_BYTES] = msg2
            .try_into()
            .map_err(|_| PairingError::PakeProtocol("bad step2 packet length"))?;
        let mut shared = self.step1.step3(packet).map_err(map_cpace)?;
        let expected = confirm_tag(&shared.k2, CONFIRM_RESPONDER, &self.identities);
        if !crypto::ct_eq(&expected, responder_confirm) {
            // Wipe the transient key material before the wrong-code return.
            shared.k1.zeroize();
            shared.k2.zeroize();
            return Err(PairingError::ConfirmationFailed);
        }
        let initiator_confirm = confirm_tag(&shared.k2, CONFIRM_INITIATOR, &self.identities);
        let keys = PairingKeys {
            session_key: shared.k1,
            confirm_key: shared.k2,
        };
        shared.k1.zeroize();
        shared.k2.zeroize();
        Ok((keys, initiator_confirm.to_vec()))
    }
}
