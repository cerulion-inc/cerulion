// SPDX-License-Identifier: AGPL-3.0-only
//! The pairing/claim BOOTSTRAP verbs + the e-stop floor verb.
//!
//! These are `cerud` [`VerbHandler`]s that live HERE (not in `cerud`) because
//! their proof is bound to the trust state — the [`SharedTrust`] and the pairing
//! crypto both live robot-side in `cerulion_remoted`, and `cerud` stays
//! deliberately dumb (no `cerulion_pairing` type leaks into its core protocol).
//! The seam is the generic caller-aware [`VerbHandler::execute_with_caller`]: the
//! ops server dispatches every verb through it, so a verb whose proof IS the
//! transport-authenticated device key (all of these) reads the caller identity.
//!
//! ## Self-gating (exempt from the access list)
//!
//! The [`crate::authorizer::PairingAuthorizer`] classifies `claim` / `pair` /
//! `code-pair-*` as bootstrap → Allow (an UNPAIRED but TLS-authed key reaches
//! ONLY these; an UNCLAIMED robot reaches ONLY `claim`). Each verb then self-gates
//! on its OWN proof: the chassis secret (`claim`), an offline cert chain
//! (`pair`), or an unforgeable `CpaceConfirmed` witness (`code-pair`). None trusts
//! a bare account id — the proof binds the account.
//!
//! ## The authenticated device key is the pairing subject
//!
//! Every verb binds the TLS-authenticated `remote_id` (the caller's device key) as
//! the pairing subject — for `pair` it is the `authenticated_peer_key` the cert
//! chain is asserted against; for `claim`/`code-pair` it is the key bound to the
//! account in the side-map. The device key is NEVER read from client args.
//!
//! ## Trusted clock
//!
//! Certificate validity and anti-rollback are evaluated against
//! [`RemotedClock`], the robot's trusted clock — never a client-supplied time.
//! An armed pairing code's window and the rate interval between tries run on
//! the code-pairing store's [`MonotonicClock`], which a step of the wall clock
//! does not move ([`SharedCodePairSessions`]).

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
#[cfg(any(test, feature = "test-seam"))]
use std::time::Duration;

use cerud::error::{CerudError, CerudResult};
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;

use cerulion_pairing::format::{
    AccountId, Delegation, PrincipalKind, PublicKey, SignedDeviceCert, SignedGrant,
    SignedIntermediateCert,
};
use cerulion_pairing::pair_code::{
    PairCodeReport, PairCodeState, PairedDesk, PAIR_CODE_RATE_INTERVAL,
};
use cerulion_pairing::pake::{CeremonyConfig, CpaceConfirmed, CpaceResponder, PakeIdentities};
use cerulion_pairing::verify::{
    OwnerCertificatePresentationWire, OwnerGrantPresentation, PairingPresentation,
};
use cerulion_pairing::PairingError;

use crate::clock::{MonotonicClock, RemotedClock};
use crate::estop_lease::EstopLease;
use crate::flashback::{EstopCaptureAsk, TransportAsk};
use crate::rcp::RcpdEstop;
use crate::trust::{SharedTrust, CODE_PAIR_SCOPE};

// ── verb-name constants (mirror `crate::verbs`) ─────────────────────────────

use crate::verbs;

// ── wire helpers ────────────────────────────────────────────────────────────

/// Extract the transport-authenticated 32-byte device key from a caller. The
/// authorizer already gated bootstrap verbs on this, so a failure here is
/// defense-in-depth (a loud, fail-closed refusal), never a production path.
pub(crate) fn caller_device_key(caller: &CallerIdentity) -> CerudResult<[u8; 32]> {
    if !caller.authenticated {
        return Err(CerudError::Verb(
            "pairing verb reached without a transport-authenticated caller (the remote \
             plane admits only TLS-authed iroh peers)"
                .to_string(),
        ));
    }
    let bytes = hex::decode(&caller.id)
        .map_err(|e| CerudError::Verb(format!("caller id is not valid hex: {e}")))?;
    bytes.try_into().map_err(|v: Vec<u8>| {
        CerudError::Verb(format!(
            "caller id is {} bytes, not a 32-byte device key",
            v.len()
        ))
    })
}

/// Decode a hex-encoded 32-byte value from a required JSON string field.
fn hex32_field(args: &serde_json::Value, field: &str) -> CerudResult<[u8; 32]> {
    let s = args
        .get(field)
        .and_then(|v| v.as_str())
        .ok_or_else(|| CerudError::Verb(format!("missing required string field '{field}'")))?;
    let bytes = hex::decode(s)
        .map_err(|e| CerudError::Verb(format!("field '{field}' is not valid hex: {e}")))?;
    bytes.try_into().map_err(|v: Vec<u8>| {
        CerudError::Verb(format!("field '{field}' is {} bytes, expected 32", v.len()))
    })
}

/// Decode a hex-encoded byte string from a required JSON string field.
fn hex_bytes_field(args: &serde_json::Value, field: &str) -> CerudResult<Vec<u8>> {
    let s = args
        .get(field)
        .and_then(|v| v.as_str())
        .ok_or_else(|| CerudError::Verb(format!("missing required string field '{field}'")))?;
    hex::decode(s).map_err(|e| CerudError::Verb(format!("field '{field}' is not valid hex: {e}")))
}

/// A required JSON string field.
fn str_field(args: &serde_json::Value, field: &str) -> CerudResult<String> {
    args.get(field)
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .ok_or_else(|| CerudError::Verb(format!("missing required string field '{field}'")))
}

/// Parse an optional `principal_kind` string (`"human"` / `"machine"`), defaulting
/// to [`PrincipalKind::Human`].
fn principal_kind_field(args: &serde_json::Value) -> CerudResult<PrincipalKind> {
    match args.get("principal_kind").and_then(|v| v.as_str()) {
        None | Some("human") => Ok(PrincipalKind::Human),
        Some("machine") => Ok(PrincipalKind::Machine),
        Some(other) => Err(CerudError::Verb(format!(
            "principal_kind '{other}' is not 'human' or 'machine'"
        ))),
    }
}

/// The loud, fail-closed error returned by a bootstrap verb's caller-less
/// [`VerbHandler::execute`]. These verbs are DISPATCHED via
/// [`VerbHandler::execute_with_caller`] (the ops server always uses it), so this
/// is unreachable in production; it exists so a future code path that called
/// `execute` directly fails LOUDLY rather than running without the authenticated
/// device key it needs.
fn requires_caller(verb: &str) -> CerudError {
    CerudError::Verb(format!(
        "verb '{verb}' is a caller-bound bootstrap verb and must be dispatched with the \
         authenticated peer identity (via execute_with_caller); it cannot run without it"
    ))
}

// ── the strong-path presentation wire shape ─────────────────────────────────

/// The serde-transportable `pair` argument: a [`PairingPresentation`]'s four
/// fields. Kept a `cerulion_remoted`-local shape so the wire form is owned here
/// and `cerulion_pairing` needs no change.
///
/// It rides a **postcard** blob (hex-encoded in the JSON arg), NOT raw JSON: the
/// pairing crate's cert/grant/`Signature` types are BYTE-oriented serde (an
/// ed25519 `Signature` (de)serializes as raw bytes) and do NOT round-trip through
/// serde_json (no native bytes type). Postcard is the deterministic, compact
/// format the trust store itself persists.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PresentationWire {
    /// The root-signed intermediate certificate.
    pub intermediate: SignedIntermediateCert,
    /// The device certificate (its key must equal the authenticated peer key).
    pub device_cert: SignedDeviceCert,
    /// The access grant for this robot.
    pub grant: SignedGrant,
    /// Present only for a depth-1 (delegated) grant.
    #[serde(default)]
    pub delegation: Option<Delegation>,
}

impl PresentationWire {
    /// Reconstruct the pairing crate's [`PairingPresentation`].
    pub fn into_presentation(self) -> PairingPresentation {
        PairingPresentation {
            intermediate: self.intermediate,
            device_cert: self.device_cert,
            grant: self.grant,
            delegation: self.delegation,
        }
    }

    /// Encode this presentation as a postcard blob, hex-encoded — the exact
    /// `presentation_postcard` value the `pair` verb decodes. (The desk client +
    /// the tests build a presentation and hand it to `pair` via this.)
    pub fn to_postcard_hex(&self) -> Result<String, CerudError> {
        let bytes = postcard::to_stdvec(self)
            .map_err(|e| CerudError::Verb(format!("encoding the presentation failed: {e}")))?;
        Ok(hex::encode(bytes))
    }

    /// Decode a `presentation_postcard` hex blob into a [`PairingPresentation`],
    /// erroring loudly (never fail-open) on bad hex or a malformed blob.
    pub fn from_postcard_hex(s: &str) -> Result<PairingPresentation, CerudError> {
        let bytes = hex::decode(s).map_err(|e| {
            CerudError::Verb(format!("presentation_postcard is not valid hex: {e}"))
        })?;
        let wire: PresentationWire = postcard::from_bytes(&bytes).map_err(|e| {
            CerudError::Verb(format!("presentation did not decode (postcard): {e}"))
        })?;
        Ok(wire.into_presentation())
    }
}

// ── the owner-signed access grant wire shape ─────────────────────────────────

/// The `present-grant` argument shape. The CANONICAL serde struct lives in
/// `cerulion_pairing` ([`cerulion_pairing::verify::OwnerGrantPresentationWire`]) so
/// the desk carriage (`cerulion_wireclient`) and this robot-side verb share ONE byte
/// layout with no package cycle — this is a re-export, NOT a second (drift-prone)
/// definition.
pub use cerulion_pairing::verify::OwnerGrantPresentationWire as OwnerGrantWire;

/// Decode a `grant_postcard` hex blob into an [`OwnerGrantPresentation`], erroring
/// loudly (never fail-open) on bad hex or a malformed blob. Wraps the shared
/// [`OwnerGrantWire::from_postcard`] with the verb's `CerudError` shape.
fn owner_grant_from_postcard_hex(s: &str) -> Result<OwnerGrantPresentation, CerudError> {
    let bytes = hex::decode(s)
        .map_err(|e| CerudError::Verb(format!("grant_postcard is not valid hex: {e}")))?;
    OwnerGrantWire::from_postcard(&bytes).map_err(|e| {
        CerudError::Verb(format!(
            "owner-grant presentation did not decode (postcard): {e}"
        ))
    })
}

// ── the CPace fallback session store ────────────────────────────────────────

/// The refusal of a `code-pair-start` while no code is armed.
pub const NOT_ARMED: &str = "no code pairing is armed on this robot; a pairing code is made on \
                             the robot itself: get one from the robot and run `cerulion pair` \
                             again";
/// The refusal of a try on a code a desk already paired with.
pub const CODE_USED: &str = "code pairing rejected: this pairing code was already used by a desk; \
                             get a new code from the robot and run `cerulion pair` again";
/// The refusal of a try that comes less than
/// [`PAIR_CODE_RATE_INTERVAL`] after the robot's last admitted try.
pub const TRY_THROTTLED: &str = "code pairing rejected: this robot takes one code pairing try \
                                 every 2 s; wait and run `cerulion pair` again";
/// The refusal of a try while no try is left on the code and not every one
/// has failed: the rest are pending on desks still pairing, or a right code's
/// row is being saved.
pub const TRIES_IN_PROGRESS: &str = "code pairing rejected: every try on this pairing code is \
                                     taken by a desk that is still pairing; wait and run \
                                     `cerulion pair` again";
/// The refusal of a `code-pair-finish` from a device with no try in progress.
pub const NO_SESSION: &str = "no code-pairing session is in progress for this device; call \
                              code-pair-start first";

/// The refusal text of a try on a code whose `tries` tries all failed; the
/// desk reads it as a wrong code.
pub fn code_exhausted(tries: u8) -> String {
    pake_text(PairingError::AttemptsExhausted { max: tries })
}

/// The refusal text of a try once the code's window has passed, which a
/// responder whose own liveness check fails answers too.
pub fn code_expired() -> String {
    pake_text(PairingError::PakeExpired)
}
/// Whether a right code whose access row could not be saved (the store
/// refused the row, or writing it failed) counts as a failed try. When it does
/// not, the try is given back.
pub const SAVE_FAILURE_IS_A_FAILED_TRY: bool = true;

const NS_PER_S: u64 = 1_000_000_000;

/// How an armed code ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Ended {
    Paired,
    Exhausted,
    Expired,
}

/// The facts about an armed code that [`settle`] and [`admit_try`] decide
/// from; times are monotonic nanoseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CodeView {
    /// The tries every desk shares.
    tries: u8,
    /// Tries admitted so far: failed, pending, being saved, or paired.
    used: u8,
    /// Tries that failed.
    failed: u8,
    /// A right code's access row is being saved.
    saving: bool,
    armed_at_ns: u64,
    window_ns: u64,
    /// The first outcome reached, which stays.
    ended: Option<Ended>,
}

impl CodeView {
    fn past_window(&self, now_ns: u64) -> bool {
        now_ns.saturating_sub(self.armed_at_ns) > self.window_ns
    }
}

/// The outcome an armed code has come to at `now_ns`: the first outcome
/// reached stays; while a right code's row is being saved there is none yet;
/// then every try failed is [`Ended::Exhausted`], and a window that has passed
/// is [`Ended::Expired`]. A pairing is reached only by its save.
fn settle(view: CodeView, now_ns: u64) -> Option<Ended> {
    if view.ended.is_some() {
        return view.ended;
    }
    if view.saving {
        None
    } else if view.failed >= view.tries {
        Some(Ended::Exhausted)
    } else if view.past_window(now_ns) {
        Some(Ended::Expired)
    } else {
        None
    }
}

/// Why a `code-pair-start` uses no try.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Refusal {
    /// A desk paired with the code.
    Used,
    /// Every try failed.
    Exhausted,
    /// The window has passed.
    Expired,
    /// No try is left, and not every one failed: the rest are pending, or a
    /// right code's row is being saved.
    InProgress,
    /// The robot's last admitted try was less than the rate interval ago.
    Throttled,
}

/// Whether a `code-pair-start` at `now_ns` gets a try, `last_try_ns` being the
/// robot's last admitted try. Checked in order: the code was used, every try
/// failed, the window has passed, no try is left, the rate interval has not
/// passed. A time before `last_try_ns`, which the store's monotonic clock read
/// under its lock never gives, is refused as too soon.
fn admit_try(
    view: CodeView,
    last_try_ns: Option<u64>,
    now_ns: u64,
    interval_ns: u64,
) -> Result<(), Refusal> {
    if let Some(ended) = view.ended {
        return Err(ended_refusal(ended));
    }
    if view.failed >= view.tries {
        return Err(Refusal::Exhausted);
    }
    if view.past_window(now_ns) {
        return Err(Refusal::Expired);
    }
    if view.used >= view.tries {
        return Err(Refusal::InProgress);
    }
    match last_try_ns {
        Some(last) if now_ns.saturating_sub(last) < interval_ns => Err(Refusal::Throttled),
        _ => Ok(()),
    }
}

/// The `(used, failed)` tries after a right code whose row could not be
/// saved: the try counts as failed, or, when `counts_as_failed` is false, it
/// is given back.
fn after_save_failure(used: u8, failed: u8, counts_as_failed: bool) -> (u8, u8) {
    if counts_as_failed {
        (used, failed.saturating_add(1))
    } else {
        (used.saturating_sub(1), failed)
    }
}

/// The refusal of a try on a code that ended as `ended`.
fn ended_refusal(ended: Ended) -> Refusal {
    match ended {
        Ended::Paired => Refusal::Used,
        Ended::Exhausted => Refusal::Exhausted,
        Ended::Expired => Refusal::Expired,
    }
}

/// The verb error a refused try answers with.
fn refusal_error(refusal: Refusal, tries: u8) -> CerudError {
    CerudError::Verb(match refusal {
        Refusal::Used => CODE_USED.into(),
        Refusal::Exhausted => code_exhausted(tries),
        Refusal::Expired => code_expired(),
        Refusal::InProgress => TRIES_IN_PROGRESS.into(),
        Refusal::Throttled => TRY_THROTTLED.into(),
    })
}

/// How an armed code ended, and the desk that paired.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Outcome {
    Paired(PairedDesk),
    Exhausted,
    Expired,
}

/// The armed code: its policy, when it was armed, its tries and its outcome.
struct CodeLedger {
    /// Which arm of this store it is, from 1.
    arm: u64,
    code: String,
    policy: CeremonyConfig,
    armed_at_ns: u64,
    used: u8,
    failed: u8,
    /// A right code's row is being saved.
    saving: bool,
    outcome: Option<Outcome>,
}

impl CodeLedger {
    fn view(&self) -> CodeView {
        CodeView {
            tries: self.policy.max_attempts,
            used: self.used,
            failed: self.failed,
            saving: self.saving,
            armed_at_ns: self.armed_at_ns,
            window_ns: self.policy.ttl_ns,
            ended: self.outcome.as_ref().map(|outcome| match outcome {
                Outcome::Paired(_) => Ended::Paired,
                Outcome::Exhausted => Ended::Exhausted,
                Outcome::Expired => Ended::Expired,
            }),
        }
    }

    fn state(&self) -> PairCodeState {
        match &self.outcome {
            None => PairCodeState::Armed,
            Some(Outcome::Paired(desk)) => PairCodeState::Paired(desk.clone()),
            Some(Outcome::Exhausted) => PairCodeState::Exhausted,
            Some(Outcome::Expired) => PairCodeState::Expired,
        }
    }
}

/// The store behind [`SharedCodePairSessions`].
#[derive(Default)]
struct CodeState {
    ledger: Option<CodeLedger>,
    /// The responder of each desk key's pending try.
    pending: BTreeMap<[u8; 32], CpaceResponder>,
    /// How many ops sessions each desk key has open.
    open: BTreeMap<[u8; 32], usize>,
    /// The robot's last admitted try; a new arm leaves it.
    last_try_ns: Option<u64>,
    arms: u64,
    /// Rises on every change of the armed code's state.
    generation: u64,
}

impl CodeState {
    /// Latch the outcome [`settle`] gives at `now_ns`, if the code has none
    /// yet; when the window has passed, every pending try is counted failed.
    /// Whether anything changed.
    fn latch(&mut self, now_ns: u64) -> bool {
        let Some(ledger) = self.ledger.as_mut() else {
            return false;
        };
        if ledger.outcome.is_some() {
            return false;
        }
        match settle(ledger.view(), now_ns) {
            None | Some(Ended::Paired) => return false,
            Some(Ended::Exhausted) => ledger.outcome = Some(Outcome::Exhausted),
            Some(Ended::Expired) => {
                let pending = u8::try_from(self.pending.len()).unwrap_or(u8::MAX);
                ledger.failed = ledger.failed.saturating_add(pending);
                self.pending.clear();
                ledger.outcome = Some(Outcome::Expired);
            }
        }
        true
    }

    fn snapshot(&self) -> Option<CodeSnapshot> {
        let ledger = self.ledger.as_ref()?;
        Some(CodeSnapshot::of(ledger, self.generation))
    }

    /// A pending try of `initiator` counted failed, as when its sessions all
    /// ended; whether there was one.
    fn fail_pending(&mut self, initiator: &[u8; 32]) -> bool {
        if self.pending.remove(initiator).is_none() {
            return false;
        }
        if let Some(ledger) = self.ledger.as_mut() {
            ledger.failed = ledger.failed.saturating_add(1);
        }
        true
    }
}

/// The armed code's state at one generation of the store.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CodeSnapshot {
    /// Which arm of this store the code is, from 1.
    pub arm: u64,
    /// The store's generation, which rises on every change of the state.
    pub generation: u64,
    /// What the code came to so far.
    pub state: PairCodeState,
    /// The tries every desk shares.
    pub tries: u8,
    /// How many of them failed.
    pub failed: u8,
    /// The code's window in whole seconds, counted from the arm.
    pub ttl_s: u64,
}

impl CodeSnapshot {
    fn of(ledger: &CodeLedger, generation: u64) -> Self {
        CodeSnapshot {
            arm: ledger.arm,
            generation,
            state: ledger.state(),
            tries: ledger.policy.max_attempts,
            failed: ledger.failed,
            ttl_s: ledger.policy.ttl_ns / NS_PER_S,
        }
    }

    /// The state as a [`PairCodeReport`].
    pub fn report(&self) -> PairCodeReport {
        PairCodeReport {
            status: self.state.status(),
            tries: self.tries,
            failed: self.failed,
        }
    }
}

/// What [`SharedCodePairSessions::arm`] did.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CodeArmed {
    /// The new code's state.
    pub armed: CodeSnapshot,
    /// The last state of the code it replaced, its outcome latched; `None`
    /// when no code was armed.
    pub replaced: Option<CodeSnapshot>,
}

struct SessionsInner {
    state: Mutex<CodeState>,
    /// Held across every `code-pair-finish`, the row save of a right code
    /// included, so two right codes are never saved for one arm, and taken by
    /// every arm and disarm, so neither replaces a code whose row is being
    /// saved. Taken only on blocking threads, and never with `state` held.
    saving: Mutex<()>,
    /// Held across every `code-pair-start`, the CPace step it runs with
    /// `state` released included, and across every arm and disarm, so a try's
    /// responder is stored before the next start is judged and no arm lands
    /// while one is built. Taken after `saving` when both are, and never with
    /// `state` held.
    starting: Mutex<()>,
    clock: MonotonicClock,
    changed: tokio::sync::Notify,
    /// A pause of the next start's CPace step ([`HeldBuild`]).
    #[cfg(any(test, feature = "test-seam"))]
    build_pause: Mutex<Option<BuildPause>>,
}

/// The two ends of a pause a [`HeldBuild`] holds: the start tells `reached`
/// when its CPace step is due, and waits on `release`.
#[cfg(any(test, feature = "test-seam"))]
struct BuildPause {
    reached: std::sync::mpsc::Sender<()>,
    release: std::sync::mpsc::Receiver<()>,
}

/// The next `code-pair-start` that is admitted, paused just before its CPace
/// step, with the store's state lock released
/// ([`SharedCodePairSessions::hold_next_build_for_test`]). Dropping it lets
/// the step run.
#[cfg(any(test, feature = "test-seam"))]
pub struct HeldBuild {
    reached: std::sync::mpsc::Receiver<()>,
    release: std::sync::mpsc::Sender<()>,
}

#[cfg(any(test, feature = "test-seam"))]
impl HeldBuild {
    /// Whether the paused start reached its CPace step within `bound`.
    pub fn reached_within(&self, bound: Duration) -> bool {
        self.reached.recv_timeout(bound).is_ok()
    }

    /// Let the paused start run its CPace step.
    pub fn release(self) {
        let _ = self.release.send(());
    }
}

/// The code-pairing store: the armed guest code and its tries, shared by
/// every ops session.
///
/// **One budget.** An armed code has one budget of tries (the policy's
/// `max_attempts`) that every desk shares. `code-pair-start` admits a try
/// while one is left, inside the window, and at least
/// [`PAIR_CODE_RATE_INTERVAL`] after the robot's last admitted try, whatever
/// code that was for; a refused start uses no try. A start reads the clock
/// once it holds the store's state lock, so the admitted tries are at least
/// the interval apart however the starts of several desks race. With no try
/// left and not every one failed it is refused with [`TRIES_IN_PROGRESS`],
/// and with every try failed with [`code_exhausted`]. Each admitted try gets a
/// fresh CPace responder created at the arm time, so the responder's own
/// liveness check enforces the same window at `respond` and at `finish`.
///
/// **When a try fails.** A wrong tag at `code-pair-finish` and a failed
/// `respond` fail the try at once. A desk that sees a wrong code itself never
/// calls `code-pair-finish`, so its try stays pending until every ops session
/// of its key has ended ([`SharedCodePairSessions::open_session`]): then no
/// finish for it can arrive, and it is counted failed. A desk key's new try
/// replaces its pending one, which is counted failed too. When the window
/// passes, every pending try is counted failed. A right code whose row cannot
/// be saved, a save that panics included, counts as a failed try when
/// [`SAVE_FAILURE_IS_A_FAILED_TRY`] is set, and is given back when it is not;
/// the desk's next try gets a fresh responder either way.
///
/// **The outcome stays.** The first outcome reached is kept: paired (the row
/// was saved), exhausted (every try failed) or expired (the window passed),
/// in that order of precedence at one instant. While a right code's row is
/// being saved no other outcome is reached. A paired code refuses every later
/// try ([`CODE_USED`]), so one code pairs one desk, and the other desks'
/// pending tries are dropped uncounted.
///
/// **Time.** The window and the rate interval are measured on the store's
/// [`MonotonicClock`], so a step of the wall clock neither stretches nor cuts
/// them.
///
/// **No disk write or CPace step under the lock.** A right code's row is
/// saved, and a try's CPace step runs, with the store's state lock released,
/// so an ops session's open and close, and a reader of the state, never wait
/// for a disk or for a CPace computation. A start takes its try under the
/// lock, runs the step, then stores the responder under the lock again: a
/// code that ended meanwhile refuses the try, counted failed when the window
/// passed and uncounted when a desk paired.
#[derive(Clone)]
pub struct SharedCodePairSessions(Arc<SessionsInner>);

impl Default for SharedCodePairSessions {
    fn default() -> Self {
        SharedCodePairSessions::with_clock(MonotonicClock::Process)
    }
}

impl SharedCodePairSessions {
    /// A fresh, un-armed store on the process's monotonic clock.
    pub fn new() -> Self {
        SharedCodePairSessions::default()
    }

    /// A fresh, un-armed store whose window and rate interval are measured
    /// on `clock`.
    pub fn with_clock(clock: MonotonicClock) -> Self {
        SharedCodePairSessions(Arc::new(SessionsInner {
            state: Mutex::new(CodeState::default()),
            saving: Mutex::new(()),
            starting: Mutex::new(()),
            clock,
            changed: tokio::sync::Notify::new(),
            #[cfg(any(test, feature = "test-seam"))]
            build_pause: Mutex::new(None),
        }))
    }

    fn lock(&self) -> MutexGuard<'_, CodeState> {
        self.0.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn starting(&self) -> MutexGuard<'_, ()> {
        self.0
            .starting
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
    }

    /// Pause the next admitted `code-pair-start` just before its CPace step,
    /// with the state lock released, until the returned [`HeldBuild`] is
    /// released or dropped.
    #[cfg(any(test, feature = "test-seam"))]
    pub fn hold_next_build_for_test(&self) -> HeldBuild {
        let (reached_tx, reached) = std::sync::mpsc::channel();
        let (release, release_rx) = std::sync::mpsc::channel();
        *self
            .0
            .build_pause
            .lock()
            .unwrap_or_else(PoisonError::into_inner) = Some(BuildPause {
            reached: reached_tx,
            release: release_rx,
        });
        HeldBuild { reached, release }
    }

    /// Wait on the pause [`SharedCodePairSessions::hold_next_build_for_test`]
    /// set, if any.
    #[cfg(any(test, feature = "test-seam"))]
    fn pause_build(&self) {
        let pause = self
            .0
            .build_pause
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .take();
        if let Some(pause) = pause {
            let _ = pause.reached.send(());
            let _ = pause.release.recv();
        }
    }

    fn now(&self) -> u64 {
        self.0.clock.now_ns()
    }

    /// Tell [`SharedCodePairSessions::changed`] waiters that the state changed.
    fn notify(&self, changed: bool) {
        if changed {
            self.0.changed.notify_one();
        }
    }

    /// Wait until the armed code's state changes.
    pub async fn changed(&self) {
        self.0.changed.notified().await;
    }

    /// Arm `code` under `policy` (`max_attempts` tries every desk shares, in a
    /// window of `ttl_ns` from now). Replaces any armed code, its tries and
    /// its pending tries, once a right code's row being saved is saved and a
    /// start's CPace step in progress has ended; the rate interval runs on
    /// from the last admitted try. Returns the new code's state and the
    /// replaced code's last state, its outcome latched as
    /// [`SharedCodePairSessions::disarm`] answers it.
    pub fn arm(&self, code: impl Into<String>, policy: CeremonyConfig) -> CodeArmed {
        let _saving = self.0.saving.lock().unwrap_or_else(PoisonError::into_inner);
        let _starting = self.starting();
        let mut st = self.lock();
        let now = self.now();
        if st.latch(now) {
            st.generation += 1;
        }
        let replaced = st.snapshot();
        st.arms += 1;
        st.generation += 1;
        let ledger = CodeLedger {
            arm: st.arms,
            code: code.into(),
            policy,
            armed_at_ns: now,
            used: 0,
            failed: 0,
            saving: false,
            outcome: None,
        };
        let snapshot = CodeSnapshot::of(&ledger, st.generation);
        st.ledger = Some(ledger);
        st.pending.clear();
        drop(st);
        self.notify(true);
        CodeArmed {
            armed: snapshot,
            replaced,
        }
    }

    /// Drop the armed code, if any, and every pending try, once a right
    /// code's row being saved is saved and a start's CPace step in progress
    /// has ended; the code's last state, its outcome latched as
    /// [`SharedCodePairSessions::snapshot`] does, or `None` when no code was
    /// armed. A later start is refused with [`NOT_ARMED`], and a later finish
    /// with [`NO_SESSION`].
    pub fn disarm(&self) -> Option<CodeSnapshot> {
        let _saving = self.0.saving.lock().unwrap_or_else(PoisonError::into_inner);
        let _starting = self.starting();
        let mut st = self.lock();
        let now = self.now();
        if st.latch(now) {
            st.generation += 1;
        }
        let last = st.snapshot();
        if st.ledger.take().is_some() {
            st.generation += 1;
        }
        st.pending.clear();
        drop(st);
        self.notify(last.is_some());
        last
    }

    /// The armed code's state now, after latching the outcome now settles (a
    /// window that passed with no desk acting is expired here); `None` when
    /// no code is armed.
    pub fn snapshot(&self) -> Option<CodeSnapshot> {
        let mut st = self.lock();
        let now = self.now();
        let changed = st.latch(now);
        if changed {
            st.generation += 1;
        }
        let snapshot = st.snapshot();
        drop(st);
        self.notify(changed);
        snapshot
    }

    /// An ops session of the desk key `initiator` opened. When the last open
    /// session of that key ends (the guard drops), its pending try, if any, is
    /// counted failed: no finish for it can arrive any more.
    pub fn open_session(&self, initiator: [u8; 32]) -> CodePairSession {
        *self.lock().open.entry(initiator).or_insert(0) += 1;
        CodePairSession {
            sessions: self.clone(),
            initiator,
        }
    }

    fn session_ended(&self, initiator: [u8; 32]) {
        let mut st = self.lock();
        let now = self.now();
        let mut changed = st.latch(now);
        let last = match st.open.get_mut(&initiator) {
            Some(open) if *open > 1 => {
                *open -= 1;
                false
            }
            _ => {
                st.open.remove(&initiator);
                true
            }
        };
        if last && st.fail_pending(&initiator) {
            st.latch(now);
            changed = true;
        }
        if changed {
            st.generation += 1;
        }
        drop(st);
        self.notify(changed);
    }

    /// Run the responder's step 2 for `initiator`'s `msg1` as one try on the
    /// armed code (see the type docs for when a start is admitted). A refused
    /// start uses no try: [`NOT_ARMED`], [`CODE_USED`], [`code_exhausted`],
    /// [`code_expired`], [`TRIES_IN_PROGRESS`] or [`TRY_THROTTLED`]. The CPace
    /// step runs with the state lock released; a code that ended while it ran
    /// refuses the try as [`code_expired`] (the try counted failed) or
    /// [`CODE_USED`] (uncounted).
    pub fn start(
        &self,
        initiator: [u8; 32],
        responder_key: PublicKey,
        msg1: &[u8],
    ) -> CerudResult<(Vec<u8>, [u8; 32])> {
        let _starting = self.starting();
        let taken = self.take_try(initiator)?;
        #[cfg(any(test, feature = "test-seam"))]
        self.pause_build();
        let mut responder = CpaceResponder::new(
            taken.code,
            PakeIdentities::new(PublicKey(initiator), responder_key, Vec::new()),
            taken.policy,
            taken.armed_at_ns,
        );
        let reply = responder.respond(msg1, taken.now_ns);
        self.store_try(initiator, taken.arm, responder, reply.is_ok())?;
        reply
            .map(|r| (r.msg2, r.responder_confirm))
            .map_err(map_pake)
    }

    /// Admit a try of `initiator` and take it from the budget, under the state
    /// lock: what its CPace step needs.
    fn take_try(&self, initiator: [u8; 32]) -> CerudResult<TakenTry> {
        let mut guard = self.lock();
        // Read with the lock held: a start that waited for the lock is judged
        // at the time it got it, after every try admitted before it.
        let now = self.now();
        let st = &mut *guard;
        let latched = st.latch(now);
        let Some(ledger) = st.ledger.as_mut() else {
            return Err(CerudError::Verb(NOT_ARMED.into()));
        };
        let interval = PAIR_CODE_RATE_INTERVAL.as_nanos() as u64;
        if let Err(refusal) = admit_try(ledger.view(), st.last_try_ns, now, interval) {
            let tries = ledger.policy.max_attempts;
            if latched {
                st.generation += 1;
            }
            drop(guard);
            self.notify(latched);
            return Err(refusal_error(refusal, tries));
        }
        ledger.used += 1;
        st.last_try_ns = Some(now);
        let taken = TakenTry {
            arm: ledger.arm,
            code: ledger.code.clone(),
            policy: ledger.policy,
            armed_at_ns: ledger.armed_at_ns,
            now_ns: now,
        };
        // A pending try of this key can no longer finish: its desk began a
        // new one, and that try's keys are gone.
        if st.fail_pending(&initiator) {
            st.generation += 1;
            drop(guard);
            self.notify(true);
        }
        Ok(taken)
    }

    /// Store the outcome of the CPace step of a try `initiator` took on arm
    /// `arm`, under the state lock: its `responder` pending when the step
    /// worked, the try counted failed when it did not. A code that ended
    /// meanwhile refuses the try instead (see [`SharedCodePairSessions::start`]).
    fn store_try(
        &self,
        initiator: [u8; 32],
        arm: u64,
        responder: CpaceResponder,
        worked: bool,
    ) -> CerudResult<()> {
        let mut guard = self.lock();
        let now = self.now();
        let st = &mut *guard;
        st.latch(now);
        let stored = match st.ledger.as_mut().filter(|ledger| ledger.arm == arm) {
            // Arms and disarms wait for a start in progress, so the arm is
            // the try's own while one is armed.
            None => Err(CerudError::Verb(NOT_ARMED.into())),
            Some(ledger) => match ledger.view().ended {
                None => {
                    if worked {
                        st.pending.insert(initiator, responder);
                    } else {
                        ledger.failed = ledger.failed.saturating_add(1);
                    }
                    Ok(())
                }
                Some(ended) => {
                    // A try whose step ran as the window passed is counted
                    // failed, as a pending one is; once a desk paired, the
                    // others' tries are dropped uncounted.
                    if ended == Ended::Expired {
                        ledger.failed = ledger.failed.saturating_add(1);
                    }
                    Err(refusal_error(
                        ended_refusal(ended),
                        ledger.policy.max_attempts,
                    ))
                }
            },
        };
        st.latch(now);
        st.generation += 1;
        drop(guard);
        self.notify(true);
        stored
    }

    /// Verify `initiator`'s confirmation tag for its pending try and, when
    /// it matches, mint the [`CpaceConfirmed`] witness bound to `account` and
    /// hand it to `save`, which writes the access row. A saved row pairs the
    /// code with the desk labelled `label`
    /// ([`label_token`](cerulion_pairing::pair_code::label_token)); a wrong tag
    /// fails the try; a failed save is counted as the type docs say. Either
    /// way the try is over, and the desk's next try gets a fresh responder.
    /// `save` runs with the store's state lock released; a `save` that panics
    /// is counted as a failed save before the panic goes on.
    pub fn finish_with(
        &self,
        initiator: [u8; 32],
        initiator_confirm: &[u8],
        account: AccountId,
        principal_kind: PrincipalKind,
        label: &str,
        save: impl FnOnce(&CpaceConfirmed) -> CerudResult<()>,
    ) -> CerudResult<()> {
        let _saving = self.0.saving.lock().unwrap_or_else(PoisonError::into_inner);
        let mut guard = self.lock();
        let now = self.now();
        let st = &mut *guard;
        let mut changed = st.latch(now);
        let confirmed = match st.ledger.as_mut() {
            None => Err(CerudError::Verb(NO_SESSION.into())),
            Some(ledger) => match ledger.view().ended {
                Some(ended) => Err(refusal_error(
                    ended_refusal(ended),
                    ledger.policy.max_attempts,
                )),
                None => match st.pending.remove(&initiator) {
                    None => Err(CerudError::Verb(NO_SESSION.into())),
                    Some(mut responder) => {
                        changed = true;
                        match responder.finish(initiator_confirm, now, account, principal_kind) {
                            Ok(confirmed) => {
                                ledger.saving = true;
                                Ok((confirmed, ledger.arm))
                            }
                            Err(e) => {
                                ledger.failed = ledger.failed.saturating_add(1);
                                Err(map_pake(e))
                            }
                        }
                    }
                },
            },
        };
        st.latch(now);
        if changed {
            st.generation += 1;
        }
        drop(guard);
        self.notify(changed);
        let (confirmed, arm) = confirmed?;

        // Ends the save when dropped, also when `save` panics.
        let mut end = SaveEnd {
            sessions: self,
            arm,
            paired: None,
        };
        let saved = save(&confirmed);
        if saved.is_ok() {
            end.paired = Some(PairedDesk::tokenized(account.0, label));
        }
        drop(end);
        saved
    }

    /// End the row save of a right code on arm `arm`: the code pairs `paired`
    /// when its row was saved, and the try is counted as a failed save when
    /// it was not. Arms and disarms wait for the save, so only a store whose
    /// ledger was written directly has another arm here, and it is left as it
    /// is.
    fn end_save(&self, arm: u64, paired: Option<PairedDesk>) {
        let mut guard = self.lock();
        let now = self.now();
        let st = &mut *guard;
        let Some(ledger) = st.ledger.as_mut().filter(|ledger| ledger.arm == arm) else {
            return;
        };
        ledger.saving = false;
        match paired {
            Some(desk) => {
                ledger.outcome = Some(Outcome::Paired(desk));
                st.pending.clear();
            }
            None => {
                (ledger.used, ledger.failed) =
                    after_save_failure(ledger.used, ledger.failed, SAVE_FAILURE_IS_A_FAILED_TRY);
            }
        }
        st.latch(now);
        st.generation += 1;
        drop(guard);
        self.notify(true);
    }
}

/// A try [`SharedCodePairSessions::start`] took from the budget: the arm it
/// is on and what its CPace step needs.
struct TakenTry {
    arm: u64,
    code: String,
    policy: CeremonyConfig,
    armed_at_ns: u64,
    now_ns: u64,
}

/// The end of a right code's row save, run when it is dropped
/// ([`SharedCodePairSessions::end_save`]): `paired` is the desk when the row
/// was saved, and `None` for a save that failed or panicked.
struct SaveEnd<'a> {
    sessions: &'a SharedCodePairSessions,
    arm: u64,
    paired: Option<PairedDesk>,
}

impl Drop for SaveEnd<'_> {
    fn drop(&mut self) {
        self.sessions.end_save(self.arm, self.paired.take());
    }
}

/// An open ops session of one desk key ([`SharedCodePairSessions::open_session`]).
pub struct CodePairSession {
    sessions: SharedCodePairSessions,
    initiator: [u8; 32],
}

impl Drop for CodePairSession {
    fn drop(&mut self) {
        self.sessions.session_ended(self.initiator);
    }
}

/// Map a CPace [`PairingError`] to a client-facing verb error (a code-pairing
/// rejection — wrong code, exhausted attempts, expired TTL — is a verb failure,
/// never fail-open).
fn map_pake(e: PairingError) -> CerudError {
    CerudError::Verb(pake_text(e))
}

/// The text of a code-pairing rejection for the CPace error `e`.
fn pake_text(e: PairingError) -> String {
    format!("code pairing rejected: {e}")
}

// ── the verb handlers ────────────────────────────────────────────────────────

/// `claim` — physical-possession claim of an UNCLAIMED robot (chassis secret).
pub struct ClaimVerb {
    shared: SharedTrust,
    clock: RemotedClock,
}

impl ClaimVerb {
    /// Build the claim verb over the live trust state + the robot's trusted clock.
    pub fn new(shared: SharedTrust, clock: RemotedClock) -> Self {
        ClaimVerb { shared, clock }
    }
}

impl VerbHandler for ClaimVerb {
    fn name(&self) -> &'static str {
        verbs::CLAIM
    }
    fn is_mutating(&self) -> bool {
        true // writes the owner row + side-map; intent-receipt-before-effect
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(requires_caller(verbs::CLAIM))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let device_key = caller_device_key(caller)?;
        let account = AccountId(hex32_field(args, "account")?);
        let chassis = hex_bytes_field(args, "chassis_secret")?;
        let principal_kind = principal_kind_field(args)?;
        self.shared
            .claim(
                &device_key,
                account,
                &chassis,
                principal_kind,
                self.clock.now_ns(),
            )
            .map_err(|e| CerudError::Verb(format!("claim failed: {e}")))?;
        Ok(serde_json::json!({
            "claimed": true,
            "account": hex::encode(account.0),
        }))
    }
}

/// `pair` — strong-path new pairing from an offline-verifiable cert chain.
pub struct PairVerb {
    shared: SharedTrust,
    clock: RemotedClock,
}

impl PairVerb {
    /// Build the pair verb over the live trust state + the robot's trusted clock.
    pub fn new(shared: SharedTrust, clock: RemotedClock) -> Self {
        PairVerb { shared, clock }
    }
}

impl VerbHandler for PairVerb {
    fn name(&self) -> &'static str {
        verbs::PAIR
    }
    fn is_mutating(&self) -> bool {
        true // establishes an access row + side-map; intent-receipt-before-effect
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(requires_caller(verbs::PAIR))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let device_key = caller_device_key(caller)?;
        let account = match (
            args.get("presentation_postcard"),
            args.get("owner_certificate_postcard"),
        ) {
            (Some(_), None) => {
                let blob = str_field(args, "presentation_postcard")?;
                let presentation = PresentationWire::from_postcard_hex(&blob)?;
                let name = args.get("name").and_then(|v| v.as_str()).map(str::to_owned);
                self.shared.verify_and_establish(
                    &presentation,
                    &device_key,
                    name,
                    self.clock.now_ns(),
                )
            }
            (None, Some(_)) => {
                let blob = str_field(args, "owner_certificate_postcard")?;
                let bytes = hex::decode(&blob).map_err(|_| {
                    CerudError::Verb("owner_certificate_postcard is not valid hex".into())
                })?;
                let presentation = OwnerCertificatePresentationWire::from_postcard(&bytes)
                    .map_err(|e| {
                        CerudError::Verb(format!("owner certificate could not be decoded: {e}"))
                    })?;
                self.shared.verify_owner_certificate_and_establish(
                    &presentation,
                    &device_key,
                    self.clock.now_ns(),
                )
            }
            _ => return Err(CerudError::Verb(
                "pair requires exactly one of presentation_postcard or owner_certificate_postcard"
                    .into(),
            )),
        }
        .map_err(|e| CerudError::Verb(format!("pairing failed: {e}")))?;
        Ok(serde_json::json!({
            "paired": true,
            "account": hex::encode(account.0),
            "source": "strong_chain",
        }))
    }
}

/// `present-grant`: establish access from a desk-carried,
/// OWNER-SIGNED access grant, verified offline against the robot's own owner.
///
/// The subject desk carries the grant its owner signed (bundled with the owner's
/// device cert + the intermediate) and presents it here with its OWN subject device
/// cert; the robot verifies the whole chain against its claimed owner and writes a
/// [`crate::trust::SharedTrust::establish_owner_grant`] row. Self-gating (the proof is
/// the owner's signature); the authorizer classifies it bootstrap so a not-yet-paired
/// but authed desk can reach it (an UNCLAIMED robot still admits only `claim`).
pub struct PresentGrantVerb {
    shared: SharedTrust,
    clock: RemotedClock,
}

impl PresentGrantVerb {
    /// Build the present-grant verb over the live trust state + the robot's trusted clock.
    pub fn new(shared: SharedTrust, clock: RemotedClock) -> Self {
        PresentGrantVerb { shared, clock }
    }
}

impl VerbHandler for PresentGrantVerb {
    fn name(&self) -> &'static str {
        verbs::PRESENT_GRANT
    }
    fn is_mutating(&self) -> bool {
        true // establishes an access row + side-map; intent-receipt-before-effect
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(requires_caller(verbs::PRESENT_GRANT))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let device_key = caller_device_key(caller)?;
        let blob = str_field(args, "grant_postcard")?;
        let presentation = owner_grant_from_postcard_hex(&blob)?;
        let name = args
            .get("name")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());
        let account = self
            .shared
            .establish_owner_grant(&presentation, &device_key, name, self.clock.now_ns())
            .map_err(|e| CerudError::Verb(format!("owner-grant presentation rejected: {e}")))?;
        Ok(serde_json::json!({
            "paired": true,
            "account": hex::encode(account.0),
            "source": "owner_signed_grant",
        }))
    }
}

/// `code-pair-start`, CPace step 2: consume the guest's `msg1` as one try on
/// the armed code, reply `msg2` + the responder confirmation tag. NOT mutating
/// (no durable trust write; the row persists at `code-pair-finish`).
pub struct CodePairStartVerb {
    shared: SharedTrust,
    sessions: SharedCodePairSessions,
}

impl CodePairStartVerb {
    /// Build the code-pair-start verb over the code-pairing store `sessions`,
    /// which measures its window on its own monotonic clock.
    pub fn new(shared: SharedTrust, sessions: SharedCodePairSessions) -> Self {
        CodePairStartVerb { shared, sessions }
    }
}

impl VerbHandler for CodePairStartVerb {
    fn name(&self) -> &'static str {
        verbs::CODE_PAIR_START
    }
    fn is_mutating(&self) -> bool {
        false // a PAKE round; the durable row is written at code-pair-finish
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(requires_caller(verbs::CODE_PAIR_START))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let initiator = caller_device_key(caller)?;
        let responder_key = self.shared.robot_transport_key();
        let msg1 = hex_bytes_field(args, "msg1")?;
        let (msg2, responder_confirm) = self.sessions.start(initiator, responder_key, &msg1)?;
        Ok(serde_json::json!({
            "msg2": hex::encode(msg2),
            "responder_confirm": hex::encode(responder_confirm),
        }))
    }
}

/// `code-pair-finish` — verify the initiator's confirmation tag and persist a
/// code-paired access row for the authenticated device's self-account, the
/// row's times read on the trusted clock. The row is saved with the
/// code-pairing store's lock released ([`SharedCodePairSessions::finish_with`]).
/// A code does not prove membership of a supplied cloud account. Mutating.
pub struct CodePairFinishVerb {
    shared: SharedTrust,
    sessions: SharedCodePairSessions,
    clock: RemotedClock,
}

impl CodePairFinishVerb {
    /// Build the code-pair-finish verb.
    pub fn new(shared: SharedTrust, sessions: SharedCodePairSessions, clock: RemotedClock) -> Self {
        CodePairFinishVerb {
            shared,
            sessions,
            clock,
        }
    }
}

impl VerbHandler for CodePairFinishVerb {
    fn name(&self) -> &'static str {
        verbs::CODE_PAIR_FINISH
    }
    fn is_mutating(&self) -> bool {
        true // persists the code-paired row; intent-receipt-before-effect
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(requires_caller(verbs::CODE_PAIR_FINISH))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let device_key = caller_device_key(caller)?;
        let account = AccountId(device_key);
        if AccountId(hex32_field(args, "account")?) != account {
            return Err(CerudError::Verb(
                "code pairing accepts only the self-account derived from the authenticated device key; use a verified certificate chain for a cloud account"
                    .into(),
            ));
        }
        let name = str_field(args, "name")?;
        let principal_kind = principal_kind_field(args)?;
        let initiator_confirm = hex_bytes_field(args, "initiator_confirm")?;
        let now = self.clock.now_ns();
        self.sessions.finish_with(
            device_key,
            &initiator_confirm,
            account,
            principal_kind,
            &name,
            |confirmed| {
                self.shared
                    .establish_code_pairing(
                        confirmed,
                        &device_key,
                        account,
                        name.clone(),
                        CODE_PAIR_SCOPE,
                        now,
                    )
                    .map_err(|e| CerudError::Verb(format!("code pairing failed: {e}")))
            },
        )?;
        Ok(serde_json::json!({
            "paired": true,
            "account": hex::encode(account.0),
            "source": "code_paired",
        }))
    }
}

/// `engage-estop` — the safety permission floor: any PAIRED account engages it,
/// regardless of scope. Deliberately NOT mutating: e-stop must NEVER be blocked by
/// a receipt-write failure (the intent-before-effect gate is fail-CLOSED, and
/// fail-closing the SAFETY floor would be unsafe), so the effect runs first and
/// the OUTCOME receipt still records who engaged it.
///
/// ## Session token = the STABLE per-pairing device key (lease over reconnects)
///
/// The engager recorded in the lease is `caller.id` — the TLS-authenticated
/// device key hex, which is STABLE per pairing (a reconnect from the same paired
/// device presents the SAME key), NEVER the ephemeral QUIC connection id. So the
/// [`ControlLease`](cerud::lease::ControlLease) session token survives a WAN
/// drop + reopen: within the deadman
/// window a reconnect re-presents the same token, and `engage_estop` is idempotent
/// (it keeps the ORIGINAL engager). E-stop is robot STATE (`EstopState::Engaged`)
/// that persists across reconnects until cleared — a transient reconnect never
/// drops the floor, and a DIFFERENT paired device key's engage cannot take the
/// floor from the first engager. (The Remote Plane design record,
/// §Q6; pinned by
/// `cerud/tests/lease_test.rs` + `cerulion_remoted/tests/estop_starvation_test.rs`.)
///
/// ## The robot's rcpd
///
/// On the `cerulion-remoted` daemon each engage also sends an `estop` tool
/// call to the robot's `cerulion-rcpd`, which closes rcpd's own latch, and
/// sends it again until rcpd answers that it did ([`crate::rcp`]). The
/// engage is counted with `RcpdEstop::send` after the lease has flipped and
/// before the verb writes its `e-stop ENGAGED` line; the delivery runs on a
/// task of its own, and the verb answers without waiting for it. An engage
/// while a delivery runs starts none: that delivery covers it, and sends
/// once more when the engage came after the attempt rcpd answers began. An
/// engager that is not a device key sends nothing, and that is logged at
/// warn.
pub struct EngageEstopVerb {
    lease: EstopLease,
    /// Who to tell that an incident was declared.
    ///
    /// NOT an `Option`. A verb built by the only constructor a production build
    /// can reach always has a real ask, so "the e-stop silently captures
    /// nothing" is not a state this type can be in — see
    /// [`EngageEstopVerb::with_capture_ask`], which is the ONLY way to substitute
    /// one and is compiled out of a plain `cargo build`.
    capture: Arc<dyn EstopCaptureAsk>,
    /// The robot's rcpd, sent an `estop` after each engage until it answers;
    /// `None` unless the daemon wired its rcp link in.
    rcpd: Option<Arc<RcpdEstop>>,
}

impl EngageEstopVerb {
    /// Build the e-stop verb over the shared control lease. It engages
    /// through [`EstopLease::engage`], which counts the engage in before it
    /// waits for the lease's lock; the act verbs built from clones of the
    /// same `lease` send nothing while it is counted.
    ///
    /// Wires the PRODUCTION capture ask. It opens no transport here and none at
    /// rest — only an actual engage reaches iceoryx2, on a detached thread (see
    /// [`crate::flashback`]).
    pub fn new(lease: EstopLease) -> Self {
        EngageEstopVerb {
            lease,
            capture: Arc::new(TransportAsk::new()),
            rcpd: None,
        }
    }

    /// Build the verb over an EXPLICIT capture ask.
    ///
    /// **Test-only, and the gating is the no-inert-shipping guarantee rather
    /// than tidiness.** If a production build could substitute the ask, a single
    /// edit could leave the verb wired to a do-nothing double and every
    /// behavioural test in this crate would stay green. Because this constructor
    /// does not exist in a plain build, the only reachable wiring is
    /// [`EngageEstopVerb::new`]'s.
    #[doc(hidden)]
    #[cfg(any(test, feature = "test-seam"))]
    pub fn with_capture_ask(
        lease: impl Into<EstopLease>,
        capture: Arc<dyn EstopCaptureAsk>,
    ) -> Self {
        EngageEstopVerb {
            lease: lease.into(),
            capture,
            rcpd: None,
        }
    }

    /// The verb that also sends an `estop` to the robot's rcpd through `rcpd`
    /// after each engage, until rcpd answers ([`crate::rcp`]). Only the
    /// daemon's serving core, which holds the rcp link, builds one.
    pub(crate) fn with_rcpd_estop(mut self, rcpd: Arc<RcpdEstop>) -> Self {
        self.rcpd = Some(rcpd);
        self
    }
}

impl VerbHandler for EngageEstopVerb {
    fn name(&self) -> &'static str {
        verbs::ENGAGE_ESTOP
    }
    fn is_mutating(&self) -> bool {
        false // safety floor — never fail-close the e-stop on a receipt write
    }
    fn execute(&self, _args: &serde_json::Value) -> CerudResult<serde_json::Value> {
        Err(requires_caller(verbs::ENGAGE_ESTOP))
    }
    fn execute_with_caller(
        &self,
        caller: &CallerIdentity,
        _args: &serde_json::Value,
    ) -> CerudResult<serde_json::Value> {
        let safe = self.lease.engage(&caller.id);
        // rcpd's own latch closes on an `estop` tool call. The delivery runs
        // on a task of its own and `send` returns at once, after the lease
        // has flipped and before the ENGAGED line below is written. In the
        // daemon every `tracing` line is queued for the log writer's own
        // thread (`crate::log_writer`), so a full stderr holds neither this
        // verb nor an attempt of the delivery. Nothing the delivery does
        // reaches this verb's answer.
        if let Some(rcpd) = &self.rcpd {
            match caller_device_key(caller) {
                Ok(engager) => {
                    rcpd.send(&engager);
                }
                Err(e) => tracing::warn!(
                    by = %caller.id,
                    error = %e,
                    "cerulion_remoted: no estop sent to rcpd: the engager is not a device key"
                ),
            }
        }
        tracing::warn!(by = %caller.id, "cerulion_remoted: e-stop ENGAGED (safety floor)");
        // A human declared an incident; ask the machine's
        // recorder for the moment.
        //
        // AFTER the lease has already flipped, which is the ordering that makes
        // the whole thing safe to do at all: the robot is stopped before this
        // line runs, so anything that happens here can delay an ACK but never a
        // stop. `ask` returns `()` and publishes on a detached thread, so it can
        // neither fail this verb nor block it.
        self.capture.ask(&caller.id);
        Ok(serde_json::json!({
            "engaged": true,
            "by": caller.id,
            "safe_frame": safe.description,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerud::lease::ControlLease;
    use std::sync::atomic::{AtomicU64, Ordering};

    #[test]
    fn caller_device_key_requires_auth_and_a_32_byte_hex_id() {
        let key = [7u8; 32];
        assert_eq!(
            caller_device_key(&CallerIdentity::verified(hex::encode(key))).unwrap(),
            key
        );
        // Unauthenticated → loud refusal even with a valid-looking id.
        assert!(caller_device_key(&CallerIdentity {
            id: hex::encode(key),
            authenticated: false,
        })
        .is_err());
        // Authenticated but not a 32-byte key → refusal.
        assert!(caller_device_key(&CallerIdentity::verified("local-dev")).is_err());
        assert!(caller_device_key(&CallerIdentity::verified(hex::encode([1u8; 33]))).is_err());
    }

    #[test]
    fn principal_kind_field_parses_or_defaults_to_human() {
        assert_eq!(
            principal_kind_field(&serde_json::json!({})).unwrap(),
            PrincipalKind::Human
        );
        assert_eq!(
            principal_kind_field(&serde_json::json!({ "principal_kind": "human" })).unwrap(),
            PrincipalKind::Human
        );
        assert_eq!(
            principal_kind_field(&serde_json::json!({ "principal_kind": "machine" })).unwrap(),
            PrincipalKind::Machine
        );
        assert!(principal_kind_field(&serde_json::json!({ "principal_kind": "robot" })).is_err());
    }

    #[test]
    fn from_postcard_hex_rejects_bad_hex_and_non_postcard_bytes_loudly() {
        // Bad hex → a loud, fail-closed verb error (never fail-open).
        let bad_hex = PresentationWire::from_postcard_hex("zz!!not-hex").unwrap_err();
        assert!(bad_hex.to_string().contains("not valid hex"), "{bad_hex}");
        // Valid hex, but not a postcard-encoded presentation → decode error.
        let not_postcard =
            PresentationWire::from_postcard_hex(&hex::encode([0xffu8; 8])).unwrap_err();
        assert!(
            not_postcard.to_string().contains("did not decode"),
            "{not_postcard}"
        );
    }

    const ROBOT_KEY: [u8; 32] = [0xB0; 32];
    const CODE: &str = "48213907";
    const WRONG: &str = "11111111";
    const T0: u64 = 1_000_000_000_000;
    const SECOND: u64 = 1_000_000_000;
    const RATE: u64 = 2 * SECOND;
    const WINDOW: u64 = 600 * SECOND;
    const A: [u8; 32] = [0xA1; 32];
    const B: [u8; 32] = [0xB2; 32];
    const C: [u8; 32] = [0xC3; 32];
    const D: [u8; 32] = [0xD4; 32];

    /// A store on a fixed monotonic clock at `T0`, with `PAIR_CODE_POLICY` armed
    /// at `T0`.
    fn armed() -> (SharedCodePairSessions, MonotonicClock) {
        let clock = MonotonicClock::fixed(T0);
        let sessions = SharedCodePairSessions::with_clock(clock.clone());
        sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        (sessions, clock)
    }

    /// The desk's step 1 and the robot's step 2 for `code`: the desk's attempt
    /// and the robot's reply.
    fn start(
        sessions: &SharedCodePairSessions,
        desk: [u8; 32],
        code: &str,
    ) -> CerudResult<(cerulion_pairing::pake::InitiatorAttempt, Vec<u8>, [u8; 32])> {
        let initiator = cerulion_pairing::pake::CpaceInitiator::new(
            code,
            PakeIdentities::new(PublicKey(desk), PublicKey(ROBOT_KEY), Vec::new()),
        );
        let (attempt, msg1) = initiator.begin().expect("cpace begin");
        let (msg2, confirm) = sessions.start(desk, PublicKey(ROBOT_KEY), &msg1)?;
        Ok((attempt, msg2, confirm))
    }

    /// The desk's step 1 for `code`: its attempt and its `msg1`.
    fn step1(desk: [u8; 32], code: &str) -> (cerulion_pairing::pake::InitiatorAttempt, Vec<u8>) {
        cerulion_pairing::pake::CpaceInitiator::new(
            code,
            PakeIdentities::new(PublicKey(desk), PublicKey(ROBOT_KEY), Vec::new()),
        )
        .begin()
        .expect("cpace begin")
    }

    /// The desk's finish for a started try with the right code, saved by
    /// `save`.
    fn finish(
        sessions: &SharedCodePairSessions,
        desk: [u8; 32],
        started: (cerulion_pairing::pake::InitiatorAttempt, Vec<u8>, [u8; 32]),
        save: impl FnOnce(&CpaceConfirmed) -> CerudResult<()>,
    ) -> CerudResult<()> {
        let (attempt, msg2, confirm) = started;
        let (_keys, tag) = attempt
            .finish(&msg2, &confirm)
            .expect("the desk agrees with the robot on the code");
        sessions.finish_with(
            desk,
            &tag,
            AccountId(desk),
            PrincipalKind::Human,
            "lab desk",
            save,
        )
    }

    /// One whole ceremony with the right code, its row saved.
    fn pair(sessions: &SharedCodePairSessions, desk: [u8; 32]) -> CerudResult<()> {
        let started = start(sessions, desk, CODE)?;
        finish(sessions, desk, started, |_| Ok(()))
    }

    /// A try whose finish carries a tag the robot refuses, as a desk that
    /// sends a finish for a wrong code does.
    fn wrong_tag(sessions: &SharedCodePairSessions, desk: [u8; 32]) -> CerudResult<()> {
        start(sessions, desk, WRONG)?;
        sessions.finish_with(
            desk,
            &[0; 32],
            AccountId(desk),
            PrincipalKind::Human,
            "d",
            |_| Ok(()),
        )
    }

    fn refused<T>(result: CerudResult<T>) -> String {
        match result {
            Ok(_) => panic!("expected a refusal, got a success"),
            Err(e) => e.to_string(),
        }
    }

    /// A verb error's text as a refused call renders it.
    fn verb(text: &str) -> String {
        format!("verb error: {text}")
    }

    /// The armed code's `(state, tries, failed)`.
    fn state(sessions: &SharedCodePairSessions) -> (PairCodeState, u8, u8) {
        let s = sessions.snapshot().expect("a code is armed");
        (s.state, s.tries, s.failed)
    }

    fn view(used: u8, failed: u8, saving: bool, ended: Option<Ended>) -> CodeView {
        CodeView {
            tries: 3,
            used,
            failed,
            saving,
            armed_at_ns: T0,
            window_ns: WINDOW,
            ended,
        }
    }

    #[test]
    fn settle_latches_the_first_outcome_and_ranks_exhausted_before_expired() {
        let past = T0 + WINDOW + 1;
        let cases: Vec<(&str, CodeView, u64, Option<Ended>)> = vec![
            ("fresh", view(0, 0, false, None), T0, None),
            ("last instant", view(0, 0, false, None), T0 + WINDOW, None),
            (
                "window passed",
                view(0, 0, false, None),
                past,
                Some(Ended::Expired),
            ),
            ("before the arm", view(0, 0, false, None), T0 - 1, None),
            ("two failed", view(2, 2, false, None), T0, None),
            (
                "all failed",
                view(3, 3, false, None),
                T0,
                Some(Ended::Exhausted),
            ),
            (
                "all failed, late",
                view(3, 3, false, None),
                past,
                Some(Ended::Exhausted),
            ),
            (
                "one pending, late",
                view(3, 2, false, None),
                past,
                Some(Ended::Expired),
            ),
            ("saving, late", view(1, 0, true, None), past, None),
            ("saving, two failed", view(3, 2, true, None), past, None),
            (
                "paired stays",
                view(1, 0, false, Some(Ended::Paired)),
                past,
                Some(Ended::Paired),
            ),
            (
                "expired stays",
                view(3, 3, false, Some(Ended::Expired)),
                past,
                Some(Ended::Expired),
            ),
            (
                "exhausted stays",
                view(3, 3, false, Some(Ended::Exhausted)),
                T0,
                Some(Ended::Exhausted),
            ),
        ];
        for (label, view, now, want) in cases {
            assert_eq!(settle(view, now), want, "{label}");
        }
    }

    #[test]
    fn admit_try_refuses_in_order_used_exhausted_expired_in_progress_throttled() {
        let past = T0 + WINDOW + 1;
        type Case = (
            &'static str,
            CodeView,
            Option<u64>,
            u64,
            Result<(), Refusal>,
        );
        let cases: Vec<Case> = vec![
            ("first try", view(0, 0, false, None), None, T0, Ok(())),
            (
                "paired",
                view(3, 2, false, Some(Ended::Paired)),
                Some(T0),
                T0,
                Err(Refusal::Used),
            ),
            (
                "exhausted",
                view(3, 3, false, Some(Ended::Exhausted)),
                None,
                T0,
                Err(Refusal::Exhausted),
            ),
            (
                "expired",
                view(1, 1, false, Some(Ended::Expired)),
                None,
                T0,
                Err(Refusal::Expired),
            ),
            (
                "every try failed, not yet latched",
                view(3, 3, false, None),
                None,
                T0,
                Err(Refusal::Exhausted),
            ),
            (
                "every try failed, late",
                view(3, 3, false, None),
                None,
                past,
                Err(Refusal::Exhausted),
            ),
            (
                "rest pending",
                view(3, 1, false, None),
                None,
                T0,
                Err(Refusal::InProgress),
            ),
            (
                "none failed, all pending",
                view(3, 0, false, None),
                Some(T0),
                T0 + RATE,
                Err(Refusal::InProgress),
            ),
            (
                "the last one being saved",
                view(3, 2, true, None),
                None,
                T0,
                Err(Refusal::InProgress),
            ),
            (
                "none left, late",
                view(3, 0, false, None),
                None,
                past,
                Err(Refusal::Expired),
            ),
            (
                "the last one being saved, late",
                view(3, 2, true, None),
                None,
                past,
                Err(Refusal::Expired),
            ),
            (
                "late",
                view(1, 1, false, None),
                None,
                past,
                Err(Refusal::Expired),
            ),
            (
                "late and too soon",
                view(1, 1, false, None),
                Some(past),
                past,
                Err(Refusal::Expired),
            ),
            (
                "last instant",
                view(1, 1, false, None),
                None,
                T0 + WINDOW,
                Ok(()),
            ),
            (
                "too soon",
                view(1, 1, false, None),
                Some(T0),
                T0 + RATE - 1,
                Err(Refusal::Throttled),
            ),
            (
                "same instant",
                view(1, 1, false, None),
                Some(T0),
                T0,
                Err(Refusal::Throttled),
            ),
            (
                "interval passed",
                view(1, 1, false, None),
                Some(T0),
                T0 + RATE,
                Ok(()),
            ),
            (
                "clock before the last try",
                view(1, 1, false, None),
                Some(T0 + 5 * SECOND),
                T0,
                Err(Refusal::Throttled),
            ),
            (
                "one tick before the last try",
                view(1, 1, false, None),
                Some(T0 + 1),
                T0,
                Err(Refusal::Throttled),
            ),
            (
                "another desk while a row is saved",
                view(1, 0, true, None),
                Some(T0),
                T0 + RATE,
                Ok(()),
            ),
        ];
        for (label, view, last, now, want) in cases {
            assert_eq!(admit_try(view, last, now, RATE), want, "{label}");
        }
    }

    #[test]
    fn a_save_failure_counts_as_a_failed_try_or_gives_the_try_back() {
        // As shipped, a save failure counts as a failed try.
        assert_eq!(
            after_save_failure(2, 1, SAVE_FAILURE_IS_A_FAILED_TRY),
            (2, 2)
        );
        assert_eq!(after_save_failure(2, 1, true), (2, 2));
        assert_eq!(after_save_failure(2, 1, false), (1, 1));
        assert_eq!(after_save_failure(1, 0, true), (1, 1));
        assert_eq!(after_save_failure(1, 0, false), (0, 0));
    }

    #[test]
    fn each_refusal_answers_its_text() {
        assert_eq!(
            refusal_error(Refusal::Used, 3).to_string(),
            format!("verb error: {CODE_USED}")
        );
        assert_eq!(
            refusal_error(Refusal::Exhausted, 3).to_string(),
            "verb error: code pairing rejected: pairing code burned: all 3 attempts exhausted"
        );
        assert_eq!(
            refusal_error(Refusal::Expired, 3).to_string(),
            "verb error: code pairing rejected: pairing ceremony expired"
        );
        assert_eq!(
            refusal_error(Refusal::InProgress, 3).to_string(),
            format!("verb error: {TRIES_IN_PROGRESS}")
        );
        // The named texts are the store's and the responder's own.
        assert_eq!(
            code_exhausted(3),
            "code pairing rejected: pairing code burned: all 3 attempts exhausted"
        );
        assert_eq!(
            code_exhausted(5),
            "code pairing rejected: pairing code burned: all 5 attempts exhausted"
        );
        assert_eq!(
            code_expired(),
            "code pairing rejected: pairing ceremony expired"
        );
        assert_eq!(
            map_pake(PairingError::PakeExpired).to_string(),
            format!("verb error: {}", code_expired())
        );
        assert_eq!(
            refusal_error(Refusal::Throttled, 3).to_string(),
            format!("verb error: {TRY_THROTTLED}")
        );
        assert_eq!(
            NOT_ARMED,
            "no code pairing is armed on this robot; a pairing code is made on the robot \
             itself: get one from the robot and run `cerulion pair` again"
        );
        assert_eq!(
            CODE_USED,
            "code pairing rejected: this pairing code was already used by a desk; get a new \
             code from the robot and run `cerulion pair` again"
        );
        assert_eq!(
            TRIES_IN_PROGRESS,
            "code pairing rejected: every try on this pairing code is taken by a desk that is \
             still pairing; wait and run `cerulion pair` again"
        );
        assert_eq!(
            NO_SESSION,
            "no code-pairing session is in progress for this device; call code-pair-start first"
        );
        assert_eq!(
            TRY_THROTTLED,
            "code pairing rejected: this robot takes one code pairing try every 2 s; wait and \
             run `cerulion pair` again"
        );
        // The throttle text names the interval the gate uses.
        assert!(TRY_THROTTLED.contains(&format!("every {} s", PAIR_CODE_RATE_INTERVAL.as_secs())));
        assert_eq!(PAIR_CODE_RATE_INTERVAL.as_nanos(), u128::from(RATE));

        let sessions = SharedCodePairSessions::with_clock(MonotonicClock::fixed(T0));
        assert!(refused(start(&sessions, A, CODE)).ends_with(NOT_ARMED));
        assert!(refused(wrong_tag(&sessions, A)).ends_with(NOT_ARMED));
        assert!(refused(sessions.finish_with(
            A,
            &[0; 32],
            AccountId(A),
            PrincipalKind::Human,
            "d",
            |_| Ok(())
        ))
        .ends_with(NO_SESSION));
        assert_eq!(sessions.snapshot(), None);
    }

    #[test]
    fn one_desk_pairs_and_the_outcome_names_its_account_and_label_token() {
        let (sessions, _clock) = armed();
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        pair(&sessions, A).expect("desk A pairs");
        assert_eq!(
            state(&sessions),
            (
                PairCodeState::Paired(PairedDesk::new(A, "lab_desk").unwrap()),
                3,
                0
            )
        );
        assert_eq!(
            sessions.snapshot().unwrap().report(),
            PairCodeReport {
                status: cerulion_pairing::pair_code::PairCodeStatus::Paired,
                tries: 3,
                failed: 0,
            }
        );
    }

    #[test]
    fn three_desks_with_a_wrong_code_use_the_shared_budget_up() {
        let (sessions, clock) = armed();
        for (i, desk) in [A, B, C].into_iter().enumerate() {
            clock.set(T0 + RATE * i as u64);
            assert!(refused(wrong_tag(&sessions, desk)).contains("confirmation failed"));
        }
        assert_eq!(state(&sessions), (PairCodeState::Exhausted, 3, 3));
        // A fourth desk with the RIGHT code is refused: the code is used up.
        clock.set(T0 + RATE * 3);
        assert_eq!(refused(pair(&sessions, D)), verb(&code_exhausted(3)));
        assert_eq!(state(&sessions), (PairCodeState::Exhausted, 3, 3));
    }

    #[test]
    fn pending_tries_hold_the_budget_until_every_session_of_their_desk_ends() {
        let (sessions, clock) = armed();
        let mut open = Vec::new();
        for (i, desk) in [A, B, C].into_iter().enumerate() {
            clock.set(T0 + RATE * i as u64);
            open.push(sessions.open_session(desk));
            start(&sessions, desk, WRONG).expect("an admitted try");
        }
        // Every try is pending: no try is left, but none failed yet, so a
        // desk with the right code is told to wait, not that its code burned.
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        clock.set(T0 + RATE * 3);
        assert_eq!(refused(start(&sessions, D, CODE)), verb(TRIES_IN_PROGRESS));
        // A second session of desk A that ends leaves A's try pending.
        drop(sessions.open_session(A));
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        let mut failed = 0;
        for session in open {
            drop(session);
            failed += 1;
            let want = if failed == 3 {
                PairCodeState::Exhausted
            } else {
                PairCodeState::Armed
            };
            assert_eq!(state(&sessions), (want, 3, failed));
        }
    }

    #[test]
    fn the_right_code_on_the_third_try_pairs() {
        let (sessions, clock) = armed();
        for (i, desk) in [A, B].into_iter().enumerate() {
            clock.set(T0 + RATE * i as u64);
            refused(wrong_tag(&sessions, desk));
        }
        clock.set(T0 + RATE * 2);
        pair(&sessions, C).expect("the third try pairs");
        assert_eq!(
            state(&sessions),
            (
                PairCodeState::Paired(PairedDesk::new(C, "lab_desk").unwrap()),
                3,
                2
            )
        );
    }

    #[test]
    fn the_rate_gate_admits_one_try_every_interval_robot_wide_and_survives_a_re_arm() {
        let (sessions, clock) = armed();
        let _a = start(&sessions, A, CODE).expect("the first try");
        clock.set(T0 + RATE - 1);
        assert_eq!(
            refused(start(&sessions, B, CODE)),
            format!("verb error: {TRY_THROTTLED}")
        );
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        let st = sessions.lock();
        assert_eq!(
            (st.ledger.as_ref().unwrap().used, st.pending.len()),
            (1, 1),
            "a throttled start uses no try"
        );
        drop(st);
        // A new arm keeps the gate.
        sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        assert_eq!(
            refused(start(&sessions, B, CODE)),
            format!("verb error: {TRY_THROTTLED}")
        );
        clock.set(T0 + RATE);
        start(&sessions, B, CODE).expect("the interval passed");
        // A clock read before the last try is refused as too soon.
        clock.set(T0);
        assert_eq!(refused(start(&sessions, C, CODE)), verb(TRY_THROTTLED));
    }

    #[test]
    fn a_start_that_waits_for_the_lock_is_judged_at_the_time_it_gets_it() {
        let (sessions, clock) = armed();
        clock.set(T0 + WINDOW);
        let initiator = cerulion_pairing::pake::CpaceInitiator::new(
            CODE,
            PakeIdentities::new(PublicKey(A), PublicKey(ROBOT_KEY), Vec::new()),
        );
        let (_attempt, msg1) = initiator.begin().expect("cpace begin");
        // Desk A's start waits for the lock at the window's last instant and
        // gets it once the window has passed.
        let held = sessions.lock();
        let waiting = std::thread::spawn({
            let sessions = sessions.clone();
            move || sessions.start(A, PublicKey(ROBOT_KEY), &msg1)
        });
        std::thread::sleep(Duration::from_millis(200));
        clock.set(T0 + WINDOW + 1);
        drop(held);
        assert_eq!(
            refused(waiting.join().unwrap()),
            verb(&code_expired()),
            "the start is judged when it holds the lock"
        );
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 0));
    }

    #[test]
    fn a_save_that_panics_counts_as_a_failed_save_and_the_code_still_ends() {
        let (sessions, clock) = armed();
        let started = start(&sessions, A, CODE).unwrap();
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            finish(&sessions, A, started, |_| panic!("the row save panicked"))
        }));
        assert!(panicked.is_err(), "the panic goes on");
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 1));
        clock.set(T0 + WINDOW + 1);
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 1));
        // The next code's right code is saved: the save lock is free again.
        sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        pair(&sessions, B).expect("the next code pairs");
        assert_eq!(state(&sessions).0.status().as_str(), "paired");
    }

    #[test]
    fn the_window_counts_from_the_arm_and_a_quiet_expired_code_reads_expired() {
        let (sessions, clock) = armed();
        clock.set(T0 + WINDOW);
        let session = sessions.open_session(A);
        let started = start(&sessions, A, CODE).expect("the last instant is admitted");
        clock.set(T0 + WINDOW + 1);
        // The window passed with A's try pending: it is counted failed, and
        // the outcome stays expired when A's session ends.
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 1));
        assert_eq!(
            refused(finish(&sessions, A, started, |_| Ok(()))),
            verb(&code_expired())
        );
        drop(session);
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 1));
        assert_eq!(refused(start(&sessions, B, CODE)), verb(&code_expired()));

        // With no desk acting, the next look reads expired.
        sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        clock.set(T0 + 2 * WINDOW + 2);
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 0));
    }

    #[test]
    fn a_save_error_counts_a_failed_try_once_and_the_same_desk_retries_and_pairs() {
        let (sessions, clock) = armed();
        let started = start(&sessions, A, CODE).unwrap();
        let err = refused(finish(&sessions, A, started, |_| {
            Err(CerudError::Verb("code pairing failed: disk full".into()))
        }));
        assert_eq!(err, "verb error: code pairing failed: disk full");
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 1));
        clock.set(T0 + RATE);
        pair(&sessions, A).expect("the same desk's next try gets a fresh responder");
        let st = sessions.lock();
        let ledger = st.ledger.as_ref().unwrap();
        assert_eq!((ledger.used, ledger.failed), (2, 1));
    }

    #[test]
    fn a_row_is_saved_with_the_lock_released_and_no_outcome_lands_while_it_is_saved() {
        let (sessions, clock) = armed();
        let started = start(&sessions, A, CODE).unwrap();
        let during = sessions.clone();
        let late = clock.clone();
        finish(&sessions, A, started, move |_| {
            // The state lock is free: a reader does not wait for the save.
            assert!(
                during.0.state.try_lock().is_ok(),
                "the state lock is held while a row is saved"
            );
            late.set(T0 + WINDOW + 1);
            assert_eq!(state(&during), (PairCodeState::Armed, 3, 0));
            Ok(())
        })
        .expect("a right code saved in the window pairs although the window then ends");
        assert_eq!(
            state(&sessions),
            (
                PairCodeState::Paired(PairedDesk::new(A, "lab_desk").unwrap()),
                3,
                0
            )
        );

        // A save that fails after the window ends the code as expired.
        sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        clock.set(T0 + WINDOW + 1 + RATE);
        let started = start(&sessions, B, CODE).unwrap();
        let late = clock.clone();
        refused(finish(&sessions, B, started, move |_| {
            late.set(T0 + 2 * WINDOW + 2 * RATE);
            Err(CerudError::Verb(
                "code pairing failed: robot is unclaimed".into(),
            ))
        }));
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 1));
    }

    #[test]
    fn a_second_right_code_waits_for_the_first_rows_save_and_is_refused_as_used() {
        let (sessions, clock) = armed();
        let a = start(&sessions, A, CODE).unwrap();
        clock.set(T0 + RATE);
        let b = start(&sessions, B, CODE).unwrap();
        let b_saves = Arc::new(AtomicU64::new(0));
        let (b_done, b_returned) = std::sync::mpsc::channel();
        let mut b_finish = None;
        finish(&sessions, A, a, |_| {
            let sessions = sessions.clone();
            let saves = Arc::clone(&b_saves);
            b_finish = Some(std::thread::spawn(move || {
                let result = finish(&sessions, B, b, |_| {
                    saves.fetch_add(1, Ordering::SeqCst);
                    Ok(())
                });
                let _ = b_done.send(());
                result
            }));
            // Desk B's finish waits while desk A's row is saved.
            assert_eq!(
                b_returned.recv_timeout(Duration::from_millis(500)),
                Err(std::sync::mpsc::RecvTimeoutError::Timeout),
                "a second right code finished while a row was saved"
            );
            Ok(())
        })
        .expect("desk A pairs");
        let b_result = b_finish.take().unwrap().join().unwrap();
        assert_eq!(refused(b_result), format!("verb error: {CODE_USED}"));
        assert_eq!(b_saves.load(Ordering::SeqCst), 0, "no row is saved for B");
        assert_eq!(
            state(&sessions),
            (
                PairCodeState::Paired(PairedDesk::new(A, "lab_desk").unwrap()),
                3,
                0
            )
        );
    }

    #[test]
    fn a_re_arm_leaves_the_replaced_codes_tries_and_saves_no_part_in_the_new_code() {
        const LATER: &str = "50284617";
        let new_pairs = |sessions: &SharedCodePairSessions, desk| {
            let started = start(sessions, desk, LATER).expect("a try on the new code");
            finish(sessions, desk, started, |_| Ok(())).expect("the new code pairs");
        };

        // Desk A's try on the replaced code is pending at the re-arm.
        let (sessions, clock) = armed();
        let a_session = sessions.open_session(A);
        let a = start(&sessions, A, CODE).unwrap();
        sessions.arm(LATER, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        assert_eq!(
            refused(finish(&sessions, A, a, |_| Ok(()))),
            format!("verb error: {NO_SESSION}")
        );
        drop(a_session);
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        clock.set(T0 + RATE);
        new_pairs(&sessions, B);
        assert_eq!(state(&sessions).0.status().as_str(), "paired");

        // A re-arm that comes while desk A's row for the replaced code is
        // saved waits for the save, and answers the pairing it ended in.
        let (sessions, clock) = armed();
        let a = start(&sessions, A, CODE).unwrap();
        let (done, rearmed) = std::sync::mpsc::channel();
        finish(&sessions, A, a, |_| {
            std::thread::spawn({
                let sessions = sessions.clone();
                move || {
                    let _ = done
                        .send(sessions.arm(LATER, cerulion_pairing::pair_code::PAIR_CODE_POLICY));
                }
            });
            assert_eq!(
                rearmed.recv_timeout(Duration::from_millis(300)).err(),
                Some(std::sync::mpsc::RecvTimeoutError::Timeout),
                "a re-arm landed while a row was saved"
            );
            Ok(())
        })
        .expect("desk A's row is saved");
        let later = rearmed.recv_timeout(Duration::from_secs(10)).unwrap();
        let replaced = later.replaced.expect("a code was armed");
        assert_eq!(
            (replaced.state, replaced.failed),
            (
                PairCodeState::Paired(PairedDesk::new(A, "lab_desk").unwrap()),
                0
            )
        );
        // The new code has no part in the save.
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        clock.set(T0 + RATE);
        new_pairs(&sessions, B);
    }

    #[test]
    fn a_re_arm_answers_the_replaced_codes_last_state_with_its_outcome_latched() {
        // Nothing armed: nothing replaced.
        let clock = MonotonicClock::fixed(T0);
        let sessions = SharedCodePairSessions::with_clock(clock.clone());
        let first = sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        assert_eq!((first.armed.arm, first.replaced), (1, None));
        // A code with a failed try and no outcome is answered as armed.
        refused(wrong_tag(&sessions, A));
        let second = sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        let replaced = second.replaced.expect("a code was armed");
        assert_eq!(
            (replaced.arm, replaced.state, replaced.failed),
            (1, PairCodeState::Armed, 1)
        );
        // A window that passed with no desk acting is latched as expired.
        clock.set(T0 + WINDOW + 1);
        let third = sessions.arm(CODE, cerulion_pairing::pair_code::PAIR_CODE_POLICY);
        let replaced = third.replaced.expect("a code was armed");
        assert_eq!(
            (replaced.arm, replaced.state, replaced.failed),
            (2, PairCodeState::Expired, 0)
        );
        assert_eq!(third.armed.arm, 3);
        assert!(replaced.generation < third.armed.generation);
    }

    #[test]
    fn a_finish_with_no_try_pending_answers_no_session_or_the_codes_end() {
        let (sessions, clock) = armed();
        let no_try = |sessions: &SharedCodePairSessions, desk| {
            refused(sessions.finish_with(
                desk,
                &[0; 32],
                AccountId(desk),
                PrincipalKind::Human,
                "d",
                |_| Ok(()),
            ))
        };
        assert_eq!(no_try(&sessions, A), format!("verb error: {NO_SESSION}"));
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 0));
        for (i, desk) in [A, B, C].into_iter().enumerate() {
            clock.set(T0 + RATE * i as u64);
            refused(wrong_tag(&sessions, desk));
        }
        assert_eq!(no_try(&sessions, D), verb(&code_exhausted(3)));
        assert_eq!(state(&sessions), (PairCodeState::Exhausted, 3, 3));
    }

    #[test]
    fn a_desk_mid_ceremony_when_another_pairs_is_refused_and_counts_nothing() {
        let (sessions, clock) = armed();
        let b_session = sessions.open_session(B);
        let b = start(&sessions, B, CODE).unwrap();
        clock.set(T0 + RATE);
        pair(&sessions, A).expect("desk A pairs");
        assert_eq!(
            refused(finish(&sessions, B, b, |_| Ok(()))),
            format!("verb error: {CODE_USED}")
        );
        drop(b_session);
        clock.set(T0 + 2 * RATE);
        assert_eq!(
            refused(start(&sessions, C, CODE)),
            format!("verb error: {CODE_USED}")
        );
        let (got, _, failed) = state(&sessions);
        assert_eq!((got.status().as_str(), failed), ("paired", 0));
    }

    #[test]
    fn a_new_try_from_a_desk_counts_its_pending_try_failed() {
        let (sessions, clock) = armed();
        let first = start(&sessions, A, CODE).unwrap();
        clock.set(T0 + RATE);
        let second = start(&sessions, A, CODE).unwrap();
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 1));
        // The first try's keys are gone; the second one pairs.
        drop(first);
        finish(&sessions, A, second, |_| Ok(())).expect("the newer try pairs");
        assert_eq!(state(&sessions).2, 1);
    }

    #[test]
    fn a_step1_packet_the_responder_refuses_fails_the_try() {
        let (sessions, _clock) = armed();
        let err = refused(sessions.start(A, PublicKey(ROBOT_KEY), b"short"));
        assert_eq!(
            err,
            "verb error: code pairing rejected: CPace protocol error: bad step1 packet length"
        );
        assert_eq!(state(&sessions), (PairCodeState::Armed, 3, 1));
    }

    #[test]
    fn a_start_runs_its_cpace_step_with_the_state_lock_released() {
        let (sessions, _clock) = armed();
        let held = sessions.hold_next_build_for_test();
        let (desk_step, msg1) = step1(A, CODE);
        let starting = std::thread::spawn({
            let sessions = sessions.clone();
            move || sessions.start(A, PublicKey(ROBOT_KEY), &msg1)
        });
        assert!(held.reached_within(Duration::from_secs(10)));
        // While the step is paused, an ops session of another key opens and
        // closes, and the state is read, without waiting for it.
        let (done, opened) = std::sync::mpsc::channel();
        std::thread::spawn({
            let sessions = sessions.clone();
            move || {
                drop(sessions.open_session(B));
                let _ = done.send(sessions.snapshot());
            }
        });
        let seen = opened
            .recv_timeout(Duration::from_secs(10))
            .expect("a session opens and closes while a CPace step runs");
        assert_eq!(
            seen.map(|s| (s.state, s.failed)),
            Some((PairCodeState::Armed, 0))
        );
        assert!(
            sessions.0.state.try_lock().is_ok(),
            "the state lock is free"
        );
        held.release();
        let (msg2, confirm) = starting.join().unwrap().expect("the try is admitted");
        // The try is pending and pairs.
        let (_keys, tag) = desk_step.finish(&msg2, &confirm).expect("same code");
        sessions
            .finish_with(A, &tag, AccountId(A), PrincipalKind::Human, "d", |_| Ok(()))
            .expect("the pending try pairs");
        assert_eq!(state(&sessions).0.status().as_str(), "paired");
    }

    #[test]
    fn a_code_that_ends_while_a_start_runs_its_cpace_step_refuses_the_try() {
        // The window passes: the try is counted failed, as a pending one is.
        let (sessions, clock) = armed();
        let held = sessions.hold_next_build_for_test();
        let (_desk, msg1) = step1(A, CODE);
        let starting = std::thread::spawn({
            let sessions = sessions.clone();
            move || sessions.start(A, PublicKey(ROBOT_KEY), &msg1)
        });
        assert!(held.reached_within(Duration::from_secs(10)));
        clock.set(T0 + WINDOW + 1);
        held.release();
        assert_eq!(refused(starting.join().unwrap()), verb(&code_expired()));
        assert_eq!(state(&sessions), (PairCodeState::Expired, 3, 1));
        assert!(sessions.lock().pending.is_empty());

        // A desk pairs meanwhile: the try is dropped uncounted.
        let (sessions, clock) = armed();
        let b = start(&sessions, B, CODE).unwrap();
        clock.set(T0 + RATE);
        let held = sessions.hold_next_build_for_test();
        let (_desk, msg1) = step1(A, CODE);
        let starting = std::thread::spawn({
            let sessions = sessions.clone();
            move || sessions.start(A, PublicKey(ROBOT_KEY), &msg1)
        });
        assert!(held.reached_within(Duration::from_secs(10)));
        finish(&sessions, B, b, |_| Ok(())).expect("desk B pairs");
        held.release();
        assert_eq!(
            refused(starting.join().unwrap()),
            format!("verb error: {CODE_USED}")
        );
        let (got, _, failed) = state(&sessions);
        assert_eq!((got.status().as_str(), failed), ("paired", 0));
        assert!(sessions.lock().pending.is_empty());
    }

    #[test]
    fn an_arm_and_a_disarm_wait_for_a_start_in_progress() {
        let (sessions, _clock) = armed();
        let held = sessions.hold_next_build_for_test();
        let (_desk, msg1) = step1(A, CODE);
        let starting = std::thread::spawn({
            let sessions = sessions.clone();
            move || sessions.start(A, PublicKey(ROBOT_KEY), &msg1)
        });
        assert!(held.reached_within(Duration::from_secs(10)));
        let (done, disarmed) = std::sync::mpsc::channel();
        std::thread::spawn({
            let sessions = sessions.clone();
            move || {
                let _ = done.send(sessions.disarm());
            }
        });
        assert_eq!(
            disarmed.recv_timeout(Duration::from_millis(300)).err(),
            Some(std::sync::mpsc::RecvTimeoutError::Timeout),
            "a disarm landed while a CPace step ran"
        );
        held.release();
        starting.join().unwrap().expect("the try is admitted");
        let last = disarmed
            .recv_timeout(Duration::from_secs(10))
            .unwrap()
            .expect("the code was armed");
        assert_eq!((last.state, last.failed), (PairCodeState::Armed, 0));
        assert!(
            sessions.lock().pending.is_empty(),
            "the disarm drops the try"
        );
    }

    #[test]
    fn a_store_emptied_under_a_start_in_progress_answers_not_armed() {
        let (sessions, _clock) = armed();
        let held = sessions.hold_next_build_for_test();
        let (_desk, msg1) = step1(A, CODE);
        let starting = std::thread::spawn({
            let sessions = sessions.clone();
            move || sessions.start(A, PublicKey(ROBOT_KEY), &msg1)
        });
        assert!(held.reached_within(Duration::from_secs(10)));
        // Arms and disarms wait for the start; only a direct write empties it.
        sessions.lock().ledger = None;
        held.release();
        assert!(refused(starting.join().unwrap()).ends_with(NOT_ARMED));
        assert!(sessions.lock().pending.is_empty());
    }

    #[test]
    fn a_disarm_waits_for_a_rows_save_and_answers_the_pairing() {
        let (sessions, _clock) = armed();
        let a = start(&sessions, A, CODE).unwrap();
        let (done, disarmed) = std::sync::mpsc::channel();
        finish(&sessions, A, a, |_| {
            std::thread::spawn({
                let sessions = sessions.clone();
                move || {
                    let _ = done.send(sessions.disarm());
                }
            });
            assert_eq!(
                disarmed.recv_timeout(Duration::from_millis(300)).err(),
                Some(std::sync::mpsc::RecvTimeoutError::Timeout),
                "a disarm landed while a row was saved"
            );
            Ok(())
        })
        .expect("desk A pairs");
        let last = disarmed
            .recv_timeout(Duration::from_secs(10))
            .unwrap()
            .expect("the code was armed");
        assert_eq!(
            last.state,
            PairCodeState::Paired(PairedDesk::new(A, "lab_desk").unwrap())
        );
        assert_eq!(sessions.snapshot(), None);
    }

    #[test]
    fn a_disarm_latches_a_passed_window_as_expired() {
        let (sessions, clock) = armed();
        clock.set(T0 + WINDOW + 1);
        let last = sessions.disarm().expect("a code was armed");
        assert_eq!((last.state, last.failed), (PairCodeState::Expired, 0));
    }

    #[test]
    fn two_runs_of_one_script_give_identical_snapshot_sequences() {
        fn run() -> Vec<String> {
            let (sessions, clock) = armed();
            let mut seen = vec![format!("{:?}", sessions.snapshot())];
            let a = sessions.open_session(A);
            // Desk A sees the wrong code itself and sends no finish.
            start(&sessions, A, WRONG).unwrap();
            seen.push(format!("{:?}", sessions.snapshot()));
            clock.set(T0 + RATE);
            refused(wrong_tag(&sessions, B));
            seen.push(format!("{:?}", sessions.snapshot()));
            drop(a);
            seen.push(format!("{:?}", sessions.snapshot()));
            clock.set(T0 + 2 * RATE);
            pair(&sessions, C).unwrap();
            seen.push(format!("{:?}", sessions.snapshot()));
            seen
        }
        let first = run();
        assert_eq!(first, run());
        assert!(first.last().unwrap().contains("Paired"), "{first:?}");
        assert!(first[3].contains("failed: 2"), "{first:?}");
    }

    #[tokio::test]
    async fn a_waiter_learns_of_a_change() {
        let (sessions, _clock) = armed();
        tokio::time::timeout(Duration::from_secs(5), sessions.changed())
            .await
            .expect("the arm is a change");
        let waiter = tokio::spawn({
            let sessions = sessions.clone();
            async move { sessions.changed().await }
        });
        tokio::task::yield_now().await;
        refused(sessions.start(A, PublicKey(ROBOT_KEY), b"short"));
        tokio::time::timeout(Duration::from_secs(5), waiter)
            .await
            .expect("a failed try is a change")
            .unwrap();
    }

    #[test]
    fn requires_caller_is_a_loud_fail_closed_error_for_each_bootstrap_verb() {
        // The caller-less `execute` of a bootstrap verb is a loud, fail-closed trap
        // (production always dispatches via `execute_with_caller`). Test it on the
        // e-stop verb (it needs only a lease, no trust store).
        let lease = Arc::new(Mutex::new(ControlLease::with_default_window()));
        let v = EngageEstopVerb::new(EstopLease::new(lease));
        let err = v.execute(&serde_json::json!({})).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("caller-bound bootstrap verb"), "{msg}");
        assert!(msg.contains("execute_with_caller"), "{msg}");
    }
}
