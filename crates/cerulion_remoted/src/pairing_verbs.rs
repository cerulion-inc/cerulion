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
//! Certificate validity, anti-rollback, and the CPace TTL are evaluated against
//! [`RemotedClock`], the robot's trusted clock — never a client-supplied time.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use cerud::error::{CerudError, CerudResult};
use cerud::lease::ControlLease;
use cerud::transport::CallerIdentity;
use cerud::verbs::VerbHandler;

use cerulion_pairing::format::{
    AccountId, Delegation, PrincipalKind, PublicKey, SignedDeviceCert, SignedGrant,
    SignedIntermediateCert,
};
use cerulion_pairing::pake::{CeremonyConfig, CpaceConfirmed, CpaceResponder, PakeIdentities};
use cerulion_pairing::verify::{OwnerGrantPresentation, PairingPresentation};
use cerulion_pairing::PairingError;

use crate::clock::RemotedClock;
use crate::flashback::{EstopCaptureAsk, TransportAsk};
use crate::trust::{SharedTrust, CODE_PAIR_SCOPE};

// ── verb-name constants (mirror `crate::verbs`) ─────────────────────────────

use crate::verbs;

// ── wire helpers ────────────────────────────────────────────────────────────

/// Extract the transport-authenticated 32-byte device key from a caller. The
/// authorizer already gated bootstrap verbs on this, so a failure here is
/// defense-in-depth (a loud, fail-closed refusal), never a production path.
fn caller_device_key(caller: &CallerIdentity) -> CerudResult<[u8; 32]> {
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

/// A shared, mutex-guarded store of in-flight CPace ceremonies, keyed by the
/// initiator's (guest's) device key, plus the currently-armed guest code.
///
/// **Bounded attempts / code-burn / TTL are enforced PER initiator device key** by
/// the reused [`CpaceResponder`]: `code-pair-start` reuses the initiator's live
/// responder (so retries accumulate attempts on ONE responder → burn), creating a
/// fresh one from the armed code only when none exists. A successful finish is
/// terminal (single session) and removes the entry.
///
/// **Limitation:** the code-burn is per-initiator, so a global
/// rate limit across DISTINCT guest keys is not enforced; the short armed-code
/// TTL bounds the exposure window.
#[derive(Default)]
pub struct CodePairSessions {
    armed: Option<ArmedCode>,
    sessions: HashMap<[u8; 32], CpaceResponder>,
}

/// The owner-armed guest code + its ceremony policy.
struct ArmedCode {
    code: String,
    config: CeremonyConfig,
}

/// A cloneable handle to the shared CPace session store.
#[derive(Clone, Default)]
pub struct SharedCodePairSessions(Arc<Mutex<CodePairSessions>>);

impl SharedCodePairSessions {
    /// A fresh, un-armed session store.
    pub fn new() -> Self {
        SharedCodePairSessions::default()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, CodePairSessions> {
        self.0.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// Arm a guest code + its ceremony policy (the OWNER-side action; a claimed
    /// owner "begins code pairing", which the guest then completes). Replaces any
    /// previously-armed code and clears in-flight sessions (a fresh code).
    pub fn arm(&self, code: impl Into<String>, config: CeremonyConfig) {
        let mut guard = self.lock();
        guard.armed = Some(ArmedCode {
            code: code.into(),
            config,
        });
        guard.sessions.clear();
    }

    /// Run the responder's step-2 for `initiator`'s `msg1`, consuming one attempt.
    /// Reuses the initiator's live responder (retries accumulate → burn) or creates
    /// one from the armed code. Errors loudly if no code is armed.
    pub fn start(
        &self,
        initiator: [u8; 32],
        responder_key: PublicKey,
        msg1: &[u8],
        now_ns: u64,
    ) -> CerudResult<(Vec<u8>, [u8; 32])> {
        let mut guard = self.lock();
        let (code, config) = {
            let armed = guard.armed.as_ref().ok_or_else(|| {
                CerudError::Verb(
                    "no code pairing is armed on this robot; an owner must arm a guest code \
                     first (begin code pairing)"
                        .to_string(),
                )
            })?;
            (armed.code.clone(), armed.config)
        };
        let responder = guard.sessions.entry(initiator).or_insert_with(|| {
            let identities = PakeIdentities::new(PublicKey(initiator), responder_key, Vec::new());
            CpaceResponder::new(code, identities, config, now_ns)
        });
        let resp = responder.respond(msg1, now_ns).map_err(map_pake)?;
        Ok((resp.msg2, resp.responder_confirm))
    }

    /// Verify the initiator's confirmation tag and mint the [`CpaceConfirmed`]
    /// witness bound to `account`. A success removes the (single-session)
    /// responder; a wrong-code failure leaves it so retries still burn.
    pub fn finish(
        &self,
        initiator: [u8; 32],
        initiator_confirm: &[u8],
        account: AccountId,
        principal_kind: PrincipalKind,
        now_ns: u64,
    ) -> CerudResult<CpaceConfirmed> {
        let mut guard = self.lock();
        let responder = guard.sessions.get_mut(&initiator).ok_or_else(|| {
            CerudError::Verb(
                "no code-pairing session is in progress for this device; call \
                 code-pair-start first"
                    .to_string(),
            )
        })?;
        let confirmed = responder
            .finish(initiator_confirm, now_ns, account, principal_kind)
            .map_err(map_pake)?;
        guard.sessions.remove(&initiator);
        Ok(confirmed)
    }
}

/// Map a CPace [`PairingError`] to a client-facing verb error (a code-pairing
/// rejection — wrong code, exhausted attempts, expired TTL — is a verb failure,
/// never fail-open).
fn map_pake(e: PairingError) -> CerudError {
    CerudError::Verb(format!("code pairing rejected: {e}"))
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
        let blob = str_field(args, "presentation_postcard")?;
        let presentation = PresentationWire::from_postcard_hex(&blob)?;
        let name = args
            .get("name")
            .and_then(|v| v.as_str())
            .map(|s| s.to_string());
        let account = self
            .shared
            .verify_and_establish(&presentation, &device_key, name, self.clock.now_ns())
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

/// `code-pair-start` — CPace step-2: consume the guest's `msg1`, reply `msg2` +
/// the responder confirmation tag. NOT mutating (no durable trust write; the row
/// persists at `code-pair-finish`).
pub struct CodePairStartVerb {
    shared: SharedTrust,
    sessions: SharedCodePairSessions,
    clock: RemotedClock,
}

impl CodePairStartVerb {
    /// Build the code-pair-start verb.
    pub fn new(shared: SharedTrust, sessions: SharedCodePairSessions, clock: RemotedClock) -> Self {
        CodePairStartVerb {
            shared,
            sessions,
            clock,
        }
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
        let (msg2, responder_confirm) =
            self.sessions
                .start(initiator, responder_key, &msg1, self.clock.now_ns())?;
        Ok(serde_json::json!({
            "msg2": hex::encode(msg2),
            "responder_confirm": hex::encode(responder_confirm),
        }))
    }
}

/// `code-pair-finish` — verify the initiator's confirmation tag and persist a
/// code-paired access row (conservative default scope). Mutating.
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
        let account = AccountId(hex32_field(args, "account")?);
        let name = str_field(args, "name")?;
        let principal_kind = principal_kind_field(args)?;
        let initiator_confirm = hex_bytes_field(args, "initiator_confirm")?;
        let now = self.clock.now_ns();
        let confirmed =
            self.sessions
                .finish(device_key, &initiator_confirm, account, principal_kind, now)?;
        self.shared
            .establish_code_pairing(&confirmed, &device_key, account, name, CODE_PAIR_SCOPE, now)
            .map_err(|e| CerudError::Verb(format!("code pairing failed: {e}")))?;
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
/// [`ControlLease`] session token survives a WAN drop + reopen: within the deadman
/// window a reconnect re-presents the same token, and `engage_estop` is idempotent
/// (it keeps the ORIGINAL engager). E-stop is robot STATE (`EstopState::Engaged`)
/// that persists across reconnects until cleared — a transient reconnect never
/// drops the floor, and a DIFFERENT paired device key's engage cannot take the
/// floor from the first engager. (The Remote Plane design record,
/// §Q6; pinned by
/// `cerud/tests/lease_test.rs` + `cerulion_remoted/tests/estop_starvation_test.rs`.)
pub struct EngageEstopVerb {
    lease: Arc<Mutex<ControlLease>>,
    /// Who to tell that an incident was declared.
    ///
    /// NOT an `Option`. A verb built by the only constructor a production build
    /// can reach always has a real ask, so "the e-stop silently captures
    /// nothing" is not a state this type can be in — see
    /// [`EngageEstopVerb::with_capture_ask`], which is the ONLY way to substitute
    /// one and is compiled out of a plain `cargo build`.
    capture: Arc<dyn EstopCaptureAsk>,
}

impl EngageEstopVerb {
    /// Build the e-stop verb over the shared control lease.
    ///
    /// Wires the PRODUCTION capture ask. It opens no transport here and none at
    /// rest — only an actual engage reaches iceoryx2, on a detached thread (see
    /// [`crate::flashback`]).
    pub fn new(lease: Arc<Mutex<ControlLease>>) -> Self {
        EngageEstopVerb {
            lease,
            capture: Arc::new(TransportAsk::new()),
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
        lease: Arc<Mutex<ControlLease>>,
        capture: Arc<dyn EstopCaptureAsk>,
    ) -> Self {
        EngageEstopVerb { lease, capture }
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
        let mut lease = self.lease.lock().unwrap_or_else(|p| p.into_inner());
        let safe = lease.engage_estop(&caller.id);
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

    #[test]
    fn code_pair_start_without_an_armed_code_is_a_loud_verb_error() {
        // Never fail-open: a code-pair-start with no armed guest code refuses.
        let sessions = SharedCodePairSessions::new();
        let err = sessions
            .start([1u8; 32], PublicKey([2u8; 32]), b"msg1", 0)
            .unwrap_err();
        assert!(
            err.to_string().contains("no code pairing is armed"),
            "{err}"
        );
    }

    #[test]
    fn requires_caller_is_a_loud_fail_closed_error_for_each_bootstrap_verb() {
        // The caller-less `execute` of a bootstrap verb is a loud, fail-closed trap
        // (production always dispatches via `execute_with_caller`). Test it on the
        // e-stop verb (it needs only a lease, no trust store).
        let lease = Arc::new(Mutex::new(ControlLease::with_default_window()));
        let v = EngageEstopVerb::new(lease);
        let err = v.execute(&serde_json::json!({})).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("caller-bound bootstrap verb"), "{msg}");
        assert!(msg.contains("execute_with_caller"), "{msg}");
    }
}
