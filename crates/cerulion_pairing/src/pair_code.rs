// SPDX-License-Identifier: MIT OR Apache-2.0
//! The robot-local file handoff that arms a code pairing, and the policy a
//! code is armed under.
//!
//! A step on the robot, run as the user that owns the robot's state directory,
//! writes [`REQUEST_FILE`] into `<state_root>/remoted/`, mode 0600: it writes a
//! staging file whose name starts with [`REQUEST_STAGING_PREFIX`] (for example
//! `.pair-code.request.<pid>.tmp`), so a starting daemon removes one a writer
//! that crashed left behind, and renames it into place. The serving daemon
//! looks for a request every [`POLL_INTERVAL`], claims it by renaming it to
//! [`CLAIMED_FILE`], reads it through owner-only checks, removes it, arms the
//! code it carries under [`PAIR_CODE_POLICY`], and answers with
//! [`RESULT_FILE`], mode 0600, which repeats the request's nonce and never
//! carries the code. The daemon writes the result again each time the code's
//! state changes: a try fails, a desk pairs, the tries are used up or the
//! window ends. A request the daemon claimed but arms no code for is answered
//! `refused` with its reason ([`PairCodeRefusal`]), and so is an armed code the
//! daemon withdraws when it stops. The result answers the request the daemon
//! claimed last: a claimed request replaces the code an earlier one armed,
//! whether its own code is armed or it is refused.
//!
//! A starting daemon removes every file [`is_handoff_file`] names before it
//! reports that it is listening: a request left before that removal is never
//! armed. Until the removal succeeds no request is claimed, and the removal
//! that then succeeds also removes every request and staging file left in the
//! meantime, unarmed and unanswered. A writer cannot see when the removal ran,
//! so when its request is gone and no result naming its nonce appears within
//! two [`POLL_INTERVAL`]s, it leaves the request once more. Leaving it again
//! does no harm: a request the daemon refused is refused again, and one it
//! armed is armed again, its window counted from the new arm. A request the
//! daemon could not read or parse has no nonce the daemon knows, so its
//! `refused` result carries `-` for the nonce: a writer that removed the result
//! before it left its request, and has read no result naming its nonce since,
//! takes such a result, written after, as its answer. A writer that read a
//! result naming its nonce reads a later result naming another nonce, or `-`,
//! as its code replaced by a request claimed after its own.
//!
//! Both files are one line of printable ASCII ending in a single `\n`, at most
//! [`MAX_FILE_BYTES`] bytes, with fields separated by single spaces and an
//! exact field count:
//!
//! ```text
//! cerulion-pair-code-request/1 <nonce> <code>
//! cerulion-pair-code-result/1 <nonce> <status> <reason> <eid> <ttl_s> <tries> <failed> <desk_account> <desk_label>
//! ```
//!
//! - `<nonce>`: 32 lowercase hex digits (16 bytes) the writer picks; the result
//!   repeats it, so a writer reads only the answer to its own request. A
//!   `refused` result whose reason is `unreadable` or `malformed` carries `-`
//!   when the daemon could not read or parse the request's nonce; one whose
//!   reason is `unconfirmed` or `stopping` always carries the nonce.
//! - `<code>`: exactly [`CODE_LEN`] ASCII digits.
//! - `<status>`: `armed`, `paired`, `exhausted`, `expired` or `refused`
//!   ([`PairCodeStatus`]).
//! - `<reason>`: `-`, unless the status is `refused`: then why, `unreadable`,
//!   `malformed`, `unconfirmed` or `stopping` ([`PairCodeRefusal`]).
//! - `<eid>`: 64 lowercase hex digits, the endpoint id of the robot that armed
//!   the code, or that refused the request.
//! - `<ttl_s>`: the code's window in whole seconds, counted from the arm, at
//!   least 1, in decimal with no leading zero; a request refused before its
//!   code was armed states the window it would have been armed for.
//! - `<tries>`: the tries every desk shares, one decimal digit from 1 to 5.
//! - `<failed>`: how many of them failed, a decimal from 0 to `<tries>`:
//!   `exhausted` has `<failed>` equal to `<tries>`, `armed`, `paired` and
//!   `refused` have fewer, and a request refused before its code was armed
//!   has 0.
//! - `<desk_account>`, `<desk_label>`: `-` and `-`, unless the status is
//!   `paired`: then the account the desk was paired as, 64 lowercase hex
//!   digits, and the label it gave, 1 to [`LABEL_MAX`] bytes of `A-Z`, `a-z`,
//!   `0-9`, `.`, `_` and `-`, other than `-` alone ([`label_token`]).
//!
//! Everything here is pure: rendering and parsing touch no file.

use core::time::Duration;

use crate::error::PairingError;
use crate::pake::CeremonyConfig;

/// How many digits a pairing code has.
pub const CODE_LEN: usize = 8;
/// How many tries every desk shares on one armed code.
pub const PAIR_CODE_TRIES: u8 = 3;
/// How long an armed code works, counted from the arm on the daemon's
/// monotonic clock.
pub const PAIR_CODE_WINDOW: Duration = Duration::from_secs(600);
/// The shortest time between two admitted tries on one robot, whatever code
/// is armed.
pub const PAIR_CODE_RATE_INTERVAL: Duration = Duration::from_secs(2);
/// The policy a code from [`REQUEST_FILE`] is armed under:
/// [`PAIR_CODE_TRIES`] tries in [`PAIR_CODE_WINDOW`].
pub const PAIR_CODE_POLICY: CeremonyConfig = CeremonyConfig {
    max_attempts: PAIR_CODE_TRIES,
    ttl_ns: PAIR_CODE_WINDOW.as_nanos() as u64,
};
const _: () = assert!(
    PAIR_CODE_TRIES >= 1 && PAIR_CODE_TRIES <= 5,
    "the tries are what CeremonyConfig::new accepts"
);
const _: () = assert!(
    PAIR_CODE_WINDOW.as_secs() >= 1 && PAIR_CODE_WINDOW.subsec_nanos() == 0,
    "the window is a whole number of seconds a result line can state"
);

/// The request a robot-local step leaves for the daemon.
pub const REQUEST_FILE: &str = "pair-code.request";
/// The name the daemon renames a request to while it reads it.
pub const CLAIMED_FILE: &str = "pair-code.request.applying";
/// The daemon's answer to an armed request.
pub const RESULT_FILE: &str = "pair-code.result";
/// The daemon's staging file for [`RESULT_FILE`].
pub const RESULT_STAGING_FILE: &str = ".pair-code.result.tmp";
/// The start of the name of a writer's staging file for [`REQUEST_FILE`]; the
/// writer adds a part of its own, such as its process id and `.tmp`.
pub const REQUEST_STAGING_PREFIX: &str = ".pair-code.request.";
/// How often the serving daemon looks for a request. While claims work, a
/// request waits about this long before its code is armed.
pub const POLL_INTERVAL: Duration = Duration::from_secs(1);
/// The longest request or result file, in bytes.
pub const MAX_FILE_BYTES: usize = 512;
/// The longest desk label a result carries, in bytes.
pub const LABEL_MAX: usize = 64;
/// The first field of a request line.
pub const REQUEST_TAG: &str = "cerulion-pair-code-request/1";
/// The first field of a result line.
pub const RESULT_TAG: &str = "cerulion-pair-code-result/1";
/// The token of a result field that has no value: the desk fields of a
/// result whose status is not `paired`, the reason of one that is not
/// `refused`, and the nonce of a `refused` result whose request's nonce the
/// daemon does not know.
const ABSENT_FIELD: &str = "-";
/// The names every file of the handoff starts with: the request, its claimed
/// name and its staging names ([`REQUEST_STAGING_PREFIX`]), the result and its
/// staging name.
const HANDOFF_PREFIXES: [&str; 2] = ["pair-code.", ".pair-code."];
const NS_PER_S: u64 = 1_000_000_000;

/// Whether `name` is a file of the handoff (see the module docs).
pub fn is_handoff_file(name: &str) -> bool {
    HANDOFF_PREFIXES
        .iter()
        .any(|prefix| name.starts_with(prefix))
}

/// `label` as a result carries it: each byte outside `A-Z`, `a-z`, `0-9`,
/// `.`, `_` and `-` becomes `_`, the first [`LABEL_MAX`] bytes are kept, and an
/// empty label, or `-` alone (a result's absent field), becomes `_`.
pub fn label_token(label: &str) -> String {
    let token: String = label
        .bytes()
        .take(LABEL_MAX)
        .map(|b| if is_label_byte(b) { char::from(b) } else { '_' })
        .collect();
    if token.is_empty() || token == ABSENT_FIELD {
        "_".to_string()
    } else {
        token
    }
}

/// The 16 bytes a writer picks for one request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PairCodeNonce([u8; 16]);

impl PairCodeNonce {
    /// The nonce made of `bytes`.
    pub const fn from_bytes(bytes: [u8; 16]) -> Self {
        PairCodeNonce(bytes)
    }

    /// The nonce's bytes.
    pub fn as_bytes(&self) -> &[u8; 16] {
        &self.0
    }
}

/// A request to arm `code`, as [`REQUEST_FILE`] carries it.
#[derive(Clone, PartialEq, Eq)]
pub struct PairCodeRequest {
    nonce: PairCodeNonce,
    code: String,
}

impl core::fmt::Debug for PairCodeRequest {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Never print the code.
        f.debug_struct("PairCodeRequest")
            .field("nonce", &self.nonce)
            .finish_non_exhaustive()
    }
}

impl PairCodeRequest {
    /// A request for `code`.
    ///
    /// # Errors
    ///
    /// [`PairingError::PairCodeFile`] when `code` is not exactly [`CODE_LEN`]
    /// ASCII digits.
    pub fn new(nonce: PairCodeNonce, code: impl Into<String>) -> Result<Self, PairingError> {
        let code = code.into();
        if !is_code(&code) {
            return Err(Rule::Code.error());
        }
        Ok(PairCodeRequest { nonce, code })
    }

    /// The nonce the result repeats.
    pub fn nonce(&self) -> PairCodeNonce {
        self.nonce
    }

    /// The code to arm.
    pub fn code(&self) -> &str {
        &self.code
    }

    /// The request line.
    pub fn render(&self) -> Vec<u8> {
        format!("{REQUEST_TAG} {} {}\n", hex(&self.nonce.0), self.code).into_bytes()
    }

    /// Parse a request line.
    ///
    /// # Errors
    ///
    /// [`PairingError::PairCodeFile`] naming the first rule the bytes break.
    pub fn parse(bytes: &[u8]) -> Result<Self, PairingError> {
        let [tag, nonce, code] = fields::<3>(bytes).map_err(Rule::error)?;
        if tag != REQUEST_TAG {
            return Err(Rule::RequestTag.error());
        }
        let nonce = PairCodeNonce(lower_hex::<16>(nonce).ok_or(Rule::Nonce.error())?);
        PairCodeRequest::new(nonce, code)
    }
}

/// The status token of a result line, and of the daemon's `status` reply.
/// A `refused` result names its reason in a field of its own
/// ([`PairCodeState::Refused`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PairCodeStatus {
    /// The code is armed: it works for a desk with tries left, until its
    /// window ends.
    Armed,
    /// A desk paired with the code; it works for no other.
    Paired,
    /// Every try failed and no desk paired.
    Exhausted,
    /// The window ended and no desk paired.
    Expired,
    /// The daemon arms no code for the request, or no longer: see the
    /// result's reason.
    Refused,
}

impl PairCodeStatus {
    /// The token a result line carries.
    pub const fn as_str(self) -> &'static str {
        match self {
            PairCodeStatus::Armed => "armed",
            PairCodeStatus::Paired => "paired",
            PairCodeStatus::Exhausted => "exhausted",
            PairCodeStatus::Expired => "expired",
            PairCodeStatus::Refused => "refused",
        }
    }

    /// Whether no later state follows this one for the same arm.
    pub const fn is_terminal(self) -> bool {
        !matches!(self, PairCodeStatus::Armed)
    }

    fn parse(token: &str) -> Option<Self> {
        [
            PairCodeStatus::Armed,
            PairCodeStatus::Paired,
            PairCodeStatus::Exhausted,
            PairCodeStatus::Expired,
            PairCodeStatus::Refused,
        ]
        .into_iter()
        .find(|status| status.as_str() == token)
    }
}

/// What a robot-side step shows for a request refused as
/// [`PairCodeRefusal::Unreadable`], whichever check the read or the removal
/// failed.
pub const REFUSED_UNREADABLE: &str = "the robot's daemon could not read the pairing code \
                                      request, or remove it once read: write the request again \
                                      as the user the daemon runs as, as a regular file of at \
                                      most 512 bytes with no other hard link, in the remoted \
                                      state directory; the file and the directory must be owned \
                                      by that user, owner-only and not symbolic links";
/// What a robot-side step shows for a request refused as
/// [`PairCodeRefusal::Malformed`].
pub const REFUSED_MALFORMED: &str = "the robot's daemon refused the pairing code request: it \
                                     is not a well-formed request of the version this daemon \
                                     reads";
/// What a robot-side step shows for a request refused as
/// [`PairCodeRefusal::Unconfirmed`].
pub const REFUSED_UNCONFIRMED: &str = "the robot's daemon did not arm the pairing code: it \
                                       could not write the result that confirms it";
/// What a robot-side step shows for a code withdrawn as
/// [`PairCodeRefusal::Stopping`].
pub const REFUSED_STOPPING: &str = "the robot's daemon stopped before a desk paired with the \
                                    pairing code; the code no longer works";

/// Why a result is `refused`: its reason token and the text a robot-side step
/// shows for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PairCodeRefusal {
    /// The daemon claimed the request but could not read it through the
    /// owner-only checks, or could not remove its claimed file.
    Unreadable,
    /// The request was read but is not a well-formed request line.
    Malformed,
    /// The request was well formed, but its code is not armed: the result that
    /// confirms it could not be written.
    Unconfirmed,
    /// The daemon withdrew the armed code when it stopped.
    Stopping,
}

impl PairCodeRefusal {
    /// The token a result line carries in its reason field.
    pub const fn as_str(self) -> &'static str {
        match self {
            PairCodeRefusal::Unreadable => "unreadable",
            PairCodeRefusal::Malformed => "malformed",
            PairCodeRefusal::Unconfirmed => "unconfirmed",
            PairCodeRefusal::Stopping => "stopping",
        }
    }

    /// The text a robot-side step shows for this reason.
    pub const fn text(self) -> &'static str {
        match self {
            PairCodeRefusal::Unreadable => REFUSED_UNREADABLE,
            PairCodeRefusal::Malformed => REFUSED_MALFORMED,
            PairCodeRefusal::Unconfirmed => REFUSED_UNCONFIRMED,
            PairCodeRefusal::Stopping => REFUSED_STOPPING,
        }
    }

    /// The reason whose token is `token`; `None` for any other.
    pub fn parse(token: &str) -> Option<Self> {
        [
            PairCodeRefusal::Unreadable,
            PairCodeRefusal::Malformed,
            PairCodeRefusal::Unconfirmed,
            PairCodeRefusal::Stopping,
        ]
        .into_iter()
        .find(|refusal| refusal.as_str() == token)
    }
}

/// The armed code's state in the daemon's `status` reply: its status, the
/// tries every desk shares and how many of them failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PairCodeReport {
    /// What the code came to so far.
    pub status: PairCodeStatus,
    /// The tries every desk shares.
    pub tries: u8,
    /// How many of them failed.
    pub failed: u8,
}

/// The desk a code paired: the account it was paired as and its label token.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PairedDesk {
    account: [u8; 32],
    label: String,
}

impl PairedDesk {
    /// The desk paired as `account` under `label`.
    ///
    /// # Errors
    ///
    /// [`PairingError::PairCodeFile`] when `label` is not 1 to [`LABEL_MAX`]
    /// bytes of the label set ([`label_token`] makes one that is).
    pub fn new(account: [u8; 32], label: impl Into<String>) -> Result<Self, PairingError> {
        let label = label.into();
        if !is_label(&label) {
            return Err(Rule::DeskLabel.error());
        }
        Ok(PairedDesk { account, label })
    }

    /// The desk paired as `account` under the token of `label`
    /// ([`label_token`]).
    pub fn tokenized(account: [u8; 32], label: &str) -> Self {
        PairedDesk {
            account,
            label: label_token(label),
        }
    }

    /// The account the desk was paired as.
    pub fn account(&self) -> &[u8; 32] {
        &self.account
    }

    /// The desk's label token.
    pub fn label(&self) -> &str {
        &self.label
    }
}

/// What a code came to, as a result line states it; only a paired code names
/// its desk, and only a refused one a reason.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PairCodeState {
    /// See [`PairCodeStatus::Armed`].
    Armed,
    /// See [`PairCodeStatus::Paired`].
    Paired(PairedDesk),
    /// See [`PairCodeStatus::Exhausted`].
    Exhausted,
    /// See [`PairCodeStatus::Expired`].
    Expired,
    /// See [`PairCodeStatus::Refused`].
    Refused(PairCodeRefusal),
}

impl PairCodeState {
    /// The status token of this state.
    pub fn status(&self) -> PairCodeStatus {
        match self {
            PairCodeState::Armed => PairCodeStatus::Armed,
            PairCodeState::Paired(_) => PairCodeStatus::Paired,
            PairCodeState::Exhausted => PairCodeStatus::Exhausted,
            PairCodeState::Expired => PairCodeStatus::Expired,
            PairCodeState::Refused(_) => PairCodeStatus::Refused,
        }
    }
}

/// The daemon's answer to a request, as [`RESULT_FILE`] carries it. It holds
/// no code.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PairCodeResult {
    nonce: Option<PairCodeNonce>,
    endpoint_id: [u8; 32],
    ttl_s: u64,
    tries: u8,
    failed: u8,
    state: PairCodeState,
}

impl PairCodeResult {
    /// The first answer for a code armed under `policy` by the robot whose
    /// endpoint id is `endpoint_id`: `armed`, no try failed, the window
    /// `policy.ttl_ns` in whole seconds, rounded down, and the tries
    /// `policy.max_attempts`.
    ///
    /// # Errors
    ///
    /// [`PairingError::PairCodeFile`] when `policy` is outside what
    /// [`CeremonyConfig::new`] accepts or its window is under one second.
    pub fn armed(
        nonce: PairCodeNonce,
        endpoint_id: [u8; 32],
        policy: &CeremonyConfig,
    ) -> Result<Self, PairingError> {
        Self::under_policy(Some(nonce), endpoint_id, policy, PairCodeState::Armed)
    }

    /// The answer to a request refused as `refusal` before its code was armed,
    /// by the robot whose endpoint id is `endpoint_id`, which arms codes under
    /// `policy`: `refused`, no try failed, and the window and tries of
    /// `policy` as [`PairCodeResult::armed`] states them. `nonce` is `None`
    /// when the request's nonce could not be read.
    ///
    /// # Errors
    ///
    /// As [`PairCodeResult::armed`], and when `nonce` is `None` and `refusal`
    /// is [`PairCodeRefusal::Unconfirmed`] or [`PairCodeRefusal::Stopping`],
    /// which always name the request's nonce.
    pub fn refused(
        nonce: Option<PairCodeNonce>,
        endpoint_id: [u8; 32],
        policy: &CeremonyConfig,
        refusal: PairCodeRefusal,
    ) -> Result<Self, PairingError> {
        Self::under_policy(nonce, endpoint_id, policy, PairCodeState::Refused(refusal))
    }

    fn under_policy(
        nonce: Option<PairCodeNonce>,
        endpoint_id: [u8; 32],
        policy: &CeremonyConfig,
        state: PairCodeState,
    ) -> Result<Self, PairingError> {
        CeremonyConfig::new(policy.max_attempts, policy.ttl_ns)
            .map_err(|_| Rule::Policy.error())?;
        Self::build(
            nonce,
            endpoint_id,
            policy.ttl_ns / NS_PER_S,
            policy.max_attempts,
            0,
            state,
        )
    }

    /// The answer stating `state`, `failed` of `tries` tries failed, for the
    /// window `ttl_s`.
    ///
    /// # Errors
    ///
    /// [`PairingError::PairCodeFile`] naming the first rule the fields break:
    /// a window under one second, tries outside 1 to 5, or a failed count above
    /// the tries or that does not fit the state (see the module docs).
    pub fn new(
        nonce: PairCodeNonce,
        endpoint_id: [u8; 32],
        ttl_s: u64,
        tries: u8,
        failed: u8,
        state: PairCodeState,
    ) -> Result<Self, PairingError> {
        Self::build(Some(nonce), endpoint_id, ttl_s, tries, failed, state)
    }

    fn build(
        nonce: Option<PairCodeNonce>,
        endpoint_id: [u8; 32],
        ttl_s: u64,
        tries: u8,
        failed: u8,
        state: PairCodeState,
    ) -> Result<Self, PairingError> {
        if nonce.is_none() {
            match &state {
                PairCodeState::Refused(
                    PairCodeRefusal::Unreadable | PairCodeRefusal::Malformed,
                ) => {}
                PairCodeState::Refused(
                    PairCodeRefusal::Unconfirmed | PairCodeRefusal::Stopping,
                ) => return Err(Rule::NonceUnlessUnparsed.error()),
                PairCodeState::Armed
                | PairCodeState::Paired(_)
                | PairCodeState::Exhausted
                | PairCodeState::Expired => return Err(Rule::NonceUnlessRefused.error()),
            }
        }
        if ttl_s == 0 {
            return Err(Rule::Window.error());
        }
        if !(1..=5).contains(&tries) {
            return Err(Rule::Tries.error());
        }
        if failed > tries {
            return Err(Rule::Failed.error());
        }
        let fits = match state {
            PairCodeState::Exhausted => failed == tries,
            PairCodeState::Armed | PairCodeState::Paired(_) | PairCodeState::Refused(_) => {
                failed < tries
            }
            PairCodeState::Expired => true,
        };
        if !fits {
            return Err(Rule::FailedForStatus.error());
        }
        Ok(PairCodeResult {
            nonce,
            endpoint_id,
            ttl_s,
            tries,
            failed,
            state,
        })
    }

    /// The nonce of the request this answers; `None` for a `refused` result
    /// whose request's nonce the daemon could not read.
    pub fn nonce(&self) -> Option<PairCodeNonce> {
        self.nonce
    }

    /// The endpoint id of the robot that armed the code, or refused the
    /// request.
    pub fn endpoint_id(&self) -> &[u8; 32] {
        &self.endpoint_id
    }

    /// The code's window in whole seconds, counted from the arm.
    pub fn ttl_s(&self) -> u64 {
        self.ttl_s
    }

    /// The tries every desk shares.
    pub fn tries(&self) -> u8 {
        self.tries
    }

    /// How many tries failed.
    pub fn failed(&self) -> u8 {
        self.failed
    }

    /// What the code came to.
    pub fn state(&self) -> &PairCodeState {
        &self.state
    }

    /// The status token of [`PairCodeResult::state`].
    pub fn status(&self) -> PairCodeStatus {
        self.state.status()
    }

    /// Why the result is `refused`; `None` for any other status.
    pub fn refusal(&self) -> Option<PairCodeRefusal> {
        match self.state {
            PairCodeState::Refused(refusal) => Some(refusal),
            _ => None,
        }
    }

    /// The result line.
    pub fn render(&self) -> Vec<u8> {
        let (account, label) = match &self.state {
            PairCodeState::Paired(desk) => (hex(&desk.account), desk.label.as_str()),
            _ => (ABSENT_FIELD.to_string(), ABSENT_FIELD),
        };
        let nonce = self.nonce.map_or(ABSENT_FIELD.to_string(), |n| hex(&n.0));
        let reason = self.refusal().map_or(ABSENT_FIELD, PairCodeRefusal::as_str);
        format!(
            "{RESULT_TAG} {nonce} {} {reason} {} {} {} {} {account} {label}\n",
            self.status().as_str(),
            hex(&self.endpoint_id),
            self.ttl_s,
            self.tries,
            self.failed,
        )
        .into_bytes()
    }

    /// Parse a result line.
    ///
    /// # Errors
    ///
    /// [`PairingError::PairCodeFile`] naming the first rule the bytes break.
    pub fn parse(bytes: &[u8]) -> Result<Self, PairingError> {
        let [tag, nonce, status, reason, eid, ttl_s, tries, failed, account, label] =
            fields::<10>(bytes).map_err(Rule::error)?;
        if tag != RESULT_TAG {
            return Err(Rule::ResultTag.error());
        }
        let nonce = match nonce {
            ABSENT_FIELD => None,
            hex => Some(PairCodeNonce(
                lower_hex::<16>(hex).ok_or(Rule::Nonce.error())?,
            )),
        };
        let status = PairCodeStatus::parse(status).ok_or(Rule::Status.error())?;
        let refusal = match status {
            PairCodeStatus::Refused => {
                Some(PairCodeRefusal::parse(reason).ok_or(Rule::Reason.error())?)
            }
            _ if reason != ABSENT_FIELD => return Err(Rule::ReasonUnlessRefused.error()),
            _ => None,
        };
        let endpoint_id = lower_hex::<32>(eid).ok_or(Rule::EndpointId.error())?;
        let ttl_s = decimal(ttl_s).ok_or(Rule::Window.error())?;
        let tries = decimal(tries)
            .and_then(|t| u8::try_from(t).ok())
            .ok_or(Rule::Tries.error())?;
        let failed = decimal(failed)
            .and_then(|f| u8::try_from(f).ok())
            .ok_or(Rule::Failed.error())?;
        let state = match status {
            PairCodeStatus::Paired => {
                let account = lower_hex::<32>(account).ok_or(Rule::DeskAccount.error())?;
                PairCodeState::Paired(PairedDesk::new(account, label)?)
            }
            _ if account != ABSENT_FIELD || label != ABSENT_FIELD => {
                return Err(Rule::DeskUnlessPaired.error())
            }
            PairCodeStatus::Armed => PairCodeState::Armed,
            PairCodeStatus::Exhausted => PairCodeState::Exhausted,
            PairCodeStatus::Expired => PairCodeState::Expired,
            PairCodeStatus::Refused => PairCodeState::Refused(refusal.ok_or(Rule::Reason.error())?),
        };
        Self::build(nonce, endpoint_id, ttl_s, tries, failed, state)
    }
}

/// Each rule a handoff file or its policy can break; [`Rule::error`] is the
/// refusal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Rule {
    TooLong,
    NoNewline,
    NotPrintable,
    Separator,
    FieldCount,
    RequestTag,
    ResultTag,
    Nonce,
    NonceUnlessRefused,
    NonceUnlessUnparsed,
    Code,
    Status,
    Reason,
    ReasonUnlessRefused,
    EndpointId,
    Window,
    Tries,
    Failed,
    FailedForStatus,
    DeskUnlessPaired,
    DeskAccount,
    DeskLabel,
    Policy,
}

impl Rule {
    fn error(self) -> PairingError {
        PairingError::PairCodeFile(match self {
            Rule::TooLong => "the file is longer than 512 bytes",
            Rule::NoNewline => "the file is not one line ending in a newline",
            Rule::NotPrintable => "the line holds a byte that is not printable ASCII",
            Rule::Separator => "the fields are not separated by single spaces",
            Rule::FieldCount => "the line has the wrong number of fields",
            Rule::RequestTag => "the line is not a version 1 pair-code request",
            Rule::ResultTag => "the line is not a version 1 pair-code result",
            Rule::Nonce => "the nonce is not 32 lowercase hex digits",
            Rule::Code => "the code is not 8 ASCII digits",
            Rule::NonceUnlessRefused => "the nonce is `-` and the status is not refused",
            Rule::NonceUnlessUnparsed => {
                "the nonce is `-` and the refusal reason is not unreadable or malformed"
            }
            Rule::Status => "the result status is not armed, paired, exhausted, expired or refused",
            Rule::Reason => {
                "the refusal reason is not unreadable, malformed, unconfirmed or stopping"
            }
            Rule::ReasonUnlessRefused => {
                "the refusal reason is not `-` and the status is not refused"
            }
            Rule::EndpointId => "the endpoint id is not 64 lowercase hex digits",
            Rule::Window => {
                "the window is not a whole number of seconds from 1, without a leading zero"
            }
            Rule::Tries => "the tries are not a number from 1 to 5",
            Rule::Failed => "the failed tries are not a number from 0 to the tries",
            Rule::FailedForStatus => {
                "the failed tries do not fit the status: exhausted has every try failed, \
                 armed, paired and refused fewer"
            }
            Rule::DeskUnlessPaired => "the desk fields are not `-` and the status is not paired",
            Rule::DeskAccount => "the desk account is not 64 lowercase hex digits",
            Rule::DeskLabel => {
                "the desk label is not 1 to 64 bytes of A-Z, a-z, 0-9, '.', '_' and '-', \
                 other than `-` alone"
            }
            Rule::Policy => "the ceremony policy is outside the accepted range",
        })
    }
}

/// The `N` space-separated fields of a one-line file.
fn fields<const N: usize>(bytes: &[u8]) -> Result<[&str; N], Rule> {
    if bytes.len() > MAX_FILE_BYTES {
        return Err(Rule::TooLong);
    }
    let line = bytes.strip_suffix(b"\n").ok_or(Rule::NoNewline)?;
    let line = core::str::from_utf8(line).map_err(|_| Rule::NotPrintable)?;
    if !line.bytes().all(|b| (b' '..=b'~').contains(&b)) {
        return Err(if line.contains('\n') {
            Rule::NoNewline
        } else {
            Rule::NotPrintable
        });
    }
    let mut out = [""; N];
    let mut split = line.split(' ');
    for slot in &mut out {
        let field = split.next().ok_or(Rule::FieldCount)?;
        if field.is_empty() {
            return Err(Rule::Separator);
        }
        *slot = field;
    }
    match split.next() {
        None => Ok(out),
        Some("") => Err(Rule::Separator),
        Some(_) => Err(Rule::FieldCount),
    }
}

fn is_code(code: &str) -> bool {
    code.len() == CODE_LEN && code.bytes().all(|b| b.is_ascii_digit())
}

fn is_label_byte(b: u8) -> bool {
    b.is_ascii_alphanumeric() || matches!(b, b'.' | b'_' | b'-')
}

fn is_label(label: &str) -> bool {
    (1..=LABEL_MAX).contains(&label.len())
        && label.bytes().all(is_label_byte)
        && label != ABSENT_FIELD
}

fn hex(bytes: &[u8]) -> String {
    crate::crypto::to_hex(bytes)
}

/// Exactly `2 * N` lowercase hex digits.
fn lower_hex<const N: usize>(text: &str) -> Option<[u8; N]> {
    fn digit(b: u8) -> Option<u8> {
        match b {
            b'0'..=b'9' => Some(b - b'0'),
            b'a'..=b'f' => Some(b - b'a' + 10),
            _ => None,
        }
    }
    let text = text.as_bytes();
    if text.len() != 2 * N {
        return None;
    }
    let mut out = [0u8; N];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (digit(text[2 * i])? << 4) | digit(text[2 * i + 1])?;
    }
    Some(out)
}

/// A decimal with no sign and no leading zero.
fn decimal(text: &str) -> Option<u64> {
    let canonical = !text.is_empty()
        && text.bytes().all(|b| b.is_ascii_digit())
        && (text == "0" || !text.starts_with('0'));
    if canonical {
        text.parse().ok()
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NONCE: PairCodeNonce = PairCodeNonce([
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
        0x0f,
    ]);
    const NONCE_HEX: &str = "000102030405060708090a0b0c0d0e0f";
    const EID_HEX: &str = "abababababababababababababababababababababababababababababababab";
    const DESK_HEX: &str = "cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd";

    fn reason(e: PairingError) -> &'static str {
        match e {
            PairingError::PairCodeFile(reason) => reason,
            other => panic!("expected a pair-code file error, got {other:?}"),
        }
    }

    fn desk(label: &str) -> PairCodeState {
        PairCodeState::Paired(PairedDesk::new([0xcd; 32], label).unwrap())
    }

    #[test]
    fn the_policy_is_three_tries_in_ten_minutes_one_try_every_two_seconds() {
        assert_eq!(CODE_LEN, 8);
        assert_eq!(PAIR_CODE_TRIES, 3);
        assert_eq!(PAIR_CODE_WINDOW, Duration::from_secs(600));
        assert_eq!(PAIR_CODE_RATE_INTERVAL, Duration::from_secs(2));
        assert_eq!(
            PAIR_CODE_POLICY,
            CeremonyConfig::new(3, 600_000_000_000).unwrap()
        );
    }

    #[test]
    fn request_line_matches_the_hand_written_oracle() {
        let request = PairCodeRequest::new(NONCE, "01234567").unwrap();
        let line = b"cerulion-pair-code-request/1 000102030405060708090a0b0c0d0e0f 01234567\n";
        assert_eq!(request.render(), line.to_vec());
        assert_eq!(
            request.render(),
            request.render(),
            "rendering is deterministic"
        );
        let parsed = PairCodeRequest::parse(line).unwrap();
        assert_eq!(parsed.nonce(), NONCE);
        assert_eq!(parsed.code(), "01234567");
        assert_eq!(parsed, request);
    }

    #[test]
    fn a_request_carries_exactly_eight_ascii_digits() {
        for code in ["00000000", "99999999", "48213907"] {
            let line = format!("{REQUEST_TAG} {NONCE_HEX} {code}\n");
            let parsed = PairCodeRequest::parse(line.as_bytes()).unwrap();
            assert_eq!(parsed.code(), code);
        }
        for code in [
            "1234567",
            "123456789",
            "1234567A",
            "ABCDEFGH",
            "1234 567",
            "1234-567",
            "١٢٣٤٥٦٧٨",
            "",
        ] {
            assert_eq!(
                reason(PairCodeRequest::new(NONCE, code).unwrap_err()),
                "the code is not 8 ASCII digits",
                "{code:?}"
            );
        }
        // The refusal names the code length the grammar checks.
        assert!(reason(Rule::Code.error()).contains(&format!("{CODE_LEN} ASCII digits")));
    }

    #[test]
    fn parse_request_refuses_each_malformed_form() {
        let good = format!("{REQUEST_TAG} {NONCE_HEX} 12345678\n");
        let long = format!("{REQUEST_TAG} {NONCE_HEX} {}\n", "1".repeat(600));
        let cases: Vec<(Vec<u8>, &str)> = vec![
            (
                format!("cerulion-pair-code-request/2 {NONCE_HEX} 12345678\n").into_bytes(),
                "the line is not a version 1 pair-code request",
            ),
            (
                format!("cerulion-pair-code-result/1 {NONCE_HEX} 12345678\n").into_bytes(),
                "the line is not a version 1 pair-code request",
            ),
            (
                format!("{REQUEST_TAG} {} 12345678\n", NONCE_HEX.to_uppercase()).into_bytes(),
                "the nonce is not 32 lowercase hex digits",
            ),
            (
                format!("{REQUEST_TAG} {} 12345678\n", &NONCE_HEX[1..]).into_bytes(),
                "the nonce is not 32 lowercase hex digits",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX}0 12345678\n").into_bytes(),
                "the nonce is not 32 lowercase hex digits",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX} 1234567\n").into_bytes(),
                "the code is not 8 ASCII digits",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX} 1234567a\n").into_bytes(),
                "the code is not 8 ASCII digits",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX} 12345678 EXTRA\n").into_bytes(),
                "the line has the wrong number of fields",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX}\n").into_bytes(),
                "the line has the wrong number of fields",
            ),
            (
                good.trim_end().as_bytes().to_vec(),
                "the file is not one line ending in a newline",
            ),
            (
                format!("{}\n", good).into_bytes(),
                "the file is not one line ending in a newline",
            ),
            (
                format!("{}\r\n", good.trim_end()).into_bytes(),
                "the line holds a byte that is not printable ASCII",
            ),
            (
                format!("{REQUEST_TAG}\t{NONCE_HEX} 12345678\n").into_bytes(),
                "the line holds a byte that is not printable ASCII",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX} 12345678\u{e9}\n").into_bytes(),
                "the line holds a byte that is not printable ASCII",
            ),
            (
                [
                    format!("{REQUEST_TAG} {NONCE_HEX} 12345678").as_bytes(),
                    b"\xff\n",
                ]
                .concat(),
                "the line holds a byte that is not printable ASCII",
            ),
            (
                format!(" {good}").into_bytes(),
                "the fields are not separated by single spaces",
            ),
            (
                format!("{REQUEST_TAG}  {NONCE_HEX} 12345678\n").into_bytes(),
                "the fields are not separated by single spaces",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX} 12345678 \n").into_bytes(),
                "the fields are not separated by single spaces",
            ),
            (long.into_bytes(), "the file is longer than 512 bytes"),
            (Vec::new(), "the file is not one line ending in a newline"),
        ];
        for (bytes, want) in cases {
            assert_eq!(
                reason(PairCodeRequest::parse(&bytes).unwrap_err()),
                want,
                "{:?}",
                String::from_utf8_lossy(&bytes)
            );
        }
        // The size bound is exactly 512 bytes: a 512-byte line is read for its
        // fields, a 513-byte one is refused before.
        let at_bound = format!("{REQUEST_TAG} {NONCE_HEX} {}\n", "1".repeat(512 - 63));
        assert_eq!(at_bound.len(), 512);
        assert_eq!(
            reason(PairCodeRequest::parse(at_bound.as_bytes()).unwrap_err()),
            "the code is not 8 ASCII digits"
        );
        let past_bound = format!("{REQUEST_TAG} {NONCE_HEX} {}\n", "1".repeat(513 - 63));
        assert_eq!(
            reason(PairCodeRequest::parse(past_bound.as_bytes()).unwrap_err()),
            "the file is longer than 512 bytes"
        );
    }

    #[test]
    fn a_request_debug_never_prints_its_code() {
        let request = PairCodeRequest::new(NONCE, "48213907").unwrap();
        let shown = format!("{request:?}");
        assert!(!shown.contains("48213907"), "{shown}");
        assert!(shown.contains("PairCodeRequest"), "{shown}");
    }

    #[test]
    fn result_lines_match_the_hand_written_oracle_for_each_status() {
        let refused = |nonce, refusal| {
            PairCodeResult::refused(nonce, [0xab; 32], &PAIR_CODE_POLICY, refusal).unwrap()
        };
        let cases: Vec<(PairCodeResult, String)> = vec![
            (
                PairCodeResult::armed(NONCE, [0xab; 32], &PAIR_CODE_POLICY).unwrap(),
                format!("{RESULT_TAG} {NONCE_HEX} armed - {EID_HEX} 600 3 0 - -\n"),
            ),
            (
                PairCodeResult::new(NONCE, [0xab; 32], 600, 3, 2, PairCodeState::Armed).unwrap(),
                format!("{RESULT_TAG} {NONCE_HEX} armed - {EID_HEX} 600 3 2 - -\n"),
            ),
            (
                PairCodeResult::new(NONCE, [0xab; 32], 600, 3, 1, desk("lab-desk.2_a")).unwrap(),
                format!(
                    "{RESULT_TAG} {NONCE_HEX} paired - {EID_HEX} 600 3 1 {DESK_HEX} lab-desk.2_a\n"
                ),
            ),
            (
                PairCodeResult::new(NONCE, [0xab; 32], 600, 3, 3, PairCodeState::Exhausted)
                    .unwrap(),
                format!("{RESULT_TAG} {NONCE_HEX} exhausted - {EID_HEX} 600 3 3 - -\n"),
            ),
            (
                PairCodeResult::new(NONCE, [0xab; 32], 600, 3, 0, PairCodeState::Expired).unwrap(),
                format!("{RESULT_TAG} {NONCE_HEX} expired - {EID_HEX} 600 3 0 - -\n"),
            ),
            (
                PairCodeResult::new(NONCE, [0xab; 32], 600, 3, 3, PairCodeState::Expired).unwrap(),
                format!("{RESULT_TAG} {NONCE_HEX} expired - {EID_HEX} 600 3 3 - -\n"),
            ),
            (
                refused(None, PairCodeRefusal::Unreadable),
                format!("{RESULT_TAG} - refused unreadable {EID_HEX} 600 3 0 - -\n"),
            ),
            (
                refused(None, PairCodeRefusal::Malformed),
                format!("{RESULT_TAG} - refused malformed {EID_HEX} 600 3 0 - -\n"),
            ),
            (
                refused(Some(NONCE), PairCodeRefusal::Unconfirmed),
                format!("{RESULT_TAG} {NONCE_HEX} refused unconfirmed {EID_HEX} 600 3 0 - -\n"),
            ),
            (
                PairCodeResult::new(
                    NONCE,
                    [0xab; 32],
                    600,
                    3,
                    2,
                    PairCodeState::Refused(PairCodeRefusal::Stopping),
                )
                .unwrap(),
                format!("{RESULT_TAG} {NONCE_HEX} refused stopping {EID_HEX} 600 3 2 - -\n"),
            ),
        ];
        for (result, line) in cases {
            assert_eq!(String::from_utf8(result.render()).unwrap(), line);
            assert_eq!(result.render(), result.render(), "deterministic");
            assert_eq!(PairCodeResult::parse(line.as_bytes()).unwrap(), result);
        }
        let paired = PairCodeResult::parse(
            format!("{RESULT_TAG} {NONCE_HEX} paired - {EID_HEX} 600 3 0 {DESK_HEX} d\n")
                .as_bytes(),
        )
        .unwrap();
        assert_eq!(paired.nonce(), Some(NONCE));
        assert_eq!(paired.endpoint_id(), &[0xab; 32]);
        assert_eq!(
            (paired.ttl_s(), paired.tries(), paired.failed()),
            (600, 3, 0)
        );
        assert_eq!(paired.status(), PairCodeStatus::Paired);
        assert_eq!(paired.refusal(), None);
        match paired.state() {
            PairCodeState::Paired(desk) => {
                assert_eq!((desk.account(), desk.label()), (&[0xcd; 32], "d"))
            }
            other => panic!("expected a paired state, got {other:?}"),
        }
        let refused = PairCodeResult::parse(
            format!("{RESULT_TAG} - refused unreadable {EID_HEX} 600 3 0 - -\n").as_bytes(),
        )
        .unwrap();
        assert_eq!(refused.nonce(), None);
        assert_eq!(refused.status(), PairCodeStatus::Refused);
        assert_eq!(refused.refusal(), Some(PairCodeRefusal::Unreadable));
    }

    #[test]
    fn each_refusal_has_its_token_and_text_and_no_other_token_is_a_reason() {
        let cases = [
            (
                PairCodeRefusal::Unreadable,
                "unreadable",
                "the robot's daemon could not read the pairing code request, or remove it \
                 once read: write the request again as the user the daemon runs as, as a \
                 regular file of at most 512 bytes with no other hard link, in the remoted \
                 state directory; the file and the directory must be owned by that user, \
                 owner-only and not symbolic links",
            ),
            (
                PairCodeRefusal::Malformed,
                "malformed",
                "the robot's daemon refused the pairing code request: it is not a \
                 well-formed request of the version this daemon reads",
            ),
            (
                PairCodeRefusal::Unconfirmed,
                "unconfirmed",
                "the robot's daemon did not arm the pairing code: it could not write the \
                 result that confirms it",
            ),
            (
                PairCodeRefusal::Stopping,
                "stopping",
                "the robot's daemon stopped before a desk paired with the pairing code; the \
                 code no longer works",
            ),
        ];
        for (refusal, token, text) in cases {
            assert_eq!(refusal.as_str(), token);
            assert_eq!(PairCodeRefusal::parse(token), Some(refusal));
            assert_eq!(refusal.text(), text, "{token}");
        }
        // The unreadable text names the size bound the read checks.
        assert!(REFUSED_UNREADABLE.contains(&format!("at most {MAX_FILE_BYTES} bytes")));
        for token in [
            "",
            "-",
            "Unreadable",
            "UNREADABLE",
            "refused",
            "stopped",
            "unreadable ",
        ] {
            assert_eq!(PairCodeRefusal::parse(token), None, "{token:?}");
        }
    }

    #[test]
    fn a_refused_result_states_the_policy_and_no_failed_try_and_refuses_a_bad_policy() {
        let policy = CeremonyConfig::new(5, 90_999_999_999).unwrap();
        let refused =
            PairCodeResult::refused(Some(NONCE), [0xab; 32], &policy, PairCodeRefusal::Malformed)
                .unwrap();
        assert_eq!(
            (refused.ttl_s(), refused.tries(), refused.failed()),
            (90, 5, 0)
        );
        assert_eq!(
            reason(
                PairCodeResult::refused(
                    None,
                    [0xab; 32],
                    &CeremonyConfig::new(3, 999_999_999).unwrap(),
                    PairCodeRefusal::Unreadable,
                )
                .unwrap_err()
            ),
            "the window is not a whole number of seconds from 1, without a leading zero"
        );
        assert_eq!(
            reason(
                PairCodeResult::refused(
                    None,
                    [0xab; 32],
                    &CeremonyConfig {
                        max_attempts: 0,
                        ttl_ns: 120_000_000_000,
                    },
                    PairCodeRefusal::Unreadable,
                )
                .unwrap_err()
            ),
            "the ceremony policy is outside the accepted range"
        );
    }

    #[test]
    fn only_an_unreadable_or_malformed_refusal_is_built_without_the_nonce() {
        let unparsed = "the nonce is `-` and the refusal reason is not unreadable or malformed";
        let cases = [
            (PairCodeRefusal::Unreadable, None),
            (PairCodeRefusal::Malformed, None),
            (PairCodeRefusal::Unconfirmed, Some(unparsed)),
            (PairCodeRefusal::Stopping, Some(unparsed)),
        ];
        for (refusal, want) in cases {
            let built = PairCodeResult::refused(None, [0xab; 32], &PAIR_CODE_POLICY, refusal);
            assert_eq!(built.err().map(reason), want, "{refusal:?}");
            // With the nonce, every reason is built.
            PairCodeResult::refused(Some(NONCE), [0xab; 32], &PAIR_CODE_POLICY, refusal).unwrap();
        }
    }

    #[test]
    fn the_largest_result_fits_the_file_bound() {
        let result = PairCodeResult::new(
            NONCE,
            [0xff; 32],
            u64::MAX,
            5,
            4,
            desk(&"x".repeat(LABEL_MAX)),
        )
        .unwrap();
        let line = result.render();
        assert_eq!(line.len(), 290);
        assert!(line.len() <= MAX_FILE_BYTES);
        assert_eq!(PairCodeResult::parse(&line).unwrap(), result);
    }

    #[test]
    fn the_first_result_window_is_the_policy_ttl_in_whole_seconds_rounded_down() {
        let policy = CeremonyConfig::new(5, 90_999_999_999).unwrap();
        let armed = PairCodeResult::armed(NONCE, [0xab; 32], &policy).unwrap();
        assert_eq!((armed.ttl_s(), armed.tries(), armed.failed()), (90, 5, 0));
        let policy = CeremonyConfig::new(1, 1_000_000_000).unwrap();
        let armed = PairCodeResult::armed(NONCE, [0xab; 32], &policy).unwrap();
        assert_eq!((armed.ttl_s(), armed.tries()), (1, 1));
    }

    #[test]
    fn a_result_is_refused_for_a_policy_outside_the_accepted_range() {
        let cases = [
            (
                CeremonyConfig::new(3, 999_999_999).unwrap(),
                "the window is not a whole number of seconds from 1, without a leading zero",
            ),
            (
                CeremonyConfig {
                    max_attempts: 0,
                    ttl_ns: 120_000_000_000,
                },
                "the ceremony policy is outside the accepted range",
            ),
            (
                CeremonyConfig {
                    max_attempts: 6,
                    ttl_ns: 120_000_000_000,
                },
                "the ceremony policy is outside the accepted range",
            ),
            (
                CeremonyConfig {
                    max_attempts: 3,
                    ttl_ns: 0,
                },
                "the ceremony policy is outside the accepted range",
            ),
        ];
        for (policy, want) in cases {
            assert_eq!(
                reason(PairCodeResult::armed(NONCE, [0xab; 32], &policy).unwrap_err()),
                want,
                "{policy:?}"
            );
        }
    }

    #[test]
    fn a_result_whose_counts_do_not_fit_its_status_cannot_be_built() {
        let failed_fit = "the failed tries do not fit the status: exhausted has every try \
                          failed, armed, paired and refused fewer";
        let cases: Vec<(u8, u8, PairCodeState, &str)> = vec![
            (
                3,
                4,
                PairCodeState::Expired,
                "the failed tries are not a number from 0 to the tries",
            ),
            (3, 3, PairCodeState::Armed, failed_fit),
            (3, 3, desk("d"), failed_fit),
            (
                3,
                3,
                PairCodeState::Refused(PairCodeRefusal::Stopping),
                failed_fit,
            ),
            (3, 2, PairCodeState::Exhausted, failed_fit),
            (
                0,
                0,
                PairCodeState::Expired,
                "the tries are not a number from 1 to 5",
            ),
            (
                6,
                0,
                PairCodeState::Armed,
                "the tries are not a number from 1 to 5",
            ),
        ];
        for (tries, failed, state, want) in cases {
            let label = format!("{tries} {failed} {state:?}");
            assert_eq!(
                reason(
                    PairCodeResult::new(NONCE, [0xab; 32], 600, tries, failed, state).unwrap_err()
                ),
                want,
                "{label}"
            );
        }
        assert_eq!(
            reason(
                PairCodeResult::new(NONCE, [0xab; 32], 0, 3, 0, PairCodeState::Armed).unwrap_err()
            ),
            "the window is not a whole number of seconds from 1, without a leading zero"
        );
    }

    #[test]
    fn parse_result_refuses_each_malformed_form() {
        /// A result line with these fields after the tag and the nonce.
        fn line(
            status: &str,
            eid: &str,
            ttl: &str,
            tries: &str,
            failed: &str,
            account: &str,
            label: &str,
        ) -> Vec<u8> {
            let reason = if status == "refused" { "stopping" } else { "-" };
            with(
                NONCE_HEX, status, reason, eid, ttl, tries, failed, account, label,
            )
        }
        #[allow(clippy::too_many_arguments)]
        fn with(
            nonce: &str,
            status: &str,
            reason: &str,
            eid: &str,
            ttl: &str,
            tries: &str,
            failed: &str,
            account: &str,
            label: &str,
        ) -> Vec<u8> {
            format!(
                "{RESULT_TAG} {nonce} {status} {reason} {eid} {ttl} {tries} {failed} {account} \
                 {label}\n"
            )
            .into_bytes()
        }
        let window = "the window is not a whole number of seconds from 1, without a leading zero";
        let tries = "the tries are not a number from 1 to 5";
        let failed = "the failed tries are not a number from 0 to the tries";
        let fit = "the failed tries do not fit the status: exhausted has every try failed, \
                   armed, paired and refused fewer";
        let unless_paired = "the desk fields are not `-` and the status is not paired";
        let account = "the desk account is not 64 lowercase hex digits";
        let label = "the desk label is not 1 to 64 bytes of A-Z, a-z, 0-9, '.', '_' and '-', other than `-` alone";
        let status = "the result status is not armed, paired, exhausted, expired or refused";
        let reason = "the refusal reason is not unreadable, malformed, unconfirmed or stopping";
        let reason_unless = "the refusal reason is not `-` and the status is not refused";
        let nonce_unless = "the nonce is `-` and the status is not refused";
        let nonce_unless_unparsed =
            "the nonce is `-` and the refusal reason is not unreadable or malformed";
        let long_label = "x".repeat(65);
        let cases: Vec<(Vec<u8>, &str)> = vec![
            (line("armed", EID_HEX, "600", "0", "0", "-", "-"), tries),
            (line("armed", EID_HEX, "600", "6", "0", "-", "-"), tries),
            (line("armed", EID_HEX, "600", "03", "0", "-", "-"), tries),
            (line("armed", EID_HEX, "600", "256", "0", "-", "-"), tries),
            (line("armed", EID_HEX, "600", "3", "4", "-", "-"), failed),
            (line("armed", EID_HEX, "600", "3", "01", "-", "-"), failed),
            (line("armed", EID_HEX, "600", "3", "-1", "-", "-"), failed),
            (line("armed", EID_HEX, "600", "3", "3", "-", "-"), fit),
            (line("exhausted", EID_HEX, "600", "3", "2", "-", "-"), fit),
            (line("paired", EID_HEX, "600", "3", "3", DESK_HEX, "d"), fit),
            (line("refused", EID_HEX, "600", "3", "3", "-", "-"), fit),
            (line("armed", EID_HEX, "0", "3", "0", "-", "-"), window),
            (line("armed", EID_HEX, "0600", "3", "0", "-", "-"), window),
            (line("armed", EID_HEX, "-1", "3", "0", "-", "-"), window),
            (line("armed", EID_HEX, "+5", "3", "0", "-", "-"), window),
            (
                line("armed", EID_HEX, "18446744073709551616", "3", "0", "-", "-"),
                window,
            ),
            (line("revoked", EID_HEX, "600", "3", "0", "-", "-"), status),
            (line("Armed", EID_HEX, "600", "3", "0", "-", "-"), status),
            (line("REFUSED", EID_HEX, "600", "3", "0", "-", "-"), status),
            (
                with(
                    NONCE_HEX, "refused", "-", EID_HEX, "600", "3", "0", "-", "-",
                ),
                reason,
            ),
            (
                with(
                    NONCE_HEX, "refused", "busy", EID_HEX, "600", "3", "0", "-", "-",
                ),
                reason,
            ),
            (
                with(
                    NONCE_HEX, "refused", "Stopping", EID_HEX, "600", "3", "0", "-", "-",
                ),
                reason,
            ),
            (
                with(
                    NONCE_HEX, "armed", "stopping", EID_HEX, "600", "3", "0", "-", "-",
                ),
                reason_unless,
            ),
            (
                with(
                    NONCE_HEX, "expired", "x", EID_HEX, "600", "3", "0", "-", "-",
                ),
                reason_unless,
            ),
            (
                with("-", "armed", "-", EID_HEX, "600", "3", "0", "-", "-"),
                nonce_unless,
            ),
            (
                with("-", "paired", "-", EID_HEX, "600", "3", "0", DESK_HEX, "d"),
                nonce_unless,
            ),
            (
                with(
                    "-",
                    "refused",
                    "unconfirmed",
                    EID_HEX,
                    "600",
                    "3",
                    "0",
                    "-",
                    "-",
                ),
                nonce_unless_unparsed,
            ),
            (
                with(
                    "-", "refused", "stopping", EID_HEX, "600", "3", "2", "-", "-",
                ),
                nonce_unless_unparsed,
            ),
            (
                line("armed", &EID_HEX[1..], "600", "3", "0", "-", "-"),
                "the endpoint id is not 64 lowercase hex digits",
            ),
            (
                line("armed", &EID_HEX.to_uppercase(), "600", "3", "0", "-", "-"),
                "the endpoint id is not 64 lowercase hex digits",
            ),
            (
                line("armed", EID_HEX, "600", "3", "0", DESK_HEX, "d"),
                unless_paired,
            ),
            (
                line("armed", EID_HEX, "600", "3", "0", "-", "d"),
                unless_paired,
            ),
            (
                line("expired", EID_HEX, "600", "3", "0", DESK_HEX, "-"),
                unless_paired,
            ),
            (
                line("exhausted", EID_HEX, "600", "3", "3", "-", "x"),
                unless_paired,
            ),
            (
                line("refused", EID_HEX, "600", "3", "0", DESK_HEX, "d"),
                unless_paired,
            ),
            (line("paired", EID_HEX, "600", "3", "0", "-", "-"), account),
            (line("paired", EID_HEX, "600", "3", "0", "-", "d"), account),
            (
                line(
                    "paired",
                    EID_HEX,
                    "600",
                    "3",
                    "0",
                    &DESK_HEX.to_uppercase(),
                    "d",
                ),
                account,
            ),
            (
                line("paired", EID_HEX, "600", "3", "0", DESK_HEX, "-"),
                label,
            ),
            (
                line("paired", EID_HEX, "600", "3", "0", DESK_HEX, "a/b"),
                label,
            ),
            (
                line("paired", EID_HEX, "600", "3", "0", DESK_HEX, "a~b"),
                label,
            ),
            (
                line("paired", EID_HEX, "600", "3", "0", DESK_HEX, &long_label),
                label,
            ),
            (
                with(
                    "zz0102030405060708090a0b0c0d0e0f",
                    "armed",
                    "-",
                    EID_HEX,
                    "600",
                    "3",
                    "0",
                    "-",
                    "-",
                ),
                "the nonce is not 32 lowercase hex digits",
            ),
            (
                with(
                    "--",
                    "refused",
                    "malformed",
                    EID_HEX,
                    "600",
                    "3",
                    "0",
                    "-",
                    "-",
                ),
                "the nonce is not 32 lowercase hex digits",
            ),
            (
                format!("{REQUEST_TAG} {NONCE_HEX} armed - {EID_HEX} 600 3 0 - -\n").into_bytes(),
                "the line is not a version 1 pair-code result",
            ),
            (
                format!("{RESULT_TAG} {NONCE_HEX} armed - {EID_HEX} 600 3 0 - - 7\n").into_bytes(),
                "the line has the wrong number of fields",
            ),
            (
                format!("{RESULT_TAG} {NONCE_HEX} armed {EID_HEX} 600 3 0 - -\n").into_bytes(),
                "the line has the wrong number of fields",
            ),
            (
                format!("{RESULT_TAG} {NONCE_HEX} armed - {EID_HEX} 600 3 0 - -").into_bytes(),
                "the file is not one line ending in a newline",
            ),
        ];
        for (bytes, want) in cases {
            assert_eq!(
                reason_of(&bytes),
                want,
                "{:?}",
                String::from_utf8_lossy(&bytes)
            );
        }
        // The largest window a u64 holds and a 64-byte label parse.
        let max = line(
            "paired",
            EID_HEX,
            "18446744073709551615",
            "5",
            "4",
            DESK_HEX,
            &"x".repeat(64),
        );
        let parsed = PairCodeResult::parse(&max).unwrap();
        assert_eq!(
            (parsed.ttl_s(), parsed.tries(), parsed.failed()),
            (u64::MAX, 5, 4)
        );
    }

    fn reason_of(bytes: &[u8]) -> &'static str {
        reason(PairCodeResult::parse(bytes).unwrap_err())
    }

    #[test]
    fn label_token_keeps_the_label_set_and_replaces_every_other_byte() {
        let cases: Vec<(String, String)> = vec![
            ("lab-desk".into(), "lab-desk".into()),
            ("Desk_2.local".into(), "Desk_2.local".into()),
            ("my desk".into(), "my_desk".into()),
            ("a/b\\c".into(), "a_b_c".into()),
            ("tab\there\nnl".into(), "tab_here_nl".into()),
            ("\u{7f}\u{0}".into(), "__".into()),
            // Each byte of a multi-byte character is replaced.
            ("café".into(), "caf__".into()),
            ("机器".into(), "______".into()),
            (String::new(), "_".into()),
            ("-".into(), "_".into()),
            ("--".into(), "--".into()),
            ("x".repeat(65), "x".repeat(64)),
            (
                format!("{}é", "x".repeat(63)),
                format!("{}_", "x".repeat(63)),
            ),
        ];
        for (label, want) in cases {
            let token = label_token(&label);
            assert_eq!(token, want, "{label:?}");
            assert_eq!(
                PairedDesk::new([0; 32], token).unwrap(),
                PairedDesk::tokenized([0; 32], &label),
                "{label:?}"
            );
        }
        assert_eq!(
            reason(PairedDesk::new([0; 32], "-").unwrap_err()),
            "the desk label is not 1 to 64 bytes of A-Z, a-z, 0-9, '.', '_' and '-', other \
             than `-` alone"
        );
        assert_eq!(
            reason(PairedDesk::new([0; 32], "").unwrap_err()),
            "the desk label is not 1 to 64 bytes of A-Z, a-z, 0-9, '.', '_' and '-', other than `-` alone"
        );
    }

    #[test]
    fn each_status_has_its_token_and_only_armed_is_not_terminal() {
        let cases = [
            (PairCodeStatus::Armed, "armed", false),
            (PairCodeStatus::Paired, "paired", true),
            (PairCodeStatus::Exhausted, "exhausted", true),
            (PairCodeStatus::Expired, "expired", true),
            (PairCodeStatus::Refused, "refused", true),
        ];
        for (status, token, terminal) in cases {
            assert_eq!(status.as_str(), token);
            assert_eq!(PairCodeStatus::parse(token), Some(status));
            assert_eq!(status.is_terminal(), terminal, "{token}");
        }
        assert_eq!(PairCodeStatus::parse("ARMED"), None);
    }

    #[test]
    fn is_handoff_file_names_every_file_of_the_handoff_and_nothing_else() {
        // A writer's staging file, named as the module docs say.
        assert!(".pair-code.request.4242.tmp".starts_with(REQUEST_STAGING_PREFIX));
        for name in [
            REQUEST_FILE,
            CLAIMED_FILE,
            RESULT_FILE,
            RESULT_STAGING_FILE,
            ".pair-code.request.4242.tmp",
            &format!("{REQUEST_STAGING_PREFIX}1.tmp"),
            "pair-code.request.withdrawn.4242",
        ] {
            assert!(is_handoff_file(name), "{name}");
        }
        for name in [
            "pair-code",
            "pair-codes.request",
            "xpair-code.request",
            "epoch.pending",
            ".epoch.pending.tmp",
            "trust_store",
            "",
        ] {
            assert!(!is_handoff_file(name), "{name}");
        }
    }
}
