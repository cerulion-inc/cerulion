// SPDX-License-Identifier: AGPL-3.0-only
//! The desk-side CPace pairing ceremony over the robot's `cerulion/ops/1` plane.
//!
//! `cerulion pair <robot>` spawns `cerulion-connectd pair …`, which runs the
//! **code-pairing** (CPace) ceremony the robot's `cerulion_remoted` implements:
//! the desk is the CPace *initiator*, the robot the *responder*. Given a short
//! pairing code the robot owner armed (and communicated to the desk operator),
//! the desk dials the ops plane, drives `code-pair-start` → `code-pair-finish`,
//! and — on success — the robot writes a durable access-list row for this desk's
//! device key. Afterwards `cerulion connect <robot>` is admitted with no flags.
//!
//! ## Why CPace (the real PAKE), not `claim`
//!
//! The robot admits three bootstrap verbs (`claim`, `pair`, `code-pair-*`).
//! `pair` needs an offline cert chain (the cloud/Studio strong path); `claim`
//! needs the chassis secret (first-desk physical possession). `cerulion pair`
//! drives the **code-pair** path — a short human-confirmable code — because that
//! is the balanced-PAKE ceremony a non-technical operator with a never-seen robot
//! actually runs (the owner starts code pairing on the robot; the operator types
//! the code). A wrong code produces mismatched CPace keys, so it is detected
//! STRUCTURALLY on the desk side (`PairingError::ConfirmationFailed`) before the
//! `code-pair-finish` call, and mapped to a distinct exit code.
//!
//! ## STDOUT state-line contract (machine-parseable — Studio subprocess driving)
//!
//! - `pairing: robot=<display> eid=<hex>` — at ceremony start.
//! - `paired: robot=<display> eid=<hex> account=<hex>` — on success (exit 0).
//!
//! Everything else (progress, errors, the interactive code prompt) goes to
//! STDERR. The exit code is the primary signal (see [`PairError::exit_code`]).
//!
//! ## Peer text
//!
//! Every `message` / `reason` a robot stamps on a rejected pairing verb — and the WHOLE
//! JSON body of a non-confirming reply — is peer-chosen, and `cerulion pair` renders it
//! to the operator at ERROR. Pairing is the moment the desk trusts the robot LEAST, so
//! the crate's boundary rule applies here exactly as in [`crate::worker`]: peer bytes
//! are neutered + BOUNDED **where they enter a [`PairError`]** — `classify_call_error`,
//! `classify_transport_error`, `parse_finish_reply`, `decode_hex_field`,
//! `classify_finish_error` — never only at the render. Classification still reads the
//! RAW text (see [`classify_verb_error`]), so sanitizing can never move an exit code.
//! Desk-authored prefixes/remediation stay OUTSIDE the bound.

use std::net::SocketAddr;
use std::time::Duration;

use cerud::client::OpsClient;
use cerud::error::CerudError;
use cerulion_link::{
    alpn, build_endpoint, dial, direct_addr, open_frame_stream, EndpointAddr, EndpointConfig,
    EndpointId, QuicOpsStream, RelayConfig,
};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::format::{AccountId, PublicKey};
use cerulion_pairing::pake::{CpaceInitiator, PakeIdentities};
use cerulion_pairing::PairingError;
use cerulion_wireclient::epoch::sanitize_peer_text;

use crate::worker::bounded_peer_display;

// ── exit-code contract ────────────────────────────────────────────────────────
//
// The `cerulion pair` verb forwards these codes verbatim (like `cerulion
// connect`), so a Studio `RemoteConnectManager`-style driver branches on them.
// Documented in BOTH the `cerulion pair` long help and `cerulion-connectd pair`.

/// The ceremony succeeded — the robot wrote a durable access-list row for this
/// desk's device key.
pub const EXIT_PAIRED: u8 = 0;
/// Usage / configuration error (bad key file, malformed args, empty code). The
/// `cerulion` CLI decides most of these before spawning; this is the residual.
pub const EXIT_USAGE: u8 = 1;
/// The robot REFUSED the ceremony — no code is armed (the owner must start code
/// pairing on the robot first), the session was denied, or the code window
/// expired. NOT a wrong code (that is [`EXIT_CODE_MISMATCH`]).
pub const EXIT_REFUSED: u8 = 2;
/// The robot could not be reached — the dial failed, the stream dropped, or the
/// ceremony timed out. Retry (check the robot is on, reachable, running remoted).
pub const EXIT_UNREACHABLE: u8 = 3;
/// The pairing CODE was wrong (the CPace confirmation did not match) or its
/// bounded attempts were exhausted. Re-check the code with the robot owner.
pub const EXIT_CODE_MISMATCH: u8 = 4;
/// The ceremony was INTERRUPTED (Ctrl-C) before completing — NOT paired. Follows
/// the Unix `128 + SIGINT(2)` convention, aligning with
/// `connect_cmd::exit_code_of`'s signal mapping (a caught Ctrl-C exits 130
/// deliberately; an uncaught one would surface as 130 via the signal too).
pub const EXIT_INTERRUPTED: u8 = 130;

// ── error taxonomy ────────────────────────────────────────────────────────────

/// The pairing-ceremony error, carrying its distinct exit class + a human reason
/// (surfaced on STDERR; the exit code is the machine signal).
#[derive(Debug)]
pub enum PairError {
    /// Bad config / usage — maps to [`EXIT_USAGE`].
    Usage(String),
    /// Couldn't reach the robot / transport failure / timeout — [`EXIT_UNREACHABLE`].
    Unreachable(String),
    /// The robot refused the ceremony (no armed code / denied / expired) —
    /// [`EXIT_REFUSED`].
    Refused(String),
    /// The pairing code was wrong / attempts exhausted — [`EXIT_CODE_MISMATCH`].
    CodeMismatch(String),
}

impl PairError {
    /// The stable exit code for this failure class.
    pub fn exit_code(&self) -> u8 {
        match self {
            PairError::Usage(_) => EXIT_USAGE,
            PairError::Unreachable(_) => EXIT_UNREACHABLE,
            PairError::Refused(_) => EXIT_REFUSED,
            PairError::CodeMismatch(_) => EXIT_CODE_MISMATCH,
        }
    }
}

impl std::fmt::Display for PairError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PairError::Usage(m) => write!(f, "pairing usage error: {m}"),
            PairError::Unreachable(m) => write!(f, "could not reach the robot: {m}"),
            PairError::Refused(m) => write!(f, "the robot refused pairing: {m}"),
            PairError::CodeMismatch(m) => write!(f, "wrong pairing code: {m}"),
        }
    }
}

impl std::error::Error for PairError {}

// ── the resolved ceremony parameters ─────────────────────────────────────────

/// The fully-resolved parameters for one `cerulion pair` ceremony.
#[derive(Clone)]
pub struct PairConfig {
    /// The robot's iroh endpoint id (== its device key's public half + the CPace
    /// responder key).
    pub robot_eid: EndpointId,
    /// Optional direct socket addresses (LAN direct-dial); empty ⇒ relay/discovery.
    pub direct_addrs: Vec<SocketAddr>,
    /// The desk's 32-byte ed25519 device seed (its iroh identity + CPace
    /// initiator key). Its public half is the key the robot access-lists.
    pub desk_seed: [u8; 32],
    /// The iroh relay configuration.
    pub relay: RelayConfig,
    /// The short pairing code the robot owner armed.
    pub code: String,
    /// The account this pairing is FOR. `None` ⇒ derive a self-account from the
    /// desk device key (the keypair-only desk with no cloud/Studio account).
    pub account: Option<[u8; 32]>,
    /// The human label the robot stores on the access-list row (e.g. the desk
    /// hostname) so the owner recognizes this desk.
    pub label: String,
    /// The display name for the STDOUT state lines (the name the user typed, or
    /// the eid hex when they passed a raw eid).
    pub robot_display: String,
    /// Bound on the dial + the whole ceremony (the desk-side deadline; the robot
    /// also enforces its own ops-session deadline).
    pub timeout: Duration,
}

impl std::fmt::Debug for PairConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let public =
            cerulion_pairing::client::DeviceIdentity::from_seed(&self.desk_seed).public_key();
        f.write_str("PairConfig { desk_seed: [REDACTED], code: [REDACTED], desk_eid: \"")?;
        for byte in public.0 {
            write!(f, "{byte:02x}")?;
        }
        f.write_str("\" }")
    }
}

/// The successful ceremony outcome.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PairOutcome {
    /// The account the robot recorded (hex), echoed from `code-pair-finish`.
    pub account_hex: String,
    /// The pairing source the robot reported (e.g. `code_paired`).
    pub source: String,
    /// The desk device key's public half (hex) — the key the robot now trusts.
    pub desk_eid_hex: String,
}

// ── the ceremony ─────────────────────────────────────────────────────────────

/// Run the desk-side CPace pairing ceremony against the robot's ops plane.
///
/// Binds the desk endpoint on the `cerulion/ops/1` ALPN, dials the robot, and —
/// on a blocking task (the sync `OpsClient` must not run on an async worker) —
/// drives `code-pair-start` → `code-pair-finish`. Every step is bounded by
/// `config.timeout`; on a desk-side timeout the connection is dropped to unblock
/// the parked read.
// Logging-rule exception (Principle 12): the two `println!`s below are not logging — they are this
// process's OUTPUT PROTOCOL. `pairing:` and `paired:` are machine-parseable
// state lines on STDOUT that a Studio driver reads; routing them through
// `tracing` would put them on stderr, behind RUST_LOG, and break every parser.
// The human-readable narration of the same steps DOES go through `tracing`.
#[allow(clippy::print_stdout)]
pub async fn run_pair(config: PairConfig) -> Result<PairOutcome, PairError> {
    // The desk identity: its public half is the CPace initiator key AND the key
    // the robot access-lists. The self-account default derives from it.
    let desk_identity = DeviceIdentity::from_seed(&config.desk_seed);
    let desk_pubkey = desk_identity.public_key();
    let desk_eid_hex = hex::encode(desk_pubkey.0);
    let account = AccountId(config.account.unwrap_or(desk_pubkey.0));

    // The `pairing:` state line (STDOUT, machine-parseable). The desk-side eid IS
    // the robot's eid (the dial target) — display it so a driver can correlate.
    println!(
        "{}",
        pairing_line(
            &config.robot_display,
            &hex::encode(config.robot_eid.as_bytes())
        )
    );

    // 1. Bind the desk endpoint on the OPS ALPN (the desk never accepts).
    let endpoint = build_endpoint(
        EndpointConfig::new(config.desk_seed)
            .with_alpns(vec![alpn::OPS.to_vec()])
            .with_relay(config.relay.clone()),
    )
    .await
    .map_err(|e| PairError::Unreachable(format!("binding the desk endpoint failed: {e}")))?;
    tracing::info!(
        desk_id = %endpoint.id(),
        robot = %config.robot_eid,
        direct_addrs = config.direct_addrs.len(),
        "cerulion pair: desk endpoint bound; dialing robot ops ALPN"
    );

    // 2. The peer address (LAN direct-dial when addrs are given, else bare eid).
    let peer: EndpointAddr = if config.direct_addrs.is_empty() {
        EndpointAddr::new(config.robot_eid)
    } else {
        direct_addr(config.robot_eid, config.direct_addrs.iter().copied())
    };

    // 3. Dial the ops plane + open the bidi stream, bounded.
    let connection = bounded(
        config.timeout,
        "dialing the robot",
        dial(&endpoint, peer, alpn::OPS),
    )
    .await?
    .map_err(|e| PairError::Unreachable(format!("dial failed: {e}")))?;
    let (send, recv) = bounded(
        config.timeout,
        "opening the ops stream",
        open_frame_stream(&connection),
    )
    .await?
    .map_err(|e| PairError::Unreachable(format!("opening the ops stream failed: {e}")))?;

    // 4. Run the SYNC ceremony on a blocking task (the `OpsClient` + `QuicOpsStream`
    //    panic if their Read/Write runs on an async worker thread). Bound it: on a
    //    desk-side timeout, DROP the connection to unblock the parked read, then
    //    reap the task.
    let robot_pubkey = PublicKey(*config.robot_eid.as_bytes());
    let code = config.code.clone();
    let label = config.label.clone();
    let ceremony = tokio::task::spawn_blocking(move || {
        run_ceremony(
            send,
            recv,
            &code,
            desk_pubkey,
            robot_pubkey,
            account,
            &label,
        )
    });
    tokio::pin!(ceremony);

    let result = tokio::select! {
        joined = &mut ceremony => joined
            .map_err(|e| PairError::Unreachable(format!("the ceremony task failed: {e}")))?,
        _ = tokio::time::sleep(config.timeout) => {
            // Unblock the parked blocking read by closing the connection, then reap.
            drop(connection);
            let _ = (&mut ceremony).await;
            return Err(PairError::Unreachable(format!(
                "the pairing ceremony timed out after {:?}",
                config.timeout
            )));
        }
    };
    // Keep the connection alive until the ceremony finished.
    drop(connection);

    let (account_hex, source) = result?;
    // The `paired:` state line (STDOUT, machine-parseable — the terminal success
    // signal a Studio driver parses, redundant with exit 0). The account echoed is
    // the robot's, which equals the one we sent.
    println!(
        "{}",
        paired_line(
            &config.robot_display,
            &hex::encode(config.robot_eid.as_bytes()),
            &account_hex
        )
    );
    tracing::info!(
        robot = %config.robot_display,
        source = %source,
        "cerulion pair: paired successfully; the robot now trusts this desk"
    );
    Ok(PairOutcome {
        account_hex,
        source,
        desk_eid_hex,
    })
}

/// Await `fut` with a desk-side timeout, mapping an elapsed timeout to a
/// [`PairError::Unreachable`] naming the phase.
async fn bounded<F: std::future::Future>(
    timeout: Duration,
    phase: &str,
    fut: F,
) -> Result<F::Output, PairError> {
    tokio::time::timeout(timeout, fut)
        .await
        .map_err(|_| PairError::Unreachable(format!("timed out {phase} after {timeout:?}")))
}

/// The synchronous CPace ceremony over one ops connection (runs on a blocking
/// task). Returns `(account_hex, source)` on success.
fn run_ceremony(
    send: cerulion_link::SendStream,
    recv: cerulion_link::RecvStream,
    code: &str,
    desk_pubkey: PublicKey,
    robot_pubkey: PublicKey,
    account: AccountId,
    label: &str,
) -> Result<(String, String), PairError> {
    let stream = QuicOpsStream::new(send, recv);
    let mut client =
        OpsClient::connect_current(stream).map_err(classify_transport_error("ops handshake"))?;

    // The CPace initiator. Empty channel-binding context — the robot's
    // `code-pair-start` responder binds the SAME empty context, so they agree.
    let initiator = CpaceInitiator::new(
        code.to_string(),
        PakeIdentities::new(desk_pubkey, robot_pubkey, Vec::new()),
    );
    let (attempt, msg1) = initiator
        .begin()
        .map_err(|e| PairError::Usage(format!("CPace begin failed: {e}")))?;

    // Step 1: send msg1, receive msg2 + the responder confirmation tag.
    let start = client
        .call(
            "code-pair-start",
            serde_json::json!({ "msg1": hex::encode(msg1) }),
        )
        .map_err(classify_call_error)?;
    let msg2 = decode_hex_field(&start, "msg2")?;
    let responder_confirm = decode_hex_field(&start, "responder_confirm")?;

    // Desk-side finish: a WRONG code yields mismatched CPace keys, so the
    // responder confirmation tag mismatches HERE — the STRUCTURAL code-mismatch
    // signal, detected before we ever call `code-pair-finish`.
    let (_keys, initiator_confirm) = attempt
        .finish(&msg2, &responder_confirm)
        .map_err(classify_finish_error)?;

    // Step 2: send the initiator confirmation tag + the account/label to persist.
    let finish = client
        .call(
            "code-pair-finish",
            serde_json::json!({
                "initiator_confirm": hex::encode(initiator_confirm),
                "account": hex::encode(account.0),
                "name": label,
            }),
        )
        .map_err(classify_call_error)?;

    parse_finish_reply(&finish)
}

/// Parse the `code-pair-finish` reply into `(account_hex, source)`. Requires a
/// confirmed `paired: true` AND a NON-EMPTY `account` string — the `paired:`
/// STDOUT line's machine contract (`account=` must never be blank for a Studio
/// parser). A missing / empty `account`, or `paired != true`, is a
/// [`PairError::Refused`] (mirrors [`decode_hex_field`]'s treatment of a garbage
/// reply) — so the caller never pins / prints success on that path. `source`
/// defaults to `code_paired` when the robot omits it (informational only). Pure —
/// oracle-tested.
fn parse_finish_reply(reply: &serde_json::Value) -> Result<(String, String), PairError> {
    let paired = reply
        .get("paired")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(false);
    if !paired {
        // The WHOLE reply is peer-chosen. `serde_json::Value`'s `Display` escapes only
        // `< 0x20` — U+007F and the C1 block (incl. the 8-bit CSI U+009B) pass through
        // RAW — and it applies no length bound at all, over a frame admitted up to
        // `MAX_FRAME_BYTES`. Render it BOUNDED + neutered.
        return Err(PairError::Refused(format!(
            "the robot did not confirm the pairing (reply: {})",
            bounded_peer_display(reply)
        )));
    }
    let account_hex = reply
        .get("account")
        .and_then(serde_json::Value::as_str)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| {
            PairError::Refused(
                "the robot reported paired but its reply carries no non-empty 'account' — the \
                 machine contract requires one; ensure the robot runs a compatible \
                 cerulion_remoted"
                    .to_string(),
            )
        })?
        .to_string();
    let source = reply
        .get("source")
        .and_then(serde_json::Value::as_str)
        .unwrap_or("code_paired")
        .to_string();
    Ok((account_hex, source))
}

/// Decode a required hex string field from an ops reply into raw bytes.
fn decode_hex_field(reply: &serde_json::Value, field: &str) -> Result<Vec<u8>, PairError> {
    let s = reply
        .get(field)
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| {
            PairError::Refused(format!("the robot's reply is missing the '{field}' field"))
        })?;
    hex::decode(s).map_err(|e| {
        // DEFENSE IN DEPTH, not a leak plug (a raw ESC would not reach the
        // terminal without it either).
        // `FromHexError::InvalidHexCharacter { c, .. }` echoes a PEER-CHOSEN char but
        // renders it with `{:?}` (char `Debug` = `escape_debug`), so an ESC already
        // arrives as the printable text `'\u{1b}'`; the message is also ~35 chars, so
        // the bound is inert here too. `bounded_peer_display` is applied anyway so that
        // EVERY peer-text interpolation in this module goes through ONE bound-and-neuter
        // seam and a future `hex` / `Display` change cannot quietly open a hole.
        PairError::Refused(format!(
            "the robot's '{field}' field is not valid hex: {}",
            bounded_peer_display(&e)
        ))
    })
}

// ── error classification (pure, oracle-tested) ────────────────────────────────

/// Which pairing exit class a robot verb error maps to. Pure discriminator over
/// the robot's `(kind, message)` — the machine-readable class it stamps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerbErrorClass {
    /// The code was wrong or its attempts were exhausted (exit 4).
    CodeMismatch,
    /// The robot refused for another reason (no armed code / denied / expired) —
    /// exit 2.
    Refused,
}

/// Classify a robot `Response::Err { kind, message }` into a pairing exit class.
///
/// A code-pairing rejection rides `kind == "verb_error"` with a message the
/// `cerulion_remoted` pairing verbs stamp (`code pairing rejected: <PairingError>`).
/// The CPace `PairingError`s that mean "the code did not work" are
/// `ConfirmationFailed` (rendered "confirmation failed") and `AttemptsExhausted`
/// (rendered "N attempts exhausted" + "burned") — those are CODE MISMATCH. Every
/// other verb error (`denied`, "no code pairing is armed", an expired TTL, an
/// unknown verb) is a REFUSAL. The wrong-code path is ALSO caught structurally
/// desk-side (`ConfirmationFailed` at `attempt.finish`), so this is the fallback
/// for a robot-side rejection.
pub fn classify_verb_error(kind: &str, message: &str) -> VerbErrorClass {
    if kind == "verb_error" {
        let m = message.to_ascii_lowercase();
        if m.contains("confirmation failed")
            || m.contains("wrong code")
            || m.contains("attempts exhausted")
            || m.contains("burned")
        {
            return VerbErrorClass::CodeMismatch;
        }
    }
    VerbErrorClass::Refused
}

/// Map a `cerud` verb-call error to a [`PairError`]. A `Remote { kind, message }`
/// is classified by [`classify_verb_error`]; a transport error
/// (`Io`/`ConnectionClosed`/`Decode`) is [`PairError::Unreachable`].
///
/// Peer text is NEUTERED + BOUNDED here, at the classifier boundary — the SAME rule
/// [`crate::worker`]'s `classify_catalog_reply` / `classify_demand_reply` follow.
/// CLASSIFICATION reads the RAW `message` ([`classify_verb_error`] matches on the
/// robot's wording); only the text CARRIED into the error is sanitized, so the
/// sanitizer can never change which exit code a robot's answer maps to.
fn classify_call_error(err: CerudError) -> PairError {
    match err {
        CerudError::Remote { kind, message } => match classify_verb_error(&kind, &message) {
            VerbErrorClass::CodeMismatch => PairError::CodeMismatch(sanitize_peer_text(&message)),
            VerbErrorClass::Refused => PairError::Refused(sanitize_peer_text(&message)),
        },
        // A per-verb authz denial (should not happen for the self-gating
        // bootstrap verbs, but map it faithfully).
        CerudError::Denied { reason, .. } => PairError::Refused(sanitize_peer_text(&reason)),
        // The handshake / session was rejected mid-call.
        CerudError::HandshakeRejected { reason, .. } => {
            PairError::Refused(sanitize_peer_text(&reason))
        }
        other => classify_transport_error("running the pairing verb")(other),
    }
}

/// Build a mapper from a `cerud` error at `phase` to a [`PairError`]. A rejected
/// handshake is a REFUSAL (the robot closed the session — e.g. a poisoned receipt
/// sink); everything else at the transport layer is UNREACHABLE.
///
/// Peer text is sanitized at this boundary, exactly as in [`classify_call_error`]. The
/// catch-all arm renders the underlying error through [`bounded_peer_display`] (the
/// desk-authored `{phase} failed:` prefix stays OUTSIDE the bound, so a hostile robot
/// cannot consume the operator's own context).
fn classify_transport_error(phase: &'static str) -> impl Fn(CerudError) -> PairError {
    move |err: CerudError| match err {
        CerudError::HandshakeRejected { reason, .. } => {
            PairError::Refused(sanitize_peer_text(&reason))
        }
        CerudError::Denied { reason, .. } => PairError::Refused(sanitize_peer_text(&reason)),
        CerudError::Remote { kind, message } => match classify_verb_error(&kind, &message) {
            VerbErrorClass::CodeMismatch => PairError::CodeMismatch(sanitize_peer_text(&message)),
            VerbErrorClass::Refused => PairError::Refused(sanitize_peer_text(&message)),
        },
        other => {
            PairError::Unreachable(format!("{phase} failed: {}", bounded_peer_display(&other)))
        }
    }
}

/// Map a CPace initiator `finish` error to a [`PairError`]. A `ConfirmationFailed`
/// is a WRONG CODE ([`PairError::CodeMismatch`], exit 4). Any OTHER finish error
/// means the ROBOT sent a malformed CPace message (a bad step-2 packet length /
/// an off-curve point), i.e. a robot-side problem — so [`PairError::Refused`]
/// (exit 2), CONSISTENT with [`decode_hex_field`]'s handling of a missing /
/// non-hex `msg2` (a garbage reply is never the desk operator's usage error).
fn classify_finish_error(e: PairingError) -> PairError {
    match e {
        PairingError::ConfirmationFailed => PairError::CodeMismatch(
            "the pairing code did not match — re-check it with the robot owner".to_string(),
        ),
        // `PairingError::Serialization(String)` can carry text derived from the robot's
        // own CPace bytes, so bound + neuter it; the desk's remediation sentence stays
        // OUTSIDE the bound so a hostile robot cannot clip it away.
        other => PairError::Refused(format!(
            "the robot sent a malformed CPace reply ({}); make sure it is running a \
             compatible cerulion_remoted",
            bounded_peer_display(&other)
        )),
    }
}

/// The `pairing:` STDOUT state line (the Studio machine contract) — exact format
/// pinned by an oracle test so a driver can parse it byte-stably.
fn pairing_line(robot_display: &str, eid_hex: &str) -> String {
    format!("pairing: robot={robot_display} eid={eid_hex}")
}

/// The `paired:` STDOUT success line (the Studio machine contract) — exact format
/// pinned by an oracle test.
fn paired_line(robot_display: &str, eid_hex: &str, account_hex: &str) -> String {
    format!("paired: robot={robot_display} eid={eid_hex} account={account_hex}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pair_config_debug_redacts_seed_and_pairing_code() {
        // RFC 8032, section 7.1, test vector 1: independent public-key oracle.
        let seed = [
            0x9d, 0x61, 0xb1, 0x9d, 0xef, 0xfd, 0x5a, 0x60, 0xba, 0x84, 0x4a, 0xf4, 0x92, 0xec,
            0x2c, 0xc4, 0x44, 0x49, 0xc5, 0x69, 0x7b, 0x32, 0x69, 0x19, 0x70, 0x3b, 0xac, 0x03,
            0x1c, 0xae, 0x7f, 0x60,
        ];
        let public_hex = "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a";
        let secret_hex = "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60";
        let public_bytes: [u8; 32] = hex::decode(public_hex).unwrap().try_into().unwrap();
        let code = "731-246-private-pairing-code";
        let config = PairConfig {
            robot_eid: EndpointId::from_bytes(&public_bytes).unwrap(),
            direct_addrs: Vec::new(),
            desk_seed: seed,
            relay: RelayConfig::Disabled,
            code: code.into(),
            account: None,
            label: "test desk".into(),
            robot_display: "test robot".into(),
            timeout: Duration::from_secs(1),
        };
        let compact = format!("{config:?}");
        let pretty = format!("{config:#?}");
        assert_eq!(
            compact,
            format!("PairConfig {{ desk_seed: [REDACTED], code: [REDACTED], desk_eid: \"{public_hex}\" }}")
        );
        for output in [compact, pretty] {
            assert!(output.contains("[REDACTED]"));
            assert!(output.contains(public_hex));
            assert!(!output.contains(secret_hex));
            assert!(!output.contains(&format!("{seed:?}")));
            assert!(!output.contains(&format!("{seed:#?}")));
            assert!(!output.contains(code));
        }
    }

    #[test]
    fn exit_codes_are_the_documented_contract() {
        assert_eq!(EXIT_PAIRED, 0);
        assert_eq!(EXIT_USAGE, 1);
        assert_eq!(EXIT_REFUSED, 2);
        assert_eq!(EXIT_UNREACHABLE, 3);
        assert_eq!(EXIT_CODE_MISMATCH, 4);
        assert_eq!(PairError::Usage("x".into()).exit_code(), EXIT_USAGE);
        assert_eq!(
            PairError::Unreachable("x".into()).exit_code(),
            EXIT_UNREACHABLE
        );
        assert_eq!(PairError::Refused("x".into()).exit_code(), EXIT_REFUSED);
        assert_eq!(
            PairError::CodeMismatch("x".into()).exit_code(),
            EXIT_CODE_MISMATCH
        );
        // The interrupt code is the Unix Ctrl-C convention (128 + SIGINT).
        assert_eq!(EXIT_INTERRUPTED, 130);
    }

    #[test]
    fn finish_error_confirmation_is_code_mismatch_else_refused() {
        // A wrong code (the CPace key-confirmation mismatch) → CodeMismatch (4).
        assert!(matches!(
            classify_finish_error(PairingError::ConfirmationFailed),
            PairError::CodeMismatch(_)
        ));
        // A malformed CPace reply from the robot (bad step-2 length / off-curve
        // point) → Refused (2), NOT a Usage(1) "your command was wrong" — the
        // robot sent garbage, matching decode_hex_field's Refused mapping.
        assert!(matches!(
            classify_finish_error(PairingError::PakeProtocol("bad step2 packet length")),
            PairError::Refused(_)
        ));
        assert_eq!(
            classify_finish_error(PairingError::PakeProtocol("x")).exit_code(),
            EXIT_REFUSED,
            "a malformed robot reply is REFUSED (2), never USAGE (1)"
        );
    }

    #[test]
    fn parse_finish_reply_requires_paired_true_and_nonempty_account() {
        // Happy: paired + account + source → (account, source).
        assert_eq!(
            parse_finish_reply(&serde_json::json!({
                "paired": true, "account": "8a2e", "source": "code_paired"
            }))
            .unwrap(),
            ("8a2e".to_string(), "code_paired".to_string())
        );
        // source defaults to code_paired when the robot omits it.
        assert_eq!(
            parse_finish_reply(&serde_json::json!({ "paired": true, "account": "8a2e" }))
                .unwrap()
                .1,
            "code_paired"
        );
        // paired:true but MISSING account → Refused (never a blank `account=` line).
        let e = parse_finish_reply(&serde_json::json!({ "paired": true })).unwrap_err();
        assert!(matches!(e, PairError::Refused(_)));
        assert_eq!(
            e.exit_code(),
            EXIT_REFUSED,
            "missing account → Refused (2), no success pin"
        );
        // paired:true but EMPTY account → Refused.
        assert!(matches!(
            parse_finish_reply(&serde_json::json!({ "paired": true, "account": "" })),
            Err(PairError::Refused(_))
        ));
        // account present but paired MISSING/false → Refused (defaults to not-paired).
        assert!(matches!(
            parse_finish_reply(&serde_json::json!({ "account": "8a2e" })),
            Err(PairError::Refused(_))
        ));
        assert!(matches!(
            parse_finish_reply(&serde_json::json!({ "paired": false, "account": "8a2e" })),
            Err(PairError::Refused(_))
        ));
    }

    #[test]
    fn stdout_state_lines_have_exact_format() {
        // Hand oracles (literal expected strings) — the Studio machine contract.
        assert_eq!(
            pairing_line("go2", "3b1fdead"),
            "pairing: robot=go2 eid=3b1fdead"
        );
        assert_eq!(
            paired_line("go2", "3b1fdead", "8a2ebeef"),
            "paired: robot=go2 eid=3b1fdead account=8a2ebeef"
        );
    }

    #[test]
    fn verb_error_classifies_wrong_code_and_burn_as_code_mismatch() {
        // The exact strings the `cerulion_remoted` pairing verbs stamp.
        assert_eq!(
            classify_verb_error("verb_error", "code pairing rejected: confirmation failed"),
            VerbErrorClass::CodeMismatch
        );
        assert_eq!(
            classify_verb_error(
                "verb_error",
                "code pairing rejected: 2 attempts exhausted (burned)"
            ),
            VerbErrorClass::CodeMismatch
        );
        // Case-insensitive.
        assert_eq!(
            classify_verb_error("verb_error", "CONFIRMATION FAILED"),
            VerbErrorClass::CodeMismatch
        );
    }

    #[test]
    fn verb_error_classifies_no_armed_code_denied_and_expiry_as_refused() {
        assert_eq!(
            classify_verb_error(
                "verb_error",
                "no code pairing is armed on this robot; an owner must arm a guest code first (begin code pairing)"
            ),
            VerbErrorClass::Refused
        );
        assert_eq!(
            classify_verb_error("verb_error", "code pairing rejected: pake expired"),
            VerbErrorClass::Refused
        );
        assert_eq!(
            classify_verb_error("denied", "unpaired"),
            VerbErrorClass::Refused
        );
        // A non-verb_error kind is never a code mismatch even if the message looks
        // code-ish (kind is authoritative for the code-pair rejection channel).
        assert_eq!(
            classify_verb_error("denied", "confirmation failed"),
            VerbErrorClass::Refused
        );
    }

    #[test]
    fn call_error_maps_remote_kinds_to_pair_error_classes() {
        assert!(matches!(
            classify_call_error(CerudError::Remote {
                kind: "verb_error".into(),
                message: "code pairing rejected: confirmation failed".into(),
            }),
            PairError::CodeMismatch(_)
        ));
        assert!(matches!(
            classify_call_error(CerudError::Remote {
                kind: "verb_error".into(),
                message: "no code pairing is armed on this robot".into(),
            }),
            PairError::Refused(_)
        ));
        assert!(matches!(
            classify_call_error(CerudError::Denied {
                caller: "abcd".into(),
                verb: "code-pair-start".into(),
                reason: "unpaired".into(),
            }),
            PairError::Refused(_)
        ));
        // A transport error is UNREACHABLE, not refused.
        assert!(matches!(
            classify_call_error(CerudError::ConnectionClosed),
            PairError::Unreachable(_)
        ));
    }

    /// The hostile shape every peer-text pin in this crate uses: a CSI screen-clear, a
    /// CR overwrite, a LF, and a BEL. Hand-written, NOT derived from the sanitizer.
    const HOSTILE: &str = "x\u{1b}[2Jy\rz\nw\u{7}v";

    /// `cerulion pair` renders ROBOT-CHOSEN text at ERROR — at the
    /// moment the desk trusts the robot LEAST — so every [`PairError`] construction site
    /// that carries peer bytes NEUTERS + BOUNDS them, exactly as `worker`'s classifiers
    /// do. An earlier sweep covered `connect`; this arm was the remaining leak.
    ///
    /// HAND ORACLES throughout (the expected neutered strings are written out, never
    /// computed by calling the sanitizer). Every arm asserts on the FULL `Display` — the
    /// operator-visible string — so a site that sanitized the wrong half still fails.
    #[test]
    fn pair_error_construction_sites_neuter_and_bound_robot_text() {
        // (a) A verb error classified as a WRONG CODE — the robot picks the message.
        let e = classify_call_error(CerudError::Remote {
            kind: "verb_error".into(),
            // Contains "confirmation failed" so it classifies as CodeMismatch even
            // though the RAW text also carries a CSI escape: classification reads the
            // raw message, the ERROR carries the neutered one.
            message: format!("confirmation failed {HOSTILE}"),
        });
        assert!(matches!(e, PairError::CodeMismatch(_)));
        assert_eq!(
            e.to_string(),
            "wrong pairing code: confirmation failed x\u{fffd}[2Jy\u{fffd}z\u{fffd}w\u{fffd}v",
            "the classification still reads the raw text; the RENDER is neutered"
        );

        // (b) A refusal reason (`Denied`) — a distinct field, a distinct site.
        let e = classify_call_error(CerudError::Denied {
            caller: "abcd".into(),
            verb: "code-pair-start".into(),
            reason: HOSTILE.into(),
        });
        assert_eq!(
            e.to_string(),
            "the robot refused pairing: x\u{fffd}[2Jy\u{fffd}z\u{fffd}w\u{fffd}v"
        );

        // (c) A handshake rejection through the TRANSPORT classifier (the other mapper).
        let e = classify_transport_error("dialing the robot")(CerudError::HandshakeRejected {
            reason: HOSTILE.into(),
            server_min: 1,
            server_max: 1,
        });
        assert_eq!(
            e.to_string(),
            "the robot refused pairing: x\u{fffd}[2Jy\u{fffd}z\u{fffd}w\u{fffd}v"
        );

        // (d) The transport classifier's CATCH-ALL arm: the desk-authored phase prefix
        //     survives OUTSIDE the bound, the peer's own text is neutered inside it.
        let e = classify_transport_error("opening the ops stream")(CerudError::Decode(
            HOSTILE.to_string(),
        ));
        assert_eq!(
            e.to_string(),
            "could not reach the robot: opening the ops stream failed: protocol decode error: \
             x\u{fffd}[2Jy\u{fffd}z\u{fffd}w\u{fffd}v"
        );

        // (e) The WHOLE-JSON interpolation — `serde_json::Value`'s Display passes U+009B
        //     (the 8-bit CSI) through RAW, so the sanitizer is the only thing stopping it.
        let e = parse_finish_reply(&serde_json::json!({
            "paired": false,
            "why": "\u{9b}2J",
        }))
        .unwrap_err();
        let shown = e.to_string();
        assert!(
            !shown.chars().any(|c| c.is_control()),
            "no raw control character survives the JSON render: {shown:?}"
        );
        assert!(
            shown.contains('\u{fffd}'),
            "the 8-bit CSI was neutered, not merely absent: {shown}"
        );

        // (f) The hex-decode echo. SCOPE: unlike (e), this
        //     site has NO sanitizer-observable signature — `FromHexError` renders the
        //     peer's offending char with `{:?}`, so a control char arrives ALREADY
        //     escaped and the sanitizer is pure defense in depth here. A `!is_control()`
        //     assert would therefore hold on the UNSANITIZED body too — a tautology that
        //     pins nothing. The real, non-vacuous
        //     property is the EXACT rendering: the peer's bad character stays VISIBLE
        //     (escaped, with its position) behind the desk's own sentence, so a site that
        //     dropped or mangled the diagnostic fails here.
        let e = decode_hex_field(&serde_json::json!({ "msg2": "\u{1b}f" }), "msg2").unwrap_err();
        assert_eq!(
            e.to_string(),
            "the robot refused pairing: the robot's 'msg2' field is not valid hex: \
             Invalid character '\\u{1b}' at position 0"
        );

        // The BOUND asserts below ((g) + (h)) are DERIVED from the site's components
        // instead of hand-counted to a couple of characters of margin: the peer half can
        // never exceed `sanitize_peer_text`'s documented 512-char ceiling plus its
        // `…(truncated)` marker (HAND constants — the sanitizer is never called), while
        // the desk-authored sentence around it lives OUTSIDE the bound BY DESIGN and is
        // free to be reworded. `desk_sentence_allowance` is the slack for that copy, so
        // adding a word to OUR wording cannot flip these into a false "bound regression"
        // report — yet a genuinely unbounded render (1_000_000 chars) still fails by
        // three orders of magnitude, and a raised sanitizer ceiling fails loudly too.
        // 96 keeps real rewording headroom (the largest desk sentence today is 74
        // chars) while holding the blind spot small: a sanitizer ceiling raised past
        // ~534 now fails these arms, instead of the ~694 a 256-char allowance let by.
        let peer_ceiling = 512 + "…(truncated)".chars().count();
        let desk_sentence_allowance = 96usize;
        let bound = peer_ceiling + desk_sentence_allowance;

        // (g) BOUND: a megabyte reason cannot flood the operator's log pipeline.
        let e = classify_call_error(CerudError::Denied {
            caller: "abcd".into(),
            verb: "code-pair-start".into(),
            reason: "z".repeat(1_000_000),
        });
        let shown = e.to_string();
        assert!(
            shown.chars().count() <= bound,
            "a megabyte refusal reason is bounded by the sanitizer ceiling ({peer_ceiling}) \
             plus the desk sentence (allowance {desk_sentence_allowance}), got {} chars",
            shown.chars().count()
        );
        assert!(shown.ends_with("…(truncated)"));

        // (h) BOUND through the whole-JSON site too (the unbounded `Value` Display).
        let e = parse_finish_reply(&serde_json::json!({
            "paired": false,
            "why": "z".repeat(1_000_000),
        }))
        .unwrap_err();
        let shown = e.to_string();
        assert!(
            shown.chars().count() <= bound,
            "a megabyte reply is bounded by the sanitizer ceiling ({peer_ceiling}) plus the \
             desk sentence (allowance {desk_sentence_allowance}), got {} chars",
            shown.chars().count()
        );
        // The marker is INTERIOR here — the desk's own `)` closes the sentence after
        // the bounded peer render, which is precisely the "desk text stays outside the
        // bound" property this site is written for.
        assert!(
            shown.contains("…(truncated)") && shown.ends_with(')'),
            "the truncation is explicit and the desk's own suffix survives: {shown}"
        );

        // (i) ANTI-TAUTOLOGY: ordinary text is carried through INTACT — a site that
        //     returned "" or dropped the peer's words would pass every assert above.
        let e = classify_call_error(CerudError::Remote {
            kind: "verb_error".into(),
            message: "no code pairing is armed on this robot".into(),
        });
        assert_eq!(
            e.to_string(),
            "the robot refused pairing: no code pairing is armed on this robot"
        );
    }
}
