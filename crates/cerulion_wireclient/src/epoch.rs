// SPDX-License-Identifier: AGPL-3.0-only
//! The SHARED desk-side revocation-epoch PUSH substrate — every decision
//! both desk paths make about pushing a cached epoch to a robot on connect.
//!
//! # Why this lives here (one implementation, two callers)
//!
//! Two desk paths dial the SAME `cerulion/wire/1` control plane and therefore both
//! carry the epoch (revocation propagation is NOT opt-in):
//!
//! - `cerulion_netd`'s iroh WAN plane, on every dial of a WAN robot;
//! - the `cerulion connect` verb (`cerulion_connectd`'s session driver), on its dial.
//!
//! A second implementation would be a second place for the classification, the size
//! guard, and the "never block the connection" posture to drift — so everything
//! except the transport call itself lives here: [`prepare_epoch_push`] decides WHAT
//! to send (and logs every reason not to), [`classify_epoch_reply`] decides what the
//! robot's answer MEANT (and logs it). Each caller supplies only its own bounded
//! control round-trip, because each owns a different timeout/error type.
//!
//! # The invariant every caller must preserve
//!
//! **Epoch freshness NEVER blocks the connection.** Every outcome in
//! [`EpochPushOutcome`] except [`EpochPushOutcome::TransportFailed`] is a POLICY
//! result: loud where an operator should care, recorded, and the dial proceeds.
//! `TransportFailed` is fatal ONLY because a control round-trip that fails mid-frame
//! leaves the (cancel-unsafe) framing desynced, so every later verb on that stream
//! would fail anyway — it is not a new failure class, and a re-dial recovers.

use std::path::{Path, PathBuf};

use cerulion_link::DEFAULT_MAX_FRAME_LEN;
use cerulion_pairing::verify::{
    EPOCH_VERSION_UNSUPPORTED_NEEDLE, NO_EPOCH_SINK_NEEDLE, UNDECODABLE_REQUEST_NEEDLE,
};

use crate::config::resolve_epoch_sync;
use crate::protocol::{decode_first_reply, WireRequest, WireResponse};

/// THE one keying rule: the desk's `<robot>.epoch` cache path inside an epochs
/// directory. A verbatim re-export of
/// [`cerulion_pairing::verify::epoch_cache_path_in`] — where the convention LIVES,
/// because the cache's WRITER (`cerulion_cli_engine::account_cmd`) must stay iroh-free
/// and therefore cannot link this crate. BOTH desk push paths (`cerulion connect` and
/// `cerulion-netd`'s WAN plane) resolve their cache path through THIS function, so a
/// key the writer produced and a key a reader looks for can never diverge.
pub use cerulion_pairing::verify::epoch_cache_path_in as epoch_cache_path;

/// THE one directory resolver + its env override, re-exported verbatim from
/// [`cerulion_pairing::verify`] (homed there so the iroh-free cache WRITER resolves the
/// SAME directory both readers do). BOTH desk paths call THESE — `cerulion connect`'s
/// binary and `cerulion-netd`'s `WanRegistry::from_env` — so a relocated cache is
/// visible to both or to neither, never to exactly one.
pub use cerulion_pairing::verify::{resolve_epoch_dir, EPOCH_DIR_ENV};

/// Resolve the desk's epoch-cache directory FROM THE ENVIRONMENT: [`EPOCH_DIR_ENV`] if
/// set (blank ignored), else the sibling `epochs/` next to `key_file`.
///
/// # Who resolves this directory, and how (three parties, one rule)
///
/// The invariant is ONE env name and ONE fallback rule everywhere, so an epoch cached
/// under a relocated directory is visible to every party or to none — never to exactly
/// one, silently (the failure mode when each path has its own env
/// var and its own resolver).
///
/// Only ONE of the three parties reaches it through this function:
///
/// | party | how it resolves |
/// |---|---|
/// | `cerulion-connectd connect` (reader) | calls THIS function, with the operator's `--key-file` as the anchor |
/// | `cerulion_cli_engine::account_cmd` (the cache WRITER) | reads [`EPOCH_DIR_ENV`] itself in `resolve_epoch_cache_path`, then calls the shared [`resolve_epoch_dir`] — with the WELL-KNOWN `~/.cerulion/desk.key` as the anchor, not a caller-supplied key file |
/// | `cerulion-netd`'s `WanRegistry` (reader) | reads [`EPOCH_DIR_ENV`] into its captured-inputs struct at `from_env`, then calls the shared [`resolve_epoch_dir`] on first use |
///
/// Each split is deliberate: the writer has no `--key-file` to anchor on (it is a desk
/// account verb, not a dial), and netd captures the env at boot — cheap — while deferring
/// the directory RULE to the first WAN dial. So there is one env NAME and one
/// resolver FUNCTION, but three env READS, and this convenience wrapper is `cerulion
/// connect`'s alone. What must never fork is [`resolve_epoch_dir`] and [`EPOCH_DIR_ENV`],
/// which all three share — that is what the three-party agreement pin
/// (`both_desk_paths_resolve_one_cache_path` + its netd and writer twins) anchors.
pub fn resolve_epoch_dir_from_env(key_file: Option<&Path>) -> Option<PathBuf> {
    resolve_epoch_dir(std::env::var(EPOCH_DIR_ENV).ok().as_deref(), key_file)
}

/// The ceiling on how much PEER-CONTROLLED text a desk log line may carry
/// ([`sanitize_peer_text`]). Long enough for a real diagnostic, short enough that a
/// hostile robot cannot dump megabytes into an operator's terminal or log pipeline.
const MAX_PEER_TEXT_LEN: usize = 512;

/// Neuter + BOUND a string the ROBOT chose before it reaches a desk log line.
///
/// Every `message`/`reason` in a `sync_epoch` answer is fully peer-controlled: the desk
/// is the one place it becomes operator-visible output. Control characters are replaced
/// with `U+FFFD` (so a hostile robot cannot inject terminal escapes — ESC-based CSI
/// screen-clears, CR overwrites, BEL) and the text is truncated at
/// `MAX_PEER_TEXT_LEN` (512) chars with an explicit `…(truncated)` marker, so a
/// megabyte-long "error" cannot flood the desk's logs. Same convention (and same
/// C0/DEL/C1 class) as `topic list`'s robot-name sanitizer. Pure — oracle-tested.
///
/// `pub` because the desk paths log peer text OUTSIDE this module too (the connect
/// session's identity-mismatch lines): one sanitizer, one policy, no second convention
/// that could forget the bound or the escape class.
pub fn sanitize_peer_text(s: &str) -> String {
    let mut out = String::with_capacity(s.len().min(MAX_PEER_TEXT_LEN));
    let mut truncated = false;
    for (i, c) in s.chars().enumerate() {
        if i >= MAX_PEER_TEXT_LEN {
            truncated = true;
            break;
        }
        out.push(match c {
            '\u{0000}'..='\u{001F}' | '\u{007F}' | '\u{0080}'..='\u{009F}' => '\u{FFFD}',
            other => other,
        });
    }
    if truncated {
        out.push_str("…(truncated)");
    }
    out
}

/// What happened to the revocation-epoch push on a dial.
///
/// Recorded per robot (netd) / per session (`cerulion connect`) so a non-delivery is
/// OBSERVABLE STATE, not merely a log line (Principle #3). Exactly ONE variant —
/// [`Applied`](EpochPushOutcome::Applied) — is a delivered-and-effective revocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpochPushOutcome {
    /// This desk has no epoch-cache directory at all (ephemeral key + no env) — nothing
    /// was pushed and nothing could be.
    NoCacheDir,
    /// This desk has no VERIFIED name for the robot it dialed, so it cannot select a
    /// cache file at all.
    ///
    /// The cache is keyed by the name the DESK knows a robot by (`cerulion pair` pins
    /// name→eid in `~/.cerulion/robots.toml`). A raw `--eid` dial of an unpinned robot —
    /// or an ambiguous reverse lookup — leaves the desk with no such name, and the one
    /// name available then is the robot's OWN reported identity, which would let the
    /// dialed peer choose which cached artifact it receives. Refusing to push is the
    /// safe outcome; the fix is to pin/pair the robot.
    UnverifiedRobotIdentity,
    /// The cache directory exists but holds no artifact for this robot: a never-synced
    /// desk, a guest (the account service's access endpoint is owner-only), or — the
    /// case worth investigating — a cache written under a DIFFERENT file name.
    NoCachedEpoch,
    /// An artifact exists but THIS BUILD cannot read it: it is corrupt, or it is stamped
    /// with an epoch-sync ENVELOPE VERSION this build does not understand (the writer is
    /// ahead of this desk). Either way this desk is NOT carrying revocations for this
    /// robot.
    ///
    /// The two shapes share the outcome (the delivery answer is identical) but keep
    /// DISTINCT log remediations — "re-sync it" vs "upgrade this desk", since re-syncing
    /// a version-skewed cache re-fetches the very shape that cannot be read. Same
    /// discipline as [`NoSink`](EpochPushOutcome::NoSink)'s three shapes.
    CacheUnreadable,
    /// The cached artifact is too large to send: the encoded control frame would
    /// exceed the peer's frame cap ([`DEFAULT_MAX_FRAME_LEN`]), which the robot's
    /// reader refuses AND which desyncs its control stream. Refusing to send it keeps
    /// a pathological cache a POLICY failure (this dial, and every later one, still
    /// works) instead of a permanent dial failure.
    CacheTooLarge {
        /// The size, in bytes, that proves the artifact cannot be sent: the encoded
        /// control frame's exact length, or — when the artifact was refused BEFORE
        /// being decoded, on its file size alone — a lower bound on it. Either way it
        /// exceeds [`DEFAULT_MAX_FRAME_LEN`].
        bytes: usize,
    },
    /// Delivered and the robot APPLIED it — the revocation is in force.
    Applied {
        /// The epoch the robot is now at.
        epoch: u64,
    },
    /// Delivered, but the robot was already at or ahead of it (the steady state).
    AlreadyCurrent {
        /// The robot's current epoch.
        epoch: u64,
    },
    /// The robot has NO usable epoch sink for this artifact — an UPGRADE-shaped
    /// outcome, in three shapes:
    ///
    /// 1. no sync sink is wired;
    /// 2. the robot PREDATES the `sync_epoch` verb entirely (it could not even decode
    ///    the request);
    /// 3. the robot knows the verb but refuses the artifact's ENVELOPE VERSION (the
    ///    two builds disagree on the epoch-sync wire shape).
    ///
    /// All three are distinct from [`Rejected`](EpochPushOutcome::Rejected): the fix is
    /// to upgrade/configure a BUILD, not to investigate the epoch. Collapsing any of
    /// them into `Rejected` sends an operator hunting a forgery that does not exist.
    NoSink,
    /// The robot REJECTED the epoch itself (forged, wrong robot, untrusted issuer, or a
    /// skewed robot clock). It is NOT current and the epoch warrants investigation.
    Rejected,
    /// The push could not complete (an undecodable reply, or an unexpected reply shape).
    NotDelivered,
    /// The control round-trip itself FAILED (timeout / stream error). The epoch was not
    /// delivered AND the control stream is desynced — the caller fails the dial and
    /// re-dials. Recorded so the observable never reads stale-successful after a
    /// failure (Principle #3).
    TransportFailed,
}

/// How LOUD a push outcome should be to an operator — the ONE classification both desk
/// paths' summary lines branch on.
///
/// Three states, not two: "the robot is current" and "this desk failed to carry a
/// revocation" do NOT partition the space. A desk that holds NOTHING for a robot — a
/// never-synced desk, a guest desk (the account service's access endpoint is
/// owner-only), any desk before its first sync — is neither. Rendering that healthy,
/// permanent steady state as a warning on EVERY connect and EVERY demand is a
/// false-alarm flood that trains operators to ignore the signal, which is exactly the
/// once-per-regime class the flood-suppression latches exist to prevent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EpochPushSeverity {
    /// The robot's access list is CURRENT with respect to this desk: it took what we
    /// carried, or it was already at/ahead of it. Log at `info`.
    Current,
    /// This desk holds NOTHING for this robot, so there is no revocation it failed to
    /// deliver — nothing is wrong and nothing is actionable. Log at `debug` (the shared
    /// substrate already names the exact path it searched, once, at `info`).
    NothingToCarry,
    /// This desk was — or may well have been — holding a revocation that did NOT reach
    /// the robot. The ONE class an operator must act on. Log at `warn`.
    NotCarried,
}

impl EpochPushOutcome {
    /// Whether the robot's access list is CURRENT with respect to this desk — i.e. the
    /// desk had nothing newer to give, or gave it and the robot took it.
    ///
    /// NOTE: `false` does NOT mean "failed to deliver" — a desk with no cached epoch is
    /// not current-by-delivery and has nothing to deliver either. Branch on
    /// [`Self::severity`] for the operator-facing loudness decision; this answers the
    /// narrower factual question ("is the robot's ACL known to match this desk's view?").
    pub fn robot_is_current(&self) -> bool {
        matches!(self.severity(), EpochPushSeverity::Current)
    }

    /// The operator-facing loudness class of this outcome — see [`EpochPushSeverity`].
    /// Total over every variant (no wildcard arm), so a NEW outcome cannot silently
    /// inherit someone else's log level.
    pub fn severity(&self) -> EpochPushSeverity {
        match self {
            // Delivered, or the robot already had it.
            Self::Applied { .. } | Self::AlreadyCurrent { .. } => EpochPushSeverity::Current,
            // This desk holds nothing for this robot: the normal state of a never-synced
            // or guest desk. NOT a failure — there is no revocation being withheld.
            Self::NoCacheDir | Self::NoCachedEpoch => EpochPushSeverity::NothingToCarry,
            // Everything else: this desk either HAD an artifact it could not deliver
            // (unreadable / oversized), tried and was refused (no sink / rejected /
            // unusable answer / transport), or could not even tell WHICH artifact was
            // meant (an unpinned robot — it may well have been holding one).
            Self::UnverifiedRobotIdentity
            | Self::CacheUnreadable
            | Self::CacheTooLarge { .. }
            | Self::NoSink
            | Self::Rejected
            | Self::NotDelivered
            | Self::TransportFailed => EpochPushSeverity::NotCarried,
        }
    }
}

/// The one-line operator rendering of a push outcome — what the desk paths print at the
/// end of a session / on a demand, so "did the revocation land?" is answerable without
/// reading the enum or grepping the dial's logs.
impl std::fmt::Display for EpochPushOutcome {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Applied { epoch } => {
                write!(f, "DELIVERED — the robot applied epoch {epoch}")
            }
            Self::AlreadyCurrent { epoch } => {
                write!(f, "already current — the robot is at epoch {epoch}")
            }
            Self::NoCacheDir => write!(
                f,
                "nothing pushed — this desk has no revocation-epoch cache directory"
            ),
            Self::UnverifiedRobotIdentity => write!(
                f,
                "nothing pushed — this desk has no pinned name for the robot it dialed, so it \
                 cannot tell which cached epoch is meant (pair/pin it)"
            ),
            Self::NoCachedEpoch => write!(
                f,
                "nothing pushed — no cached epoch for this robot on this desk"
            ),
            Self::CacheUnreadable => write!(
                f,
                "NOT delivering revocations — the cached epoch is unreadable (re-sync it)"
            ),
            Self::CacheTooLarge { bytes } => write!(
                f,
                "NOT delivering revocations — the cached epoch is too large to send \
                 ({bytes} bytes; re-sync it)"
            ),
            Self::NoSink => write!(
                f,
                "NOT delivered — the robot accepts no epoch sync (upgrade/configure it)"
            ),
            Self::Rejected => write!(
                f,
                "NOT delivered — the robot REJECTED the epoch (investigate it)"
            ),
            Self::NotDelivered => {
                write!(f, "NOT delivered — the robot's answer was unusable")
            }
            Self::TransportFailed => write!(
                f,
                "NOT delivered — the control stream failed mid-push (a re-dial recovers)"
            ),
        }
    }
}

/// The fixed JSON scaffolding around the hex blob in a `sync_epoch` control frame:
/// `{"verb":"sync_epoch","epoch_postcard":"<hex>"}` minus the hex. Pinned against the
/// REAL serialization by `control_frame_overhead_matches_the_real_encoding`, so a
/// vocabulary rename can never silently invalidate the early size gate.
const CONTROL_FRAME_OVERHEAD: usize = 41;

/// How far above the frame cap the EARLY (file-size-only) gate must be certain before
/// it refuses. Its bound is deliberately approximate (it cannot see whitespace or the
/// exact base64 tail), so everything within this margin of the cap is left to the exact
/// post-encode guard, which reports a precise size. A grossly oversized cache — the
/// only case the early gate exists for — is orders of magnitude past it.
const EARLY_GATE_MARGIN: usize = 1024;

/// A LOWER BOUND on the control-frame size a cached artifact of `cache_file_len` bytes
/// would produce — computable from the file's SIZE ALONE, before a byte is read.
///
/// The on-disk artifact is unpadded base64url (4 chars per 3 bytes) and the control
/// frame carries it as hex (2 chars per byte), so `frame = 2·N + CONTROL_FRAME_OVERHEAD`
/// where `N = floor(3·L/4)` for a whitespace-free file. [`WHITESPACE_SLACK`] bytes are
/// deducted first so a trailing newline (or a few) can only make the bound SMALLER —
/// this must never over-estimate, or a legitimate artifact a hair under the cap would
/// be refused.
///
/// Why it exists: the exact guard needs the decoded artifact, the hex string AND the
/// JSON frame in memory at once (~6× the file). A pathological cache would therefore
/// cost ~6× its size to REJECT. This bound refuses the provably-impossible ones for
/// O(1) memory, and everything that survives it is still measured exactly.
///
/// It is compared against `DEFAULT_MAX_FRAME_LEN + `[`EARLY_GATE_MARGIN`], not against
/// the cap itself, so the two guards have crisply separable domains: anything NEAR the
/// boundary is measured exactly (and reports its exact size), and only the grossly
/// oversized take this coarse path.
fn frame_len_lower_bound_for_cache(cache_file_len: usize) -> usize {
    /// Trailing `\n` / `\r\n` / stray spaces a hand-edited or tool-written cache may
    /// carry. Generous: the cost of over-slack is only that a few pathological files
    /// take the exact (post-read) path instead of the early one.
    const WHITESPACE_SLACK: usize = 16;
    let b64_len = cache_file_len.saturating_sub(WHITESPACE_SLACK);
    let artifact_bytes = b64_len / 4 * 3;
    2 * artifact_bytes + CONTROL_FRAME_OVERHEAD
}

/// What [`prepare_epoch_push`] decided: either there is nothing to send (with the
/// already-logged reason), or here is the exact control frame to write.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EpochPushPlan {
    /// Nothing to send — the carried outcome is the reason (already logged).
    Skip(EpochPushOutcome),
    /// Send this pre-encoded control frame, then hand the reply to
    /// [`classify_epoch_reply`].
    Send {
        /// The `sync_epoch` request, already JSON-encoded and size-checked against the
        /// peer's frame cap.
        frame: Vec<u8>,
    },
}

/// Decide whether to push an epoch to `robot`, and encode the frame if so.
///
/// `cache_path` is the desk's `<robot>.epoch` artifact path, or `None` when this desk
/// has no epoch-cache directory at all.
///
/// `note_missing` is consulted ONLY on the "there is no cached artifact" branch, and
/// returning `true` logs that (INFO) line: a long-lived caller that re-dials (netd)
/// passes a gate that dedups to once per (robot, process) so a reconnect loop cannot
/// flood, while a one-shot caller (`cerulion connect`) passes `&|| true`. It is a
/// callback rather than a `bool` precisely so a desk whose cache DISAPPEARS between
/// dials still gets the line — a precomputed flag would have been consumed by an
/// earlier, healthy dial.
///
/// Never returns an error: every failure mode is a [`EpochPushPlan::Skip`] carrying a
/// classified, already-logged outcome — the never-block-the-connection floor.
pub fn prepare_epoch_push(
    robot: &str,
    cache_path: Option<&Path>,
    note_missing: &dyn Fn() -> bool,
) -> EpochPushPlan {
    let Some(path) = cache_path else {
        // The no-cache-dir case is logged ONCE by the caller's config resolution (with
        // the remediation), so this per-dial line stays at debug.
        tracing::debug!(
            robot = %robot,
            "cerulion: no revocation-epoch cache directory on this desk — pushing none"
        );
        return EpochPushPlan::Skip(EpochPushOutcome::NoCacheDir);
    };
    // EARLY size gate: a cache whose FILE SIZE alone proves the control frame cannot
    // fit is refused before it is read, so a pathological artifact costs O(1) memory to
    // reject instead of ~6× its size (decoded artifact + hex + JSON frame all live at
    // once in the exact guard below). The bound is deliberately conservative — anything
    // it lets through is still measured exactly.
    if let Ok(meta) = std::fs::metadata(path) {
        let lower_bound = frame_len_lower_bound_for_cache(meta.len() as usize);
        if lower_bound > DEFAULT_MAX_FRAME_LEN + EARLY_GATE_MARGIN {
            tracing::warn!(
                robot = %robot,
                cache = %path.display(),
                cache_file_bytes = meta.len(),
                min_frame_bytes = lower_bound,
                max_frame_bytes = DEFAULT_MAX_FRAME_LEN,
                "cerulion: the cached revocation epoch is TOO LARGE to push (its file size alone \
                 exceeds what the peer's frame cap could carry) — pushing NOTHING so the \
                 connection still comes up; this desk is NOT delivering revocations to this robot\
                 . Re-sync the epoch from the account service."
            );
            return EpochPushPlan::Skip(EpochPushOutcome::CacheTooLarge { bytes: lower_bound });
        }
    }
    let blob = match resolve_epoch_sync(path) {
        Ok(Some(b)) => b,
        Ok(None) => {
            // The normal state for a desk that has never synced this robot (and for
            // every guest desk — the account service's access endpoint is owner-only).
            // Nothing is weakened: the robot keeps its current epoch.
            //
            // At INFO, not debug: this is ALSO what a MISMATCHED cache filename looks
            // like (the file is keyed by the robot name this desk knows it by, while
            // the writer keys it by whatever identity the account page holds), which
            // would otherwise be an indefinite, completely silent non-delivery. Naming
            // the exact path we looked for turns that from invisible into obvious.
            if note_missing() {
                tracing::info!(
                    robot = %robot,
                    cache = %path.display(),
                    "cerulion: no cached revocation epoch for this robot — pushing NONE\
                     . If you expected one, check that the sync wrote EXACTLY this \
                     path (the file name is the robot name this desk knows it by)."
                );
            }
            return EpochPushPlan::Skip(EpochPushOutcome::NoCachedEpoch);
        }
        Err(e) => {
            // A corrupt/unreadable cache is a real misconfiguration: this desk is
            // NOT carrying revocations. Loud, but never fatal to the dial.
            //
            // TWO shapes, kept distinct in the MESSAGE because the operator ACTIONS
            // differ (the same discipline the three `NoSink` shapes follow): an
            // artifact stamped with an envelope version THIS BUILD does not understand
            // is an UPGRADE problem — the writer (the account service / Studio) is
            // ahead of this desk — and telling that operator to "re-sync" sends them
            // to re-fetch the very artifact they cannot read
            // (a version refusal must never be reported as a damaged file).
            let text = e.to_string();
            if text.contains(EPOCH_VERSION_UNSUPPORTED_NEEDLE) {
                tracing::warn!(
                    robot = %robot,
                    cache = %path.display(),
                    error = %text,
                    "cerulion: the cached revocation epoch was written in an epoch-sync ENVELOPE \
                     VERSION this build does not understand — this desk is NOT delivering \
                     revocations to this robot; the connection proceeds. UPGRADE this \
                     desk (re-syncing will fetch the same shape); the epoch itself is not at \
                     fault."
                );
            } else {
                tracing::warn!(
                    robot = %robot,
                    cache = %path.display(),
                    error = %text,
                    "cerulion: could not read the cached revocation epoch — this desk is NOT \
                     delivering revocations to this robot; the connection proceeds. \
                     Re-sync the epoch from the account service."
                );
            }
            return EpochPushPlan::Skip(EpochPushOutcome::CacheUnreadable);
        }
    };
    let request = WireRequest::SyncEpoch {
        epoch_postcard: blob,
    };
    let frame = match serde_json::to_vec(&request) {
        Ok(f) => f,
        Err(e) => {
            tracing::warn!(
                robot = %robot,
                error = %e,
                "cerulion: could not encode the revocation-epoch push request — pushing NOTHING; \
                 the connection proceeds"
            );
            return EpochPushPlan::Skip(EpochPushOutcome::CacheUnreadable);
        }
    };
    // THE SIZE GUARD. The peer reads control frames with a `DEFAULT_MAX_FRAME_LEN`
    // cap, and an over-cap frame is not merely rejected — it consumes the length
    // prefix and PERMANENTLY desyncs the robot's control stream, so the push would
    // fail as a TRANSPORT error and take the whole dial (and every re-dial) down with
    // it. A pathological cache must stay a policy failure, so it is never sent.
    if frame.len() > DEFAULT_MAX_FRAME_LEN {
        tracing::warn!(
            robot = %robot,
            cache = %path.display(),
            frame_bytes = frame.len(),
            max_frame_bytes = DEFAULT_MAX_FRAME_LEN,
            "cerulion: the cached revocation epoch is TOO LARGE to push (its control frame \
             exceeds the peer's frame cap) — pushing NOTHING so the connection still comes up; \
             this desk is NOT delivering revocations to this robot. Re-sync the epoch \
             from the account service."
        );
        return EpochPushPlan::Skip(EpochPushOutcome::CacheTooLarge { bytes: frame.len() });
    }
    EpochPushPlan::Send { frame }
}

/// Classify (and log) the robot's answer to a `sync_epoch` push.
///
/// Decoded exactly as the catalog gate decodes its reply, so an `AcceptDecision`
/// frame surfaces its named reason instead of collapsing into a generic decode error.
pub fn classify_epoch_reply(robot: &str, reply: &[u8]) -> EpochPushOutcome {
    match decode_first_reply(reply) {
        Ok(WireResponse::EpochSynced {
            epoch,
            applied: true,
        }) => {
            tracing::info!(
                robot = %robot,
                epoch,
                "cerulion: DELIVERED a newer revocation epoch to the robot — its \
                 access list is now current"
            );
            EpochPushOutcome::Applied { epoch }
        }
        Ok(WireResponse::EpochSynced {
            epoch,
            applied: false,
        }) => {
            // The steady state (every desk pushes its cache on every connect) — debug,
            // or it would flood on reconnect loops.
            tracing::debug!(
                robot = %robot,
                epoch,
                "cerulion: the robot was already at or ahead of our cached revocation epoch — \
                 no-op"
            );
            EpochPushOutcome::AlreadyCurrent { epoch }
        }
        // FOUR structurally different refusals, kept DISTINCT in the log because
        // "upgrade the peer" and "investigate this epoch" are different operator
        // actions (the loud/precise bar). The first THREE all mean the robot has no
        // usable epoch sink FOR THIS ARTIFACT, so they share the `NoSink` outcome.
        Ok(WireResponse::Error { message, .. }) if message.contains(NO_EPOCH_SINK_NEEDLE) => {
            tracing::warn!(
                robot = %robot,
                reason = %sanitize_peer_text(&message),
                "cerulion: this robot does NOT accept epoch sync (no sync sink wired) — it will \
                 not learn revocations from this desk; the connection proceeds. \
                 Upgrade/configure the robot."
            );
            EpochPushOutcome::NoSink
        }
        // A robot PREDATING the verb cannot answer with the sink needle at all: it
        // fails to deserialize the unknown `verb` tag and answers with its
        // malformed-request marker. Classifying that as `Rejected` would send an
        // operator hunting a forged epoch when the real fix is a robot upgrade.
        Ok(WireResponse::Error { message, .. }) if message.contains(UNDECODABLE_REQUEST_NEEDLE) => {
            tracing::warn!(
                robot = %robot,
                reason = %sanitize_peer_text(&message),
                "cerulion: this robot could not DECODE the revocation-epoch push — it predates \
                 the `sync_epoch` verb and will not learn revocations from this desk; the \
                 connection proceeds. Upgrade the robot."
            );
            EpochPushOutcome::NoSink
        }
        // A robot that KNOWS the verb but refuses the artifact's ENVELOPE VERSION is
        // saying "one of us is the wrong build" — the SAME upgrade-shaped class, one
        // generation later. Reporting it as `Rejected` would send an operator hunting
        // a forged epoch / a skewed clock when the real fix is a version bump on one
        // side: exactly the misdirection the `UNDECODABLE_REQUEST_NEEDLE` arm above
        // exists to prevent, arriving through a different door.
        Ok(WireResponse::Error { message, .. })
            if message.contains(EPOCH_VERSION_UNSUPPORTED_NEEDLE) =>
        {
            tracing::warn!(
                robot = %robot,
                reason = %sanitize_peer_text(&message),
                "cerulion: this robot could not read the revocation-epoch artifact's ENVELOPE \
                 VERSION — the two builds disagree on the epoch-sync wire shape, so it will not \
                 learn revocations from this desk; the connection proceeds. Upgrade the \
                 older side (robot or desk); the epoch itself is NOT implicated."
            );
            EpochPushOutcome::NoSink
        }
        Ok(WireResponse::Error { message, .. }) => {
            tracing::warn!(
                robot = %robot,
                reason = %sanitize_peer_text(&message),
                "cerulion: the robot REJECTED the pushed revocation epoch — it is NOT current; \
                 the connection proceeds. Investigate the epoch (forged, issued for a \
                 different robot, or a skewed robot clock)."
            );
            EpochPushOutcome::Rejected
        }
        Ok(other) => {
            tracing::warn!(
                robot = %robot,
                reply = %sanitize_peer_text(&format!("{other:?}")),
                "cerulion: the robot answered the revocation-epoch push with an unexpected reply \
                 — treating it as NOT delivered"
            );
            EpochPushOutcome::NotDelivered
        }
        Err(Some(reason)) => {
            tracing::warn!(
                robot = %robot,
                reason = %sanitize_peer_text(&reason),
                "cerulion: the robot refused the connection at the revocation-epoch push — the \
                 epoch was NOT delivered"
            );
            EpochPushOutcome::Rejected
        }
        Err(None) => {
            tracing::warn!(
                robot = %robot,
                "cerulion: the robot's revocation-epoch reply decoded as neither a wire response \
                 nor an accept decision — treating it as NOT delivered"
            );
            EpochPushOutcome::NotDelivered
        }
    }
}

/// Log the one FATAL class: the control round-trip failed, so the epoch was not
/// delivered AND the stream is desynced. The caller records
/// [`EpochPushOutcome::TransportFailed`] and fails the dial (a re-dial recovers).
pub fn note_transport_failure(robot: &str, error: &str) {
    tracing::warn!(
        robot = %robot,
        error = %error,
        "cerulion: the revocation-epoch push failed on the control stream — the epoch was NOT \
         delivered and this connection is unusable; re-dialing"
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_pairing::format::{
        AccessListEpoch, AccountId, IntermediateCert, PublicKey, RobotId, Scope, Validity,
        FORMAT_VERSION,
    };
    use cerulion_pairing::verify::EpochSyncWire;

    /// Serializes the env-mutating tests in this binary (the process env is global).
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// RAII: restore (or remove) `CERULION_EPOCH_DIR` even on a panicking test.
    struct EpochDirEnvGuard(Option<String>);
    impl EpochDirEnvGuard {
        fn set(value: Option<&str>) -> Self {
            let prev = std::env::var(EPOCH_DIR_ENV).ok();
            match value {
                Some(v) => std::env::set_var(EPOCH_DIR_ENV, v),
                None => std::env::remove_var(EPOCH_DIR_ENV),
            }
            Self(prev)
        }
    }
    impl Drop for EpochDirEnvGuard {
        fn drop(&mut self) {
            match self.0.take() {
                Some(v) => std::env::set_var(EPOCH_DIR_ENV, v),
                None => std::env::remove_var(EPOCH_DIR_ENV),
            }
        }
    }

    /// The `cerulion connect` THIRD of the THREE-party
    /// agreement pin: the connect binary's epoch-cache path resolution
    /// ([`resolve_epoch_dir_from_env`] + [`epoch_cache_path`], the exact pair
    /// `ConnectCli::into_config` and the worker call) must equal, byte for byte, the
    /// literal oracle the other two halves are anchored to —
    /// `cerulion_netd::wan::netd_from_env_resolves_the_shared_epoch_cache_path` (the WAN
    /// reader, through netd's real `from_env`) and
    /// `cerulion_cli_engine::account_cmd::the_writer_resolves_the_shared_epoch_cache_path`
    /// (the cache WRITER, through its real `resolve_epoch_cache_path`).
    ///
    /// The three halves are separate tests because the three paths live in separate
    /// crates (netd must not depend on connectd — that edge is why this crate exists —
    /// and the writer must stay iroh-free, so it cannot see this crate at all), so each
    /// pins ITS production resolution against the SAME hand-written path string: if any
    /// path re-grows a private env name, a private directory rule, or a private join,
    /// ITS test fails.
    ///
    /// SCOPE: the connect binary's `into_config` is a `main.rs` one-liner over
    /// these functions, so what is pinned here is the shared composition, not that
    /// `main.rs` calls it (there is exactly one call site, and `resolve_epoch_dir` is
    /// the only public way to spell the rule).
    #[test]
    fn both_desk_paths_resolve_one_cache_path() {
        let _lock = ENV_LOCK.lock().unwrap_or_else(|p| p.into_inner());
        let key = Path::new("/home/x/.cerulion/desk.key");

        // 1. No override ⇒ the sibling `epochs/` dir next to the desk key.
        let _g = EpochDirEnvGuard::set(None);
        let dir = resolve_epoch_dir_from_env(Some(key)).expect("a key file yields a cache dir");
        assert_eq!(
            epoch_cache_path(&dir, "go2"),
            PathBuf::from("/home/x/.cerulion/epochs/go2.epoch"),
            "the connect path must resolve the literal shared oracle"
        );
        drop(_g);

        // 2. The env override RELOCATES the cache — for THIS path too (if only
        //    netd honored an override, a relocated cache would be invisible here).
        let _g = EpochDirEnvGuard::set(Some("/etc/cerulion/epochs"));
        let dir = resolve_epoch_dir_from_env(Some(key)).expect("the override yields a cache dir");
        assert_eq!(
            epoch_cache_path(&dir, "go2"),
            PathBuf::from("/etc/cerulion/epochs/go2.epoch"),
            "the shared env override must relocate the CONNECT path's cache as well"
        );
        // …and it works with an ephemeral desk key (no key file) too.
        assert_eq!(
            resolve_epoch_dir_from_env(None),
            Some(PathBuf::from("/etc/cerulion/epochs"))
        );
        drop(_g);

        // 3. A blank override is NOT an override (an exported-but-empty var must not
        //    silently relocate the cache to the current directory).
        let _g = EpochDirEnvGuard::set(Some("   "));
        assert_eq!(
            resolve_epoch_dir_from_env(Some(key)),
            Some(PathBuf::from("/home/x/.cerulion/epochs"))
        );
        drop(_g);

        // 4. Ephemeral desk key + no override ⇒ no cache location at all (a no-op
        //    push, never a refusal).
        let _g = EpochDirEnvGuard::set(None);
        assert_eq!(resolve_epoch_dir_from_env(None), None);
    }

    fn artifact(epoch: u64) -> EpochSyncWire {
        let inter_sk = ed25519_dalek::SigningKey::from_bytes(&[11u8; 32]);
        let root_sk = ed25519_dalek::SigningKey::from_bytes(&[1u8; 32]);
        let inter_pk = PublicKey(inter_sk.verifying_key().to_bytes());
        EpochSyncWire::new(
            IntermediateCert {
                version: FORMAT_VERSION,
                intermediate_key: inter_pk,
                validity: Validity {
                    not_before_ns: 0,
                    not_after_ns: 1_000_000_000_000_000,
                },
                issued_at_ns: 1,
                max_scope: Scope::OWNER_FULL,
            }
            .sign_by_roots(&[&root_sk]),
            AccessListEpoch {
                version: FORMAT_VERSION,
                robot: RobotId([0x0B; 32]),
                epoch,
                revoked_accounts: vec![AccountId([0x0C; 32])],
                revoked_devices: vec![],
                issued_at_ns: 2,
                issuer_key: inter_pk,
            }
            .sign(&inter_sk),
        )
    }

    /// The three no-send states are told apart, and a real cached artifact produces a
    /// frame carrying EXACTLY the cached bytes (hex of the postcard) — hand oracle.
    #[test]
    fn prepare_classifies_every_no_send_state_and_encodes_a_real_one() {
        // No directory at all.
        assert_eq!(
            prepare_epoch_push("go2", None, &|| true),
            EpochPushPlan::Skip(EpochPushOutcome::NoCacheDir)
        );

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(epoch_cache_file_name_for_test("go2"));

        // Directory present, no artifact.
        assert_eq!(
            prepare_epoch_push("go2", Some(&path), &|| true),
            EpochPushPlan::Skip(EpochPushOutcome::NoCachedEpoch)
        );

        // Corrupt artifact.
        std::fs::write(&path, "not an epoch artifact").unwrap();
        assert_eq!(
            prepare_epoch_push("go2", Some(&path), &|| true),
            EpochPushPlan::Skip(EpochPushOutcome::CacheUnreadable)
        );

        // A real artifact ⇒ a Send carrying the exact cached bytes.
        let art = artifact(9);
        std::fs::write(&path, crate::config::encode_epoch_cache(&art).unwrap()).unwrap();
        let EpochPushPlan::Send { frame } = prepare_epoch_push("go2", Some(&path), &|| true) else {
            panic!("a valid cache must produce a Send plan");
        };
        let request: serde_json::Value = serde_json::from_slice(&frame).unwrap();
        assert_eq!(request["verb"], "sync_epoch");
        let hexed = request["epoch_postcard"].as_str().unwrap();
        assert_eq!(
            hex::decode(hexed).unwrap(),
            art.to_postcard().unwrap(),
            "the desk is a courier: the pushed bytes ARE the cached artifact"
        );
    }

    fn epoch_cache_file_name_for_test(robot: &str) -> String {
        cerulion_pairing::verify::epoch_cache_file_name(robot)
    }

    /// The `sync_epoch` control frame is exactly `2·N + CONTROL_FRAME_OVERHEAD` bytes
    /// for an `N`-byte artifact — the arithmetic BOTH size guards rest on. Pinned
    /// against the REAL serialization (an empty blob ⇒ pure scaffolding), so a
    /// vocabulary rename that changed the overhead could not silently skew the early
    /// gate's lower bound into over-estimating.
    #[test]
    fn control_frame_overhead_matches_the_real_encoding() {
        let empty = serde_json::to_vec(&WireRequest::SyncEpoch {
            epoch_postcard: String::new(),
        })
        .unwrap();
        assert_eq!(
            empty.len(),
            CONTROL_FRAME_OVERHEAD,
            "the scaffolding is {:?}",
            String::from_utf8_lossy(&empty)
        );
        // And with a blob, the frame grows by exactly the hex length.
        let with_blob = serde_json::to_vec(&WireRequest::SyncEpoch {
            epoch_postcard: "ab".repeat(10),
        })
        .unwrap();
        assert_eq!(with_blob.len(), CONTROL_FRAME_OVERHEAD + 20);
    }

    /// The EARLY (file-size-only) gate's bound must never OVER-estimate — an
    /// over-estimate would refuse a legitimate artifact a hair under the cap — and must
    /// still bite on a file that provably cannot fit. Hand oracles.
    #[test]
    fn early_size_bound_is_conservative_and_still_bites() {
        // A whitespace-free base64url file of length L encodes floor(3L/4) bytes; the
        // bound deducts 16 slack bytes first, so it is always <= the true frame length.
        for artifact_bytes in [0usize, 1, 3, 100, 4096, 1_000_000] {
            let b64_len = artifact_bytes.div_ceil(3) * 4; // >= the real unpadded length
            let true_frame = 2 * artifact_bytes + CONTROL_FRAME_OVERHEAD;
            assert!(
                frame_len_lower_bound_for_cache(b64_len) <= true_frame,
                "the bound must never exceed the true frame length \
                 (artifact_bytes={artifact_bytes})"
            );
            // A trailing newline (or CRLF) must not push the bound above the truth.
            assert!(frame_len_lower_bound_for_cache(b64_len + 2) <= true_frame);
        }
        // A file far over the cap is refused on size alone.
        assert!(
            frame_len_lower_bound_for_cache(3 * DEFAULT_MAX_FRAME_LEN)
                > DEFAULT_MAX_FRAME_LEN + EARLY_GATE_MARGIN
        );
        // …while a file that could still fit is NOT (it takes the exact path).
        assert!(
            frame_len_lower_bound_for_cache(DEFAULT_MAX_FRAME_LEN / 4)
                <= DEFAULT_MAX_FRAME_LEN + EARLY_GATE_MARGIN
        );
    }

    /// An artifact whose encoded frame would exceed the peer's frame cap is a POLICY
    /// skip (`CacheTooLarge`), never a send that would desync the robot's control
    /// stream and permanently fail every dial — with a JUST-UNDER control proving the
    /// guard is a boundary, not a blanket refusal of large artifacts.
    ///
    /// The expectations are HAND-COMPUTED from the artifact's own postcard length
    /// (`2·N + CONTROL_FRAME_OVERHEAD`), never read back off the outcome, so a guard
    /// that reported the wrong number — or fired at the wrong threshold — fails here.
    #[test]
    fn the_frame_cap_is_a_boundary_not_a_blanket_refusal() {
        let dir = tempfile::tempdir().unwrap();

        // Pad a REAL epoch's revoked-account list until its frame lands just under /
        // just over the cap. Each AccountId costs 32 postcard bytes ⇒ 64 frame bytes.
        let sized_artifact = |accounts: usize| {
            let mut art = artifact(3);
            art.signed_epoch.epoch_data.revoked_accounts =
                (0..accounts).map(|i| AccountId([i as u8; 32])).collect();
            art
        };
        let frame_len_of = |art: &EpochSyncWire| {
            // The independent oracle: hex is 2 chars per artifact byte, plus the fixed
            // JSON scaffolding.
            2 * art.to_postcard().unwrap().len() + CONTROL_FRAME_OVERHEAD
        };

        // Binary-search-free: start from the exact per-account cost.
        let base = frame_len_of(&sized_artifact(0));
        let per_account = frame_len_of(&sized_artifact(1)) - base;
        let just_under_count = (DEFAULT_MAX_FRAME_LEN - base) / per_account;

        // --- JUST UNDER the cap: a Send carrying a frame of EXACTLY the hand-computed
        //     length (also exercises the early gate's "let it through" branch).
        let under = sized_artifact(just_under_count);
        let expected_under = frame_len_of(&under);
        assert!(expected_under <= DEFAULT_MAX_FRAME_LEN);
        assert!(
            expected_under + per_account > DEFAULT_MAX_FRAME_LEN,
            "the fixture must sit ON the boundary, not merely below it"
        );
        let path = dir.path().join(epoch_cache_file_name_for_test("under"));
        std::fs::write(&path, crate::config::encode_epoch_cache(&under).unwrap()).unwrap();
        match prepare_epoch_push("under", Some(&path), &|| true) {
            EpochPushPlan::Send { frame } => assert_eq!(
                frame.len(),
                expected_under,
                "an artifact at the boundary must be SENT, with the hand-computed frame size"
            ),
            other => panic!("a just-under-cap cache must be a Send, got {other:?}"),
        }

        // --- JUST OVER the cap (one more account): a policy skip reporting the
        //     hand-computed frame size EXACTLY (the exact, post-encode guard).
        let over = sized_artifact(just_under_count + 1);
        let expected_over = frame_len_of(&over);
        assert!(expected_over > DEFAULT_MAX_FRAME_LEN);
        let path = dir.path().join(epoch_cache_file_name_for_test("over"));
        std::fs::write(&path, crate::config::encode_epoch_cache(&over).unwrap()).unwrap();
        assert_eq!(
            prepare_epoch_push("over", Some(&path), &|| true),
            EpochPushPlan::Skip(EpochPushOutcome::CacheTooLarge {
                bytes: expected_over
            }),
            "one account over the boundary must be refused, reporting the exact frame size"
        );

        // --- FAR over the cap: refused by the EARLY (file-size-only) gate, BEFORE a
        //     byte of the artifact is read.
        //
        //     This arm DISCRIMINATES between the two guards, which is the whole point:
        //     the exact post-encode guard reports `frame.len()` (== `true_huge`), while
        //     the early gate reports a bound HAND-COMPUTED below from the file's SIZE
        //     alone — provably STRICTLY smaller (the 16-byte whitespace slack alone puts
        //     it >= 19 frame bytes under). Asserting `bytes <= true_huge` (as this arm
        //     first did) let BOTH guards land inside the window, so neutering the early
        //     gate's threshold left the test green: it measured only wall time. The
        //     exact-equality + strict-`<` asserts below FAIL if the exact guard fired.
        let huge = sized_artifact(just_under_count * 2);
        let true_huge = frame_len_of(&huge);
        let path = dir.path().join(epoch_cache_file_name_for_test("huge"));
        std::fs::write(&path, crate::config::encode_epoch_cache(&huge).unwrap()).unwrap();
        let file_len = std::fs::metadata(&path).unwrap().len() as usize;
        // The early gate's bound, re-derived HERE from the documented arithmetic (NOT
        // read back off the helper): the on-disk artifact is unpadded base64url (4 chars
        // per 3 bytes), 16 slack bytes are deducted first so trailing whitespace can only
        // shrink it, and the control frame hex-encodes every artifact byte (2 chars each)
        // on top of the fixed JSON scaffolding.
        let expected_early = 2 * ((file_len - 16) / 4 * 3) + CONTROL_FRAME_OVERHEAD;
        assert!(
            expected_early > DEFAULT_MAX_FRAME_LEN + EARLY_GATE_MARGIN,
            "the fixture must be far enough over the cap to reach the EARLY gate \
             (bound {expected_early}, gate {})",
            DEFAULT_MAX_FRAME_LEN + EARLY_GATE_MARGIN
        );
        assert!(
            expected_early < true_huge,
            "the early bound must sit STRICTLY below the exact frame length, or this arm \
             could not tell the two guards apart (bound {expected_early}, exact {true_huge})"
        );
        match prepare_epoch_push("huge", Some(&path), &|| true) {
            EpochPushPlan::Skip(EpochPushOutcome::CacheTooLarge { bytes }) => {
                assert_eq!(
                    bytes, expected_early,
                    "a far-over-cap cache must be refused by the EARLY (file-size-only) gate, \
                     reporting its hand-computed lower bound — got {bytes}, which is the exact \
                     post-encode frame length ({true_huge}) if the early gate did not fire"
                );
                assert!(
                    bytes > DEFAULT_MAX_FRAME_LEN && bytes < true_huge,
                    "the early gate must report an over-cap bound STRICTLY below the true frame \
                     length (got {bytes}, true frame {true_huge})"
                );
            }
            other => panic!("a far-over-cap cache must be a CacheTooLarge skip, got {other:?}"),
        }
    }

    /// Every reply shape maps to its outcome — hand oracles, including the THREE
    /// distinct no-sink shapes (a sink-less modern robot, a robot predating the verb,
    /// and an envelope-VERSION skew) which must NOT be reported as a rejected epoch.
    #[test]
    fn classify_maps_every_reply_shape() {
        let enc = |r: &WireResponse| serde_json::to_vec(r).unwrap();

        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::EpochSynced {
                    epoch: 7,
                    applied: true
                })
            ),
            EpochPushOutcome::Applied { epoch: 7 }
        );
        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::EpochSynced {
                    epoch: 7,
                    applied: false
                })
            ),
            EpochPushOutcome::AlreadyCurrent { epoch: 7 }
        );
        // A modern robot with no sink wired.
        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::Error {
                    topic: None,
                    message: format!("this robot {NO_EPOCH_SINK_NEEDLE} (no sink)"),
                })
            ),
            EpochPushOutcome::NoSink
        );
        // A robot PREDATING the verb: its malformed-request marker, NOT the sink
        // needle. (The REAL serde error text is pinned in
        // `cerulion_connectd/tests/protocol_parity_test.rs`.)
        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::Error {
                    topic: None,
                    message: format!(
                        "{UNDECODABLE_REQUEST_NEEDLE}: unknown variant `sync_epoch`, expected \
                         one of `catalog`, `demand`, `undemand`, `schema`, `status`"
                    ),
                })
            ),
            EpochPushOutcome::NoSink,
            "a robot that cannot decode the verb is a NoSink (upgrade it), never a Rejected \
             epoch (investigate it)"
        );
        // A robot that KNOWS the verb but refuses the
        // artifact's ENVELOPE VERSION. Same upgrade-shaped class, one generation later
        // — reporting it as `Rejected` re-creates the exact operator misdirection the
        // arm above exists to prevent. The needle is produced by the REAL refusal path
        // (`EpochSyncWire::from_postcard`), not hand-typed prose, so this arm cannot
        // pass while the robot's actual message drifts.
        let real_version_refusal = {
            let mut art = artifact(4);
            art.version = cerulion_pairing::verify::EPOCH_SYNC_WIRE_VERSION + 7;
            let bytes = art.to_postcard().expect("encode a future-version envelope");
            cerulion_pairing::verify::EpochSyncWire::from_postcard(&bytes)
                .expect_err("a future envelope version must be refused")
                .to_string()
        };
        assert!(
            real_version_refusal
                .contains(cerulion_pairing::verify::EPOCH_VERSION_UNSUPPORTED_NEEDLE),
            "the shared needle must lead the REAL refusal: {real_version_refusal}"
        );
        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::Error {
                    topic: None,
                    message: real_version_refusal,
                })
            ),
            EpochPushOutcome::NoSink,
            "an envelope-VERSION skew is a NoSink (upgrade a build), never a Rejected epoch \
             (investigate a forgery / a skewed clock)"
        );
        // A genuine epoch rejection.
        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::Error {
                    topic: None,
                    message: "the pushed access-list epoch was REJECTED: bad signature".into(),
                })
            ),
            EpochPushOutcome::Rejected
        );
        // An unexpected reply shape.
        assert_eq!(
            classify_epoch_reply(
                "r",
                &enc(&WireResponse::DemandAccepted {
                    topic: "/imu".into()
                })
            ),
            EpochPushOutcome::NotDelivered
        );
        // Undecodable bytes.
        assert_eq!(
            classify_epoch_reply("r", b"not json"),
            EpochPushOutcome::NotDelivered
        );
        // An AcceptDecision refusal frame (the robot refused the connection) — built
        // through the REAL type so the tag can never drift out from under this arm.
        let refusal = serde_json::to_vec(&crate::protocol::AcceptDecision::Refuse {
            reason: "unpaired desk key".into(),
        })
        .unwrap();
        assert_eq!(
            classify_epoch_reply("r", &refusal),
            EpochPushOutcome::Rejected
        );
    }

    /// The three `NoSink` shapes share an OUTCOME but must never share a REMEDIATION:
    /// "configure the robot", "upgrade the robot", "upgrade the older side" are three
    /// different operator actions, and the enum alone cannot tell them apart. Asserting
    /// only the discriminant would let the arms silently
    /// collapse into one message. Each is pinned by its own distinctive phrase, and
    /// each is asserted ABSENT from the other two arms (so a copy-paste collapse fails).
    #[test]
    #[tracing_test::traced_test]
    fn the_three_no_sink_arms_keep_distinct_operator_remediations() {
        let err = |message: String| {
            serde_json::to_vec(&WireResponse::Error {
                topic: None,
                message,
            })
            .unwrap()
        };
        // The distinctive RENDERED phrase of each arm (the operator's ACTION). The
        // messages are written with `\`-continuations, which Rust strips along with the
        // following indentation, so these are the exact runtime strings.
        let no_sink_wired = "no sync sink wired";
        let predates = "predates the `sync_epoch` verb";
        let version_skew = "ENVELOPE VERSION";

        // Asserted in arm order: `logs_contain` accumulates over the whole test, so
        // each "must NOT contain another arm's remediation" check is made against the
        // logs emitted SO FAR.
        assert_eq!(
            classify_epoch_reply("r", &err(format!("x {NO_EPOCH_SINK_NEEDLE} y"))),
            EpochPushOutcome::NoSink
        );
        assert!(logs_contain(no_sink_wired), "arm 1 names its own cause");
        assert!(
            !logs_contain(predates) && !logs_contain(version_skew),
            "arm 1 must not carry another arm's remediation"
        );

        assert_eq!(
            classify_epoch_reply("r", &err(format!("{UNDECODABLE_REQUEST_NEEDLE}: bad verb"))),
            EpochPushOutcome::NoSink
        );
        assert!(
            logs_contain(predates),
            "arm 2 names the upgrade-the-robot fix"
        );
        assert!(
            !logs_contain(version_skew),
            "arm 2 must not carry the version-skew remediation"
        );

        assert_eq!(
            classify_epoch_reply(
                "r",
                &err(format!(
                    "{EPOCH_VERSION_UNSUPPORTED_NEEDLE} 9 is not supported"
                ))
            ),
            EpochPushOutcome::NoSink
        );
        assert!(
            logs_contain(version_skew) && logs_contain("the epoch itself is NOT implicated"),
            "arm 3 must say the EPOCH is not at fault — the misdirection kernel C fixed"
        );
        // And the genuine rejection keeps the investigate-the-epoch remediation, which
        // must never appear on a NoSink arm.
        assert_eq!(
            classify_epoch_reply("r", &err("epoch REJECTED: bad signature".to_string())),
            EpochPushOutcome::Rejected
        );
        assert!(logs_contain("Investigate the epoch"));
    }

    /// The DESK-side half: a cache this build cannot
    /// read because its ENVELOPE VERSION is newer must say "upgrade this desk", NOT
    /// "re-sync it" — re-syncing re-fetches the exact shape that cannot be read, so the
    /// generic corrupt-file remediation sends the operator in a circle.
    ///
    /// Both shapes are exercised through the REAL reader (`resolve_epoch_sync` →
    /// `EpochSyncWire::from_postcard`), so the version arm cannot pass while the real
    /// refusal text drifts. Both keep the same OUTCOME (this desk is not carrying
    /// revocations either way) — it is the remediation that must differ.
    #[test]
    #[tracing_test::traced_test]
    fn an_unreadable_cache_says_upgrade_for_a_version_skew_and_resync_for_corruption() {
        let dir = tempfile::tempdir().unwrap();

        // (a) GENUINE corruption ⇒ "Re-sync the epoch".
        let corrupt = dir.path().join(epoch_cache_file_name_for_test("corrupt"));
        std::fs::write(&corrupt, "not an epoch artifact").unwrap();
        assert_eq!(
            prepare_epoch_push("corrupt", Some(&corrupt), &|| true),
            EpochPushPlan::Skip(EpochPushOutcome::CacheUnreadable)
        );
        assert!(logs_contain("Re-sync the epoch from the account service"));
        assert!(
            !logs_contain("UPGRADE this desk"),
            "a corrupt file must NOT be reported as a version skew"
        );

        // (b) A FUTURE envelope version (a writer ahead of this build) ⇒ "UPGRADE this
        //     desk", and explicitly NOT the epoch's fault.
        let mut art = artifact(4);
        art.version = cerulion_pairing::verify::EPOCH_SYNC_WIRE_VERSION + 1;
        let skewed = dir.path().join(epoch_cache_file_name_for_test("skewed"));
        std::fs::write(&skewed, crate::config::encode_epoch_cache(&art).unwrap()).unwrap();
        assert_eq!(
            prepare_epoch_push("skewed", Some(&skewed), &|| true),
            EpochPushPlan::Skip(EpochPushOutcome::CacheUnreadable),
            "the delivery answer is the same — this desk is not carrying revocations"
        );
        assert!(
            logs_contain("UPGRADE this desk") && logs_contain("the epoch itself is not at fault"),
            "a version-skewed cache must be reported as an upgrade, not a damaged file"
        );
    }

    /// Every variant, hand-listed once — the oracle vector for the three operator-facing
    /// derivations ([`EpochPushOutcome::severity`], [`EpochPushOutcome::robot_is_current`]
    /// and `Display`). A NEW variant added without a row here fails to compile (the
    /// `match` in the vector is exhaustive by construction below), and a variant whose
    /// severity is silently reclassified fails the assert.
    ///
    /// The severity rows are the FLOOD pin. Were both desk paths
    /// to branch on `robot_is_current()` alone, `NoCacheDir` / `NoCachedEpoch` — the
    /// permanent healthy state of a never-synced or guest desk — would WARN on EVERY
    /// connect and EVERY demand. Reclassifying either row to `NotCarried` re-creates that
    /// false-alarm flood and fails here.
    #[test]
    fn severity_display_and_is_current_are_per_variant_oracles() {
        use EpochPushSeverity::*;
        // (outcome, expected severity, the EXACT rendered line)
        let rows: Vec<(EpochPushOutcome, EpochPushSeverity, &str)> = vec![
            (
                EpochPushOutcome::Applied { epoch: 9 },
                Current,
                "DELIVERED — the robot applied epoch 9",
            ),
            (
                EpochPushOutcome::AlreadyCurrent { epoch: 12 },
                Current,
                "already current — the robot is at epoch 12",
            ),
            (
                EpochPushOutcome::NoCacheDir,
                NothingToCarry,
                "nothing pushed — this desk has no revocation-epoch cache directory",
            ),
            (
                EpochPushOutcome::NoCachedEpoch,
                NothingToCarry,
                "nothing pushed — no cached epoch for this robot on this desk",
            ),
            (
                EpochPushOutcome::UnverifiedRobotIdentity,
                NotCarried,
                "nothing pushed — this desk has no pinned name for the robot it dialed, so it \
                 cannot tell which cached epoch is meant (pair/pin it)",
            ),
            (
                EpochPushOutcome::CacheUnreadable,
                NotCarried,
                "NOT delivering revocations — the cached epoch is unreadable (re-sync it)",
            ),
            (
                EpochPushOutcome::CacheTooLarge { bytes: 17_000_000 },
                NotCarried,
                "NOT delivering revocations — the cached epoch is too large to send \
                 (17000000 bytes; re-sync it)",
            ),
            (
                EpochPushOutcome::NoSink,
                NotCarried,
                "NOT delivered — the robot accepts no epoch sync (upgrade/configure it)",
            ),
            (
                EpochPushOutcome::Rejected,
                NotCarried,
                "NOT delivered — the robot REJECTED the epoch (investigate it)",
            ),
            (
                EpochPushOutcome::NotDelivered,
                NotCarried,
                "NOT delivered — the robot's answer was unusable",
            ),
            (
                EpochPushOutcome::TransportFailed,
                NotCarried,
                "NOT delivered — the control stream failed mid-push (a re-dial recovers)",
            ),
        ];
        for (outcome, severity, rendered) in &rows {
            assert_eq!(outcome.severity(), *severity, "severity of {outcome:?}");
            assert_eq!(&outcome.to_string(), rendered, "Display of {outcome:?}");
            // `robot_is_current` is exactly "severity == Current" — one definition, not
            // a second hand-maintained variant list that could drift from it.
            assert_eq!(
                outcome.robot_is_current(),
                *severity == Current,
                "robot_is_current of {outcome:?}"
            );
        }
        // TOTALITY: every variant of the enum is represented above. The `match` is
        // exhaustive, so a new variant makes THIS fail to compile until it is added to
        // `rows` — the structural guard against a variant shipping with no oracle.
        for outcome in &rows {
            let _: () = match outcome.0 {
                EpochPushOutcome::Applied { .. }
                | EpochPushOutcome::AlreadyCurrent { .. }
                | EpochPushOutcome::NoCacheDir
                | EpochPushOutcome::NoCachedEpoch
                | EpochPushOutcome::UnverifiedRobotIdentity
                | EpochPushOutcome::CacheUnreadable
                | EpochPushOutcome::CacheTooLarge { .. }
                | EpochPushOutcome::NoSink
                | EpochPushOutcome::Rejected
                | EpochPushOutcome::NotDelivered
                | EpochPushOutcome::TransportFailed => (),
            };
        }
        assert_eq!(rows.len(), 11, "one row per variant, no duplicates");
        // Exactly TWO outcomes are quiet-healthy, and each is a "this desk holds
        // nothing" state — a hand oracle over the whole table, so promoting a genuine
        // non-delivery into the quiet class (the flood-suppression footgun in reverse)
        // fails here too.
        let quiet: Vec<_> = rows
            .iter()
            .filter(|(_, s, _)| *s == NothingToCarry)
            .map(|(o, _, _)| *o)
            .collect();
        assert_eq!(
            quiet,
            vec![
                EpochPushOutcome::NoCacheDir,
                EpochPushOutcome::NoCachedEpoch
            ]
        );
    }

    /// A robot's `message`/`reason` is FULLY peer-controlled and the desk is where it
    /// becomes operator-visible output: control characters (terminal escapes, CR
    /// overwrites, BEL) are neutered and the text is bounded, so a hostile robot can
    /// neither paint an operator's terminal nor flood the log pipeline. Hand oracles.
    #[test]
    fn peer_text_is_neutered_and_bounded() {
        // C0 controls (ESC/CR/BEL/NUL), DEL, and C1 controls all become U+FFFD;
        // ordinary text (including non-ASCII) is untouched.
        assert_eq!(
            sanitize_peer_text("a\u{1b}[2Jb\rc\u{7}d\0e\u{7f}f\u{9b}g"),
            "a\u{fffd}[2Jb\u{fffd}c\u{fffd}d\u{fffd}e\u{fffd}f\u{fffd}g"
        );
        assert_eq!(sanitize_peer_text("plain — naïve 🤖"), "plain — naïve 🤖");
        assert_eq!(sanitize_peer_text(""), "");
        // Exactly at the ceiling: untouched, no marker.
        let at_cap = "x".repeat(MAX_PEER_TEXT_LEN);
        assert_eq!(sanitize_peer_text(&at_cap), at_cap);
        // One char over: truncated to the ceiling + an EXPLICIT marker (never a silent
        // clip that could be mistaken for the robot's whole message).
        let over = "y".repeat(MAX_PEER_TEXT_LEN + 1);
        let cut = sanitize_peer_text(&over);
        assert_eq!(
            cut,
            format!("{}…(truncated)", "y".repeat(MAX_PEER_TEXT_LEN))
        );
        // A megabyte "error" cannot flood the log.
        assert!(sanitize_peer_text(&"z".repeat(1_000_000)).len() < MAX_PEER_TEXT_LEN + 32);
        // Truncation counts CHARS, not bytes — a multi-byte tail is never split mid
        // character (it is a `String`, so this would panic if it were byte-sliced).
        let multibyte = "é".repeat(MAX_PEER_TEXT_LEN + 10);
        assert!(sanitize_peer_text(&multibyte).starts_with("éé"));
    }

    /// [`sanitize_peer_text`] is IDEMPOTENT: `f(f(s)) == f(s)` for every input.
    ///
    /// This is a LOAD-BEARING property, not a curiosity. The desk sanitizes peer text
    /// at the boundary where robot bytes become a `ConnectError` (connectd's
    /// `classify_catalog_reply` / `classify_demand_reply` / `schema`'s validators) AND
    /// AGAIN at the operator render sites (`cerulion-connectd`'s two `main.rs` error
    /// arms, which render EVERY `ConnectError` variant — including variants no
    /// classifier covers). That belt-and-braces layering is only safe because a second
    /// application is a no-op; if truncation ever became non-idempotent (e.g. by
    /// counting the `…(truncated)` marker toward the ceiling, or by appending a second
    /// marker), the render sites would start mangling already-bounded text.
    ///
    /// The subtle case is the marker itself: the first pass emits
    /// `<512 chars><…(truncated)>` (524 chars), so a second application truncates at the SAME
    /// char boundary — the marker starts exactly at index 512 and is cut away, then
    /// re-appended verbatim. `…`, `(`, `)` and the marker's letters are outside the
    /// neutered C0/DEL/C1 classes, and `U+FFFD` (the neuter output) is too, so the
    /// neuter half is idempotent as well.
    ///
    /// Making the marker count toward `MAX_PEER_TEXT_LEN`, or neutering any
    /// character the sanitizer itself emits, fails this test.
    #[test]
    fn peer_text_sanitizer_is_idempotent() {
        let boundary_cases = [
            String::new(),
            "plain".to_string(),
            // The three truncation boundaries: just under, exactly at, just over.
            "x".repeat(MAX_PEER_TEXT_LEN - 1),
            "x".repeat(MAX_PEER_TEXT_LEN),
            "x".repeat(MAX_PEER_TEXT_LEN + 1),
            // Well over, plus a megabyte flood.
            "x".repeat(MAX_PEER_TEXT_LEN + 8),
            "x".repeat(1_000_000),
            // Hostile control characters, both short and past the ceiling.
            "a\u{1b}[2Jb\rc\u{7}d\0e\u{7f}f\u{9b}g".to_string(),
            format!("x\u{1b}[2J{}", "é".repeat(MAX_PEER_TEXT_LEN + 10)),
            // Text that ALREADY carries the marker (the exact shape a render site
            // re-sanitizes) and text whose tail merely looks like it.
            format!("{}…(truncated)", "y".repeat(MAX_PEER_TEXT_LEN)),
            "short…(truncated)".to_string(),
            // Multi-byte characters straddling the ceiling (char- not byte-indexed).
            "é".repeat(MAX_PEER_TEXT_LEN + 10),
            "🤖".repeat(MAX_PEER_TEXT_LEN + 3),
        ];
        for case in &boundary_cases {
            let once = sanitize_peer_text(case);
            let twice = sanitize_peer_text(&once);
            assert_eq!(
                once,
                twice,
                "sanitize_peer_text must be idempotent (input len {} chars)",
                case.chars().count()
            );
            // And the fixed point really is bounded — a second application cannot grow it.
            assert!(
                twice.chars().count() <= MAX_PEER_TEXT_LEN + "…(truncated)".chars().count(),
                "the fixed point stays bounded"
            );
        }
        // A truncated result is a FIXED POINT at exactly ceiling + marker, and stays
        // marked (re-sanitizing never eats the marker — the hazard the layering rests
        // on).
        let cut = sanitize_peer_text(&"z".repeat(1_000_000));
        assert_eq!(
            cut.chars().count(),
            MAX_PEER_TEXT_LEN + "…(truncated)".chars().count()
        );
        assert!(cut.ends_with("…(truncated)"));
        assert!(sanitize_peer_text(&cut).ends_with("…(truncated)"));
    }
}
