//! The `/__cerulion/flashback` TRIGGER CHANNEL — how something
//! that noticed a moment reaches the recorder that can capture it.
//!
//! # What this is, and what it deliberately is not
//!
//! [`trigger`](super::trigger) decides WHETHER a request
//! becomes a capture; it is pure and holds no transport. This module is the
//! carriage: it moves a [`CaptureRequest`] from an observer's process to the
//! recorder's, and moves the recorder's verdict back.
//!
//! It is modelled BYTE-FOR-BYTE on
//! [`run_registry`](crate::transport::run_registry) — itself modelled on
//! [`mirror_registry`](crate::transport::mirror_registry): the same fixed service name
//! under [`RESERVED_TOPIC_PREFIX`](crate::transport::gateway::RESERVED_TOPIC_PREFIX), the
//! same magic + version + length-prefixed record discipline, and the same
//! writer-and-reader-build-from-ONE-constant rule so their static configs cannot
//! drift. **No new transport concept**, which is exactly why the design note
//! chose it.
//!
//! # Two services, and why the second one is not optional
//!
//! By design, a manual request that hits the rate cap **fails
//! loudly, and the failure must reach the CLI user** — not merely a daemon log
//! the operator cannot see. A fire-and-forget publish cannot do that: the verb
//! would print "asked" and exit 0 while the robot captured nothing. So there are
//! two channels:
//!
//! | service | writer | reader |
//! |---|---|---|
//! | [`FLASHBACK_REQUEST_SERVICE_NAME`] | observers (`cerulion flashback`, the supervisor, monitors) | recorders |
//! | [`FLASHBACK_OUTCOME_SERVICE_NAME`] | recorders | the observer that asked |
//!
//! The outcome record carries [`SuppressReason`] VERBATIM rather than a
//! re-spelled copy, so the numbers an operator is shown are the numbers the gate
//! decided on (the one-vocabulary rule).
//!
//! # A request is BROADCAST, and that is the actual semantic
//!
//! iceoryx2 pub/sub delivers to every subscriber, so a request reaches every
//! recorder on the machine. On the shipping shape that is exactly one (one
//! recorder per run, forced by the state ring's SPSC
//! contract), and with two graphs serving at once "capture the moment"
//! genuinely means both. The requester therefore collects N outcomes rather than
//! one, each naming its recorder, and reports them all: a verb that reported only
//! the first would be silently wrong on the multi-run desk.
//!
//! # Ordering, and the one rule a requester must follow
//!
//! **Open the outcome reader BEFORE publishing the request.** A recorder's
//! verdict is published microseconds after it drains the request, and a
//! subscriber that does not exist yet cannot receive it — the same structural
//! (not timing) unreachability `run_registry`'s `RunWatcher` exists for.
//! [`FlashbackRequester::open`] does this in the right order so a caller cannot
//! get it wrong.

use std::sync::Mutex;
use std::time::{Duration, Instant};

use iceoryx2::node::Node;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::port::subscriber::Subscriber;
use iceoryx2::prelude::ServiceName;
use iceoryx2::service::port_factory::publish_subscribe::PortFactory;

use super::trigger::{CaptureRequest, SuppressReason, TriggerKind};
use crate::error::{TransportError, TransportResult};
use crate::transport::failure_regime_latch::{
    lock_regime_latch, FailureRegimeLatch, RegimeDecision,
};
use crate::transport::CerService;

/// The fixed request service — canonical `/__cerulion/flashback`, under
/// [`RESERVED_TOPIC_PREFIX`](crate::transport::gateway::RESERVED_TOPIC_PREFIX) and
/// deliberately carrying NO `/data` suffix, so `cerulion topic list` (which
/// enumerates `*/data`) never surfaces it and `cerulion_bagd`'s live enumeration
/// never taps it. Both sides build from THIS constant.
pub const FLASHBACK_REQUEST_SERVICE_NAME: &str = "/__cerulion/flashback";

/// The fixed outcome service — the reply half. Same rules as the request half.
pub const FLASHBACK_OUTCOME_SERVICE_NAME: &str = "/__cerulion/flashback/outcome";

/// How many observers may publish requests at once — a `cerulion flashback`
/// invocation, each running supervisor's fault publisher, and a monitors adapter.
/// Generous because an observer is short-lived and a refused port is a lost
/// capture.
pub const FLASHBACK_MAX_REQUESTERS: usize = 32;

/// How many recorders may listen. One per serving graph on the machine.
pub const FLASHBACK_MAX_RESPONDERS: usize = 16;

/// Queue depth on both services. A burst of faults must not evict the request an
/// operator typed.
pub const FLASHBACK_QUEUE_DEPTH: usize = 64;

/// How long a requester waits for the ACCEPTED/SUPPRESSED verdicts.
///
/// Short on purpose: a recorder answers from its own drive loop, so this bounds
/// "is anybody listening?" rather than any real work. A desk with no serving
/// graph spends it once and says so.
pub const FLASHBACK_VERDICT_WINDOW: Duration = Duration::from_millis(1_500);

/// How long past a capture's declared end a requester keeps waiting for the
/// FINISHED verdict before reporting the capture as still running.
///
/// The bag is written from memory the recorder already holds, so this is finalize
/// plus filesystem, not a data-collection window.
pub const FLASHBACK_FINALIZE_GRACE: Duration = Duration::from_secs(20);

/// Poll cadence while waiting. Slices the wait so a Ctrl-C is noticed promptly.
pub const FLASHBACK_POLL_INTERVAL: Duration = Duration::from_millis(50);

/// How often an unanswered request is RE-PUBLISHED under its own id.
///
/// One spelling for one number (the one-vocabulary rule): the `cerulion flashback` verb
/// and every fire-and-forget producer retry on the same cadence, so a robot's
/// behaviour does not depend on which of them asked.
pub const FLASHBACK_REQUEST_RETRY_INTERVAL: Duration = Duration::from_millis(300);

/// How long a FIRE-AND-FORGET producer keeps its end of the channel ALIVE after
/// publishing.
///
/// # This is not politeness — a requester that drops too early DESTROYS its own
/// request
///
/// MEASURED, and it is the reason [`FlashbackRequester::request_and_linger`]
/// exists at all. With a responder already attached and a request published, the
/// request is delivered if the requester is still alive when the recorder drains
/// and is NOT delivered if the requester was dropped first — same thread, same
/// namespace, same ordering, one variable. iceoryx2 reclaims a departing
/// publisher's unread samples, so "publish and return" is a race against the
/// recorder's drive loop that the producer usually loses.
///
/// DERIVED from [`FLASHBACK_VERDICT_WINDOW`] rather than chosen: that constant is
/// already this repo's answer to "how long does it take to find out whether
/// anybody is listening", and a producer that lingered for a different span would
/// be making a second, quieter claim about the same question.
pub const FLASHBACK_UNWATCHED_LINGER: Duration = FLASHBACK_VERDICT_WINDOW;

const MAGIC: [u8; 2] = [0xCE, 0x82];
const VERSION: u8 = 1;

/// `magic(2) + version(1) + request_id(8) + kind(1) + pin(1) + subject_len(2) + detail_len(2)`
const REQUEST_HEADER_LEN: usize = 2 + 1 + 8 + 1 + 1 + 2 + 2;
/// `magic(2) + version(1) + request_id(8) + verdict(1) + seq(8) + a(8) + b(8) + recorder_len(2) + text_len(2)`
const OUTCOME_HEADER_LEN: usize = 2 + 1 + 8 + 1 + 8 + 8 + 8 + 2 + 2;

/// The OPTIONAL trailer a `Finished` verdict may carry —
/// `claimed_span_ms(8) + achieved_span_ms(8) + truncated_frames(8)`, appended
/// AFTER the text.
///
/// # Why a trailer and not two more header fields
///
/// The header is fixed-width and fully spoken for: `Finished` already uses
/// `seq`, `a` (bytes) and `b` (the resimmable tri-state), and widening the header
/// would move every field after it — a change no older reader could survive.
///
/// A trailer is ADDITIVE in the one direction that matters. `decode_outcome`
/// bounds the text at `OUTCOME_HEADER_LEN + recorder_len + text_len` and checks
/// `bytes.len() < end`, so it has always TOLERATED trailing bytes: an older
/// reader handed one of these frames decodes exactly the verdict it always did
/// and ignores what it cannot name. The frames are carried on an exact-length
/// slice service (`loan_slice_uninit(bytes.len())`), so length is a reliable
/// signal — there is no padding for a zeroed trailer to hide in.
///
/// ABSENCE is the tri-state (absent means unknown): an older recorder emits no
/// trailer, which decodes to `None` — "this recorder made no claim" — never to a
/// fabricated span of zero.
const OUTCOME_SPAN_TRAILER_LEN: usize = 8 + 8 + 8;

/// Bound on a request's `subject` — the regime key.
pub const MAX_FLASHBACK_SUBJECT_LEN: usize = 256;
/// Bound on a request's `detail` — the human note carried into the bag.
pub const MAX_FLASHBACK_DETAIL_LEN: usize = 512;
/// Bound on an outcome's `recorder` label.
pub const MAX_FLASHBACK_RECORDER_LEN: usize = 256;
/// Bound on an outcome's path-or-reason text.
pub const MAX_FLASHBACK_TEXT_LEN: usize = 1024;

/// The largest frame either service carries.
pub const MAX_FLASHBACK_RECORD_LEN: usize = OUTCOME_HEADER_LEN
    + OUTCOME_SPAN_TRAILER_LEN
    + MAX_FLASHBACK_RECORDER_LEN
    + MAX_FLASHBACK_TEXT_LEN
    + REQUEST_HEADER_LEN
    + MAX_FLASHBACK_SUBJECT_LEN
    + MAX_FLASHBACK_DETAIL_LEN;

/// Why a frame could not be built or read.
///
/// Named arms rather than a bare `None`, because "somebody sent garbage" and
/// "this subject is too long" need different remedies and only one of them is the
/// caller's fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashbackRecordError {
    /// Not a flashback frame at all.
    BadMagic,
    /// A flashback frame from a version this build does not speak.
    BadVersion(u8),
    /// The frame is shorter than its own header claims.
    Truncated,
    /// A declared length runs past the end of the frame.
    LengthOverflow,
    /// A text field is not UTF-8.
    NotUtf8,
    /// A field exceeds its documented bound.
    TooLong,
    /// The verdict byte names no verdict this build knows.
    UnknownVerdict(u8),
    /// The kind byte names no trigger kind this build knows.
    UnknownKind(u8),
    /// The switch byte names no posture switch this build knows.
    UnknownSwitch(u8),
}

impl std::fmt::Display for FlashbackRecordError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BadMagic => write!(f, "not a flashback record (magic mismatch)"),
            Self::BadVersion(v) => write!(f, "flashback record version {v} is not understood"),
            Self::Truncated => write!(f, "flashback record is shorter than its header"),
            Self::LengthOverflow => write!(f, "a declared length runs past the end of the record"),
            Self::NotUtf8 => write!(f, "a flashback record field is not valid UTF-8"),
            Self::TooLong => write!(f, "a flashback record field exceeds its documented bound"),
            Self::UnknownVerdict(v) => write!(f, "unknown flashback verdict byte {v}"),
            Self::UnknownKind(k) => write!(f, "unknown flashback trigger kind byte {k}"),
            Self::UnknownSwitch(s) => write!(f, "unknown flashback trigger switch byte {s}"),
        }
    }
}

/// The kind's wire byte.
///
/// # Bytes are APPENDED and never re-used
///
/// A byte that ever meant one kind must never come to mean another: the frame
/// carries no version discriminator per field and no checksum over the kind, so a
/// re-used byte is silently mis-read as whatever the reader's build thinks it
/// means, and the difference — a `declared` physical event read as a `process_fault`
/// — is exactly the attribution the vocabulary exists to preserve. Adding a kind
/// therefore takes the next free byte; retiring one leaves its byte permanently
/// spent (and the decode arm removed, so an old producer's frame is REFUSED rather
/// than re-interpreted).
fn kind_to_wire(kind: TriggerKind) -> u8 {
    match kind {
        TriggerKind::Manual => 1,
        TriggerKind::ProcessFault => 2,
        TriggerKind::MonitorVerdict => 3,
        // Kinds added after the first three.
        TriggerKind::PanicDisable => 4,
        TriggerKind::RunVanished => 5,
        TriggerKind::EStop => 6,
        TriggerKind::Declared => 7,
    }
}

fn kind_from_wire(byte: u8) -> Option<TriggerKind> {
    match byte {
        1 => Some(TriggerKind::Manual),
        2 => Some(TriggerKind::ProcessFault),
        3 => Some(TriggerKind::MonitorVerdict),
        4 => Some(TriggerKind::PanicDisable),
        5 => Some(TriggerKind::RunVanished),
        6 => Some(TriggerKind::EStop),
        7 => Some(TriggerKind::Declared),
        _ => None,
    }
}

/// A request on the wire — a [`CaptureRequest`] plus the id its outcomes carry
/// back.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlashbackRequestFrame {
    /// Minted by the observer; every outcome for this request repeats it, which
    /// is how a requester tells its own verdicts from a concurrent operator's.
    pub request_id: u64,
    /// The request the gate will decide on.
    pub request: CaptureRequest,
}

/// PURE: encode a request frame.
pub fn encode_request(frame: &FlashbackRequestFrame) -> Result<Vec<u8>, FlashbackRecordError> {
    let subject = frame.request.subject.as_bytes();
    let detail = frame.request.detail.as_bytes();
    if subject.len() > MAX_FLASHBACK_SUBJECT_LEN || detail.len() > MAX_FLASHBACK_DETAIL_LEN {
        return Err(FlashbackRecordError::TooLong);
    }
    let mut out = Vec::with_capacity(REQUEST_HEADER_LEN + subject.len() + detail.len());
    out.extend_from_slice(&MAGIC);
    out.push(VERSION);
    out.extend_from_slice(&frame.request_id.to_le_bytes());
    out.push(kind_to_wire(frame.request.kind));
    out.push(u8::from(frame.request.pin));
    out.extend_from_slice(&(subject.len() as u16).to_le_bytes());
    out.extend_from_slice(&(detail.len() as u16).to_le_bytes());
    out.extend_from_slice(subject);
    out.extend_from_slice(detail);
    Ok(out)
}

/// PURE: decode a request frame — `encode_request`'s exact inverse.
pub fn decode_request(bytes: &[u8]) -> Result<FlashbackRequestFrame, FlashbackRecordError> {
    if bytes.len() < REQUEST_HEADER_LEN {
        return Err(FlashbackRecordError::Truncated);
    }
    if bytes[0..2] != MAGIC {
        return Err(FlashbackRecordError::BadMagic);
    }
    if bytes[2] != VERSION {
        return Err(FlashbackRecordError::BadVersion(bytes[2]));
    }
    let request_id = u64::from_le_bytes(bytes[3..11].try_into().expect("8 bytes"));
    let kind = kind_from_wire(bytes[11]).ok_or(FlashbackRecordError::UnknownKind(bytes[11]))?;
    let pin = bytes[12] != 0;
    let subject_len = u16::from_le_bytes(bytes[13..15].try_into().expect("2 bytes")) as usize;
    let detail_len = u16::from_le_bytes(bytes[15..17].try_into().expect("2 bytes")) as usize;
    if subject_len > MAX_FLASHBACK_SUBJECT_LEN || detail_len > MAX_FLASHBACK_DETAIL_LEN {
        return Err(FlashbackRecordError::TooLong);
    }
    let end = REQUEST_HEADER_LEN
        .checked_add(subject_len)
        .and_then(|v| v.checked_add(detail_len))
        .ok_or(FlashbackRecordError::LengthOverflow)?;
    if bytes.len() < end {
        return Err(FlashbackRecordError::LengthOverflow);
    }
    let subject_start = REQUEST_HEADER_LEN;
    let detail_start = subject_start + subject_len;
    let subject = std::str::from_utf8(&bytes[subject_start..detail_start])
        .map_err(|_| FlashbackRecordError::NotUtf8)?
        .to_string();
    let detail = std::str::from_utf8(&bytes[detail_start..end])
        .map_err(|_| FlashbackRecordError::NotUtf8)?
        .to_string();
    Ok(FlashbackRequestFrame {
        request_id,
        request: CaptureRequest {
            kind,
            subject,
            detail,
            pin,
        },
    })
}

/// What a finished capture COVERS, and the evidence for why.
///
/// ONE struct rather than three `Option<u64>`s on the variant, because they are
/// one measurement: they arrive in one trailer, and a verdict carrying a claim
/// without the achievement — or the reverse — describes nothing. Made
/// unrepresentable rather than guarded at each reader.
///
/// The SHORTFALL is deliberately NOT a field: it is
/// `claimed.saturating_sub(achieved)` and nothing else, and a carried difference
/// that can disagree with the two numbers it is a difference of is worse than no
/// difference at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FinishedSpan {
    /// What the capture's own endpoints CLAIM — `ended − floor`, never the span
    /// constant (an early trigger's floor saturates, so the constant would
    /// advertise a look-back the run had not existed long enough to have).
    pub claimed_span_ms: u64,
    /// What its frames actually reach — `ended − achieved_from`, or 0 when it
    /// carries no frames at all.
    pub achieved_span_ms: u64,
    /// Frames THIS capture lost to the window's byte ceiling.
    ///
    /// # Why a shortfall does not imply eviction
    ///
    /// It rides here because it is the only thing that can tell an operator WHY
    /// a capture covers less than it claims, and the two are genuinely
    /// independent: a capture triggered inside its first span, one on a robot
    /// whose topics are sparse, or one over an interval nobody published in, all
    /// carry a positive shortfall with NOTHING evicted. Attributing those to the
    /// byte ceiling sends the operator to raise a cap that never bound — so a
    /// renderer states the measured shortfall NEUTRALLY and reaches for the
    /// causal sentence only when this number is above zero.
    pub truncated_frames: u64,
}

impl FinishedSpan {
    /// How much of the claim is missing from the bag.
    pub fn shortfall_ms(&self) -> u64 {
        self.claimed_span_ms.saturating_sub(self.achieved_span_ms)
    }

    /// Whether the byte ceiling is EVIDENCED as (part of) the cause.
    ///
    /// A renderer must not say "the ceiling evicted it" without this — see
    /// [`truncated_frames`](Self::truncated_frames).
    pub fn evicted_during_capture(&self) -> bool {
        self.truncated_frames > 0
    }
}

/// What a recorder decided, and later, what it produced.
///
/// [`Self::Suppressed`] carries [`SuppressReason`] VERBATIM — the same type the
/// gate returned — so the numbers an operator reads are the numbers the decision
/// was made on, rather than a second spelling that can drift (the
/// one-vocabulary rule).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlashbackOutcome {
    /// A capture STARTED for this request.
    Accepted {
        /// The capture's sequence number within its recorder.
        seq: u64,
        /// Milliseconds until it stops recording (it may be extended).
        ends_in_ms: u64,
        /// Where it will land.
        path: String,
    },
    /// A capture was ALREADY recording and this request joined it.
    Extended {
        /// The capture it joined.
        seq: u64,
        /// Milliseconds until it stops recording, after this extension.
        ends_in_ms: u64,
        /// Where it will land.
        path: String,
    },
    /// The gate refused, and why — with the numbers.
    Suppressed(SuppressReason),
    /// The capture finished and the bag is on disk.
    Finished {
        /// The capture's sequence number.
        seq: u64,
        /// The bag's size.
        bytes: u64,
        /// Where it landed.
        path: String,
        /// Whether `cerulion bag play <bag> --resim all` will
        /// accept this capture, as
        /// [`judge_resimmable`](crate::flashback::resim::judge_resimmable) decided.
        ///
        /// `None` is UNKNOWN and is a real answer, not a placeholder: an older
        /// recorder makes no such claim, and the two must not be
        /// conflated — "we did not say" is not "no".
        ///
        /// # Why the verb needs it at all
        ///
        /// The verdict was written into `__cerulion/flashback.json` and NOWHERE
        /// ELSE, so an operator who ran `cerulion flashback` during an incident
        /// was told where the bag landed and had to open it to learn whether it
        /// could be resumed — which is the one thing they would want to know
        /// before the moment passed.
        ///
        /// # Wire shape
        ///
        /// It rides the `b` field, which `Finished` has always sent as 0 — a
        /// change in MEANING only, within a version this build already speaks
        /// (the `Abandoned`/`Failed` precedent). 0 stays UNKNOWN precisely so an
        /// older recorder's zero decodes to `None` rather than to a confident
        /// "not resimmable" nobody claimed.
        resimmable: Option<bool>,
        /// What this capture CLAIMS to cover and what it actually
        /// carries, so `cerulion flashback` can print "achieved 2.0 s of the
        /// claimed 45.0 s" when the capture is inspected.
        ///
        /// `None` is UNKNOWN and is a real answer: an older recorder sends
        /// no trailer and makes no such claim, which is not the same as a
        /// capture that covered nothing.
        span: Option<FinishedSpan>,
    },
    /// The recorder STOPPED WAITING for a capture that was still being written.
    ///
    /// # Why this is not [`Self::Failed`]
    ///
    /// That would be a claim the recorder cannot support.
    /// A shutdown's wait is bounded, but a writer that
    /// merely ran LATE finishes a moment after the bound and leaves a complete,
    /// finalized bag on disk — while the requester would already have been told the
    /// capture FAILED and the verb would already have exited nonzero. A verdict that
    /// contradicts the disk is worse than no verdict.
    ///
    /// So this says exactly what is known: the recorder stopped waiting, the bag
    /// may be complete or may be torn, and the path is where to look. It SETTLES
    /// the acceptance (so nothing waits out a deadline for a verdict that will
    /// never come) without asserting an outcome nobody observed.
    Abandoned {
        /// WHICH capture.
        ///
        /// NOT optional, unlike [`Failed`](Self::Failed)'s. Only an
        /// ACCEPTED capture can be abandoned — the recorder abandons a WRITER,
        /// and a writer exists only for a capture it opened — so a sequence-less
        /// `Abandoned` describes nothing that can happen. Permitting one was not
        /// harmless: a consumer keys outstanding captures on `(recorder, seq)`,
        /// so a frame with no sequence classified as "a capture happened" while
        /// belonging to no capture, and could be reported without ever reaching
        /// the accounting that decides the exit code. Made unrepresentable rather
        /// than guarded at each reader.
        seq: u64,
        /// What happened and where to look.
        reason: String,
    },
    /// The recorder accepted the request, ran the
    /// capture, and REFUSED to finalize it because it could not resim.
    ///
    /// # Its own verdict, not a `Failed` and not a `Suppressed`
    ///
    /// Three states, three verdicts, and collapsing any two of them tells an
    /// operator to do the wrong thing:
    ///
    /// * [`Suppressed`](Self::Suppressed) is the anti-spam gate declining BEFORE
    ///   a capture starts — "not now, try later".
    /// * [`Failed`](Self::Failed) is the recorder breaking — a disk error, a
    ///   thread that would not start. Something is wrong with the machine.
    /// * This is the POLICY working on a machine that is simply too small: the
    ///   plane cannot hold one whole checkpoint generation, so any bag it wrote
    ///   would be a dashcam clip wearing the Flashback name. The project forbids
    ///   that fallback outright, so the capture is refused and the operator is
    ///   told the one knob that fixes it.
    ///
    /// Reported through `Failed` it would read as a broken robot; through
    /// `Suppressed` it would read as "try later", which will never work. And a
    /// consumer counting `Failed`-with-a-sequence as "a capture happened" would
    /// report a bag that does not exist.
    ///
    /// # Wire compatibility, stated
    ///
    /// This is verdict byte 10, which an older reader rejects as
    /// [`FlashbackRecordError::UnknownVerdict`] and drops with a `debug!`. Such a
    /// requester waits out its own deadline instead of hearing the refusal. The
    /// recorder and the CLI ship from one binary, so the skew needs a desk-side
    /// `cerulion` older than the robot's recorder; it is a missing verdict, never
    /// a wrong one.
    Refused {
        /// WHICH capture. NOT optional, on `Abandoned`'s reasoning: only an
        /// ACCEPTED capture can be refused at finalize.
        seq: u64,
        /// Why, and the exact knob that fixes it.
        reason: String,
    },
    /// The recorder accepted the request and then could not produce a bag.
    ///
    /// Distinct from [`Self::Suppressed`]: a suppression is the POLICY working,
    /// while this is the recorder failing, and an operator needs to tell them
    /// apart without reading prose.
    Failed {
        /// WHICH capture failed, when one had been accepted.
        ///
        /// Load-bearing rather than informational: a
        /// requester tracks the captures it is waiting for by `(recorder, seq)`,
        /// so a failure with no sequence can never SETTLE the acceptance it
        /// belongs to — the command waited out its whole deadline and then
        /// printed that no bag was reported, having already been told, in so many
        /// words, that the bag failed.
        ///
        /// `None` only for a failure that never got as far as a capture (a
        /// request refused while a previous capture was still being written).
        /// Such a request has no acceptance to settle, which is exactly what the
        /// absence says.
        seq: Option<u64>,
        /// What went wrong.
        reason: String,
    },
}

/// One recorder's answer about one request.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlashbackOutcomeFrame {
    /// The request this answers.
    pub request_id: u64,
    /// Which recorder answered — the graph or run label, so an operator on a
    /// two-graph desk can tell the answers apart.
    pub recorder: String,
    /// The verdict.
    pub outcome: FlashbackOutcome,
}

/// PURE: encode an outcome frame.
pub fn encode_outcome(frame: &FlashbackOutcomeFrame) -> Result<Vec<u8>, FlashbackRecordError> {
    // The optional trailer, read off the ONE arm that can carry it.
    // Kept apart from the tuple below rather than widened into it, because it is
    // the only field whose PRESENCE is part of the encoding.
    let span = match &frame.outcome {
        FlashbackOutcome::Finished { span, .. } => *span,
        _ => None,
    };
    // The `(verdict, seq, a, b, text)` shape exists ONLY between these two
    // functions: `decode_outcome` reassembles the enum immediately, so no call
    // site ever sees an unlabelled pair of numbers.
    let (verdict, seq, a, b, text): (u8, u64, u64, u64, &str) = match &frame.outcome {
        FlashbackOutcome::Accepted {
            seq,
            ends_in_ms,
            path,
        } => (1, *seq, *ends_in_ms, 0, path.as_str()),
        FlashbackOutcome::Extended {
            seq,
            ends_in_ms,
            path,
        } => (2, *seq, *ends_in_ms, 0, path.as_str()),
        FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed }) => {
            (3, 0, *suppressed, 0, "")
        }
        FlashbackOutcome::Suppressed(SuppressReason::Refractory { retry_in_ns }) => {
            (4, 0, *retry_in_ns, 0, "")
        }
        // `seq` carries the manual reserve. The
        // `(verdict, seq, a, b, text)` tuple is an encoding internal to this
        // pair of functions — no call site ever sees it — and `seq` was already
        // a hard zero on every suppression arm, so borrowing it costs nothing and
        // adds no field to the frame. It is also BACK-COMPATIBLE in the direction
        // that matters: an older reader ignores `seq` on a verdict-5 frame and
        // reconstructs exactly the two numbers it always did.
        FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
            captures_in_window,
            cap,
            reserved_for_manual,
        }) => (
            5,
            u64::from(*reserved_for_manual),
            u64::from(*captures_in_window),
            u64::from(*cap),
            "",
        ),
        // # Verdict 9 carries NO back-compat story, and that is REACHABILITY
        // rather than an oversight (adjudicated)
        //
        // An older requester decoding this hits `UnknownVerdict(9)` and
        // loses the whole outcome — the path, the reason, everything — rather
        // than reading a suppression it does not understand. That would matter if
        // any such requester could RECEIVE one, and none can:
        //
        // - The ONLY shipped requester on this channel is the `cerulion
        //   flashback` verb, which sends `TriggerKind::Manual`.
        // - Manual has NO POSTURE SWITCH. This is by design (see
        //   `TriggerSwitch::for_kind` — an env var that silently disabled a verb
        //   an operator typed and waited on would be a verb that lies about
        //   having run). `refusing_switch` therefore answers `None` for it
        //   ALWAYS, so the gate cannot produce this verdict for a manual
        //   request whatever the environment says.
        // - ONE switchable kind IS published from another process: the graph
        //   supervisor's `publish_process_fault` (a dying worker must be
        //   captured even though nobody is watching). That requester is
        //   FIRE-AND-FORGET by design — `request()` publishes and returns, and
        //   the fault path never opens the outcome subscriber, so it calls NO
        //   decoder and `UnknownVerdict(9)` is unreachable there in any build.
        // - A verb whose subscriber happens to RECEIVE a verdict-9 frame
        //   addressed to such a fault request (two requesters share the outcome
        //   topic) drops it and keeps draining: `drain_outcomes`' decode-error
        //   arm is drop-with-a-debug-line, never an abort, so a foreign verdict
        //   this build does not speak cannot wedge or misreport an old verb's
        //   own wait.
        //
        // So there is no reachable old-requester/new-recorder pair. WHEN THAT
        // STOPS BEING TRUE — the first time something outside this build's
        // process image publishes a switchable kind AND WAITS ON THE ANSWER —
        // the verdict needs a compat story THEN, and this is the place to
        // write it.
        FlashbackOutcome::Suppressed(SuppressReason::Disabled { switch }) => {
            (9, 0, u64::from(switch.as_wire_byte()), 0, "")
        }
        // The resimmable verdict rides `b` as a TRI-STATE, so
        // an older recorder's 0 decodes to UNKNOWN rather than to a "no" it
        // never made.
        FlashbackOutcome::Finished {
            seq,
            bytes,
            path,
            resimmable,
            // Read above, because its presence changes the frame's
            // LENGTH rather than one of its fixed fields.
            span: _,
        } => (
            6,
            *seq,
            *bytes,
            match resimmable {
                None => 0,
                Some(false) => 1,
                Some(true) => 2,
            },
            path.as_str(),
        ),
        // The sequence rides the field every other verdict already uses, and
        // `b` distinguishes "no capture" from "capture 0" — so this is a wire
        // change in MEANING only, within a version this build already speaks.
        FlashbackOutcome::Abandoned { seq, reason } => (8, *seq, 0, 1, reason.as_str()),
        // Verdict 10. `b` is 1 for the same reason `Abandoned`'s is —
        // the sequence is always present, so the flag says "this names a
        // capture" rather than leaving a reader to infer it from a zero.
        FlashbackOutcome::Refused { seq, reason } => (10, *seq, 0, 1, reason.as_str()),
        FlashbackOutcome::Failed { seq, reason } => (
            7,
            seq.unwrap_or(0),
            0,
            u64::from(seq.is_some()),
            reason.as_str(),
        ),
    };
    let recorder = frame.recorder.as_bytes();
    let text = text.as_bytes();
    if recorder.len() > MAX_FLASHBACK_RECORDER_LEN || text.len() > MAX_FLASHBACK_TEXT_LEN {
        return Err(FlashbackRecordError::TooLong);
    }
    let mut out = Vec::with_capacity(OUTCOME_HEADER_LEN + recorder.len() + text.len());
    out.extend_from_slice(&MAGIC);
    out.push(VERSION);
    out.extend_from_slice(&frame.request_id.to_le_bytes());
    out.push(verdict);
    out.extend_from_slice(&seq.to_le_bytes());
    out.extend_from_slice(&a.to_le_bytes());
    out.extend_from_slice(&b.to_le_bytes());
    out.extend_from_slice(&(recorder.len() as u16).to_le_bytes());
    out.extend_from_slice(&(text.len() as u16).to_le_bytes());
    out.extend_from_slice(recorder);
    out.extend_from_slice(text);
    // The optional trailer, LAST — see `OUTCOME_SPAN_TRAILER_LEN`. A
    // recorder with nothing to say appends nothing, so its frame is byte-identical
    // to a pre-trailer one.
    if let Some(span) = span {
        out.extend_from_slice(&span.claimed_span_ms.to_le_bytes());
        out.extend_from_slice(&span.achieved_span_ms.to_le_bytes());
        out.extend_from_slice(&span.truncated_frames.to_le_bytes());
    }
    Ok(out)
}

/// PURE: decode an outcome frame — `encode_outcome`'s exact inverse.
pub fn decode_outcome(bytes: &[u8]) -> Result<FlashbackOutcomeFrame, FlashbackRecordError> {
    if bytes.len() < OUTCOME_HEADER_LEN {
        return Err(FlashbackRecordError::Truncated);
    }
    if bytes[0..2] != MAGIC {
        return Err(FlashbackRecordError::BadMagic);
    }
    if bytes[2] != VERSION {
        return Err(FlashbackRecordError::BadVersion(bytes[2]));
    }
    let request_id = u64::from_le_bytes(bytes[3..11].try_into().expect("8 bytes"));
    let verdict = bytes[11];
    let seq = u64::from_le_bytes(bytes[12..20].try_into().expect("8 bytes"));
    let a = u64::from_le_bytes(bytes[20..28].try_into().expect("8 bytes"));
    let b = u64::from_le_bytes(bytes[28..36].try_into().expect("8 bytes"));
    let recorder_len = u16::from_le_bytes(bytes[36..38].try_into().expect("2 bytes")) as usize;
    let text_len = u16::from_le_bytes(bytes[38..40].try_into().expect("2 bytes")) as usize;
    if recorder_len > MAX_FLASHBACK_RECORDER_LEN || text_len > MAX_FLASHBACK_TEXT_LEN {
        return Err(FlashbackRecordError::TooLong);
    }
    let end = OUTCOME_HEADER_LEN
        .checked_add(recorder_len)
        .and_then(|v| v.checked_add(text_len))
        .ok_or(FlashbackRecordError::LengthOverflow)?;
    if bytes.len() < end {
        return Err(FlashbackRecordError::LengthOverflow);
    }
    let recorder_start = OUTCOME_HEADER_LEN;
    let text_start = recorder_start + recorder_len;
    let recorder = std::str::from_utf8(&bytes[recorder_start..text_start])
        .map_err(|_| FlashbackRecordError::NotUtf8)?
        .to_string();
    let text = std::str::from_utf8(&bytes[text_start..end])
        .map_err(|_| FlashbackRecordError::NotUtf8)?
        .to_string();
    // The OPTIONAL span trailer. Keyed on LENGTH, which is reliable
    // here because these frames ride an exact-length slice service — the
    // publisher loans `bytes.len()`, so there is no padding a zeroed trailer
    // could hide in. A frame that carries no trailer answers `None`, which is
    // "this recorder made no claim" and never a fabricated span of zero.
    //
    // # The two residuals are OPPOSITE cases, and only one of them is legacy
    //
    // A PARTIAL trailer (1..TRAILER_LEN bytes past the text) is a TORN or
    // corrupt frame and is REFUSED. Reading it as a pre-trailer frame would
    // hide corruption behind back-compat — the sender demonstrably tried to
    // append something and the frame does not carry all of it, which is a
    // different fact from a sender that appended nothing. `drain_outcomes`
    // drops an unreadable frame with a debug line, so a torn frame costs ONE
    // frame and never the drain.
    //
    // EXTRA bytes past a WHOLE trailer are TOLERATED, and that is the
    // forward-compat mechanism this trailer itself used to land: an older
    // reader ignores a newer writer's next additive trailer, exactly as a
    // pre-trailer reader ignores this one. Rejecting them would forbid the
    // next extension and re-create the problem the discipline solves — so this
    // reads the FIRST `OUTCOME_SPAN_TRAILER_LEN` bytes and ignores the rest.
    //
    // The WHOLE lengths this reader accepts are 0 and the CURRENT trailer —
    // deliberately not any intermediate width the trailer passed through during
    // development (a 16-byte form existed only in unreleased
    // builds; no released writer ever emitted it, and accepting it
    // would let a CURRENT trailer torn at that boundary decode as a "legacy"
    // span with a fabricated zero — the exact corruption-as-back-compat this
    // check refuses). The rule a FUTURE widening must follow: it accepts every
    // whole length a RELEASED writer ever emitted, and nothing else.
    let residual = bytes.len() - end;
    if residual > 0 && residual < OUTCOME_SPAN_TRAILER_LEN {
        return Err(FlashbackRecordError::Truncated);
    }
    let span = bytes
        .get(end..end + OUTCOME_SPAN_TRAILER_LEN)
        .map(|t| FinishedSpan {
            claimed_span_ms: u64::from_le_bytes(t[0..8].try_into().expect("8 bytes")),
            achieved_span_ms: u64::from_le_bytes(t[8..16].try_into().expect("8 bytes")),
            truncated_frames: u64::from_le_bytes(t[16..24].try_into().expect("8 bytes")),
        });
    let outcome = match verdict {
        1 => FlashbackOutcome::Accepted {
            seq,
            ends_in_ms: a,
            path: text,
        },
        2 => FlashbackOutcome::Extended {
            seq,
            ends_in_ms: a,
            path: text,
        },
        3 => FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed: a }),
        4 => FlashbackOutcome::Suppressed(SuppressReason::Refractory { retry_in_ns: a }),
        5 => FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
            captures_in_window: u32::try_from(a).unwrap_or(u32::MAX),
            cap: u32::try_from(b).unwrap_or(u32::MAX),
            // From `seq` — see `encode_outcome`. A recorder with no manual reserve sends a zero
            // here, which decodes to "no reserve was in force", and that is the
            // TRUE statement about a recorder that had none.
            reserved_for_manual: u32::try_from(seq).unwrap_or(u32::MAX),
        }),
        6 => FlashbackOutcome::Finished {
            seq,
            bytes: a,
            path: text,
            // An UNRECOGNISED code reads UNKNOWN rather than being refused: a
            // newer recorder adding a verdict this build cannot name must not
            // make the whole outcome — the path, the size — unreadable.
            resimmable: match b {
                1 => Some(false),
                2 => Some(true),
                _ => None,
            },
            span,
        },
        7 => FlashbackOutcome::Failed {
            seq: (b != 0).then_some(seq),
            reason: text,
        },
        8 => FlashbackOutcome::Abandoned { seq, reason: text },
        10 => FlashbackOutcome::Refused { seq, reason: text },
        9 => FlashbackOutcome::Suppressed(SuppressReason::Disabled {
            // REFUSED rather than guessed. A switch byte this build cannot name
            // would otherwise have to be rendered as some other switch, and the
            // whole content of this verdict is WHICH variable an operator must go
            // and change — a wrong one sends them to edit a knob that is not the
            // one refusing.
            switch: crate::flashback::switch::TriggerSwitch::from_wire_byte(
                u8::try_from(a).unwrap_or(u8::MAX),
            )
            .ok_or(FlashbackRecordError::UnknownSwitch(
                u8::try_from(a).unwrap_or(u8::MAX),
            ))?,
        }),
        other => return Err(FlashbackRecordError::UnknownVerdict(other)),
    };
    Ok(FlashbackOutcomeFrame {
        request_id,
        recorder,
        outcome,
    })
}

/// Mint an id that no concurrent requester on this machine will repeat.
///
/// Same construction as `run_registry::mint_writer_id` and for the same reason:
/// pid + a nanosecond stamp + a process-global sequence, so two `cerulion
/// flashback` invocations racing on one desk cannot collide and read each
/// other's verdicts.
pub fn mint_request_id() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static SEQ: AtomicU64 = AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, Ordering::Relaxed);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    let pid = std::process::id() as u64;
    pid.wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ nanos.rotate_left(23)
        ^ seq.wrapping_mul(0xD1B5_4A32_D192_ED03).rotate_left(37)
}

/// Mint the transient node a channel end publishes/subscribes on.
///
/// Mirrors `run_registry::build_registry_node` exactly, and for the same
/// reason: `cerulion_bagd` is explicit in its `Cargo.toml` about naming no
/// iceoryx2 type, and the `cerulion flashback` verb should not have to either.
/// Both ends therefore reach this module through
/// [`FlashbackResponder::open_on_manager`] / [`FlashbackRequester::open_on_manager`]
/// and never see a `Node`.
fn build_channel_node(
    config: &iceoryx2::config::Config,
    role: &str,
) -> TransportResult<Node<CerService>> {
    crate::iceoryx_logger::init_iceoryx_log_level_from_env();
    let config = crate::transport::disable_auto_dead_node_cleanup(config.clone());
    let node_name: iceoryx2::prelude::NodeName =
        role.try_into().map_err(|e| TransportError::Internal {
            reason: format!("flashback channel: invalid node name '{role}': {e:?}"),
        })?;
    iceoryx2::prelude::NodeBuilder::new()
        .name(&node_name)
        .config(&config)
        .create::<CerService>()
        .map_err(|e| TransportError::Internal {
            reason: format!("flashback channel: could not create the '{role}' node: {e:?}"),
        })
}

fn open_service(
    node: &Node<CerService>,
    name: &'static str,
    max_publishers: usize,
    max_subscribers: usize,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    let service: ServiceName = name.try_into().map_err(|e| TransportError::Internal {
        reason: format!("flashback channel: invalid service name '{name}': {e:?}"),
    })?;
    node.service_builder(&service)
        .publish_subscribe::<[u8]>()
        .subscriber_max_buffer_size(FLASHBACK_QUEUE_DEPTH)
        .max_subscribers(max_subscribers)
        .max_publishers(max_publishers)
        .open_or_create()
        .map_err(|e| TransportError::Internal {
            reason: format!(
                "flashback channel: could not open the control service '{name}': {e:?}"
            ),
        })
}

fn open_request_service(
    node: &Node<CerService>,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    open_service(
        node,
        FLASHBACK_REQUEST_SERVICE_NAME,
        FLASHBACK_MAX_REQUESTERS,
        FLASHBACK_MAX_RESPONDERS,
    )
}

fn open_outcome_service(
    node: &Node<CerService>,
) -> TransportResult<PortFactory<CerService, [u8], ()>> {
    open_service(
        node,
        FLASHBACK_OUTCOME_SERVICE_NAME,
        // The roles are REVERSED on the reply half: recorders write, observers
        // read. Deriving both caps from the same two constants keeps them in
        // step — a channel whose reply half could hold fewer recorders than its
        // request half would refuse exactly the verdict an operator is waiting
        // for.
        FLASHBACK_MAX_RESPONDERS,
        FLASHBACK_MAX_REQUESTERS,
    )
}

/// The RECORDER's end: read requests, write verdicts.
pub struct FlashbackResponder {
    requests: Subscriber<CerService, [u8], ()>,
    outcomes: Publisher<CerService, [u8], ()>,
    /// The label every outcome this recorder publishes carries.
    recorder: String,
    /// Kept alive for the ports' lifetime when this end minted its own node.
    _node: Option<Node<CerService>>,
    /// Flood suppression for unreadable REQUEST frames.
    ///
    /// Behind a `Mutex` because [`Self::drain_requests`] takes `&self` — the
    /// same reason the transport-side latches do. A poisoned latch is recovered
    /// rather than propagated (`lock_regime_latch`): a diagnostic must never
    /// wedge the drain it observes.
    request_decode_latch: Mutex<FailureRegimeLatch>,
}

impl FlashbackResponder {
    /// Attach a recorder to the channel on the namespace `mgr` runs on.
    ///
    /// The entry point for a consumer that must not name an iceoryx2 type. It is
    /// also the only CORRECT one for a recorder: a responder on a different SHM
    /// root hears no requests at all, so the channel must bind to the namespace
    /// the recorder taps on.
    ///
    /// # Errors
    ///
    /// As [`FlashbackResponder::open`].
    pub fn open_on_manager(
        mgr: &crate::transport::TransportManager,
        recorder: impl Into<String>,
    ) -> TransportResult<Self> {
        Self::open_on_config(&mgr.iox_config(), recorder)
    }

    /// [`FlashbackResponder::open_on_manager`] against an explicit config.
    ///
    /// # Errors
    ///
    /// As [`FlashbackResponder::open`].
    pub fn open_on_config(
        config: &iceoryx2::config::Config,
        recorder: impl Into<String>,
    ) -> TransportResult<Self> {
        let node = build_channel_node(config, "cerulion-flashback-recorder")?;
        let mut this = Self::open(&node, recorder)?;
        this._node = Some(node);
        Ok(this)
    }

    /// Attach a recorder to the channel.
    ///
    /// `recorder` is the label an operator sees beside each verdict — the graph
    /// name, so a two-graph desk's answers are tellable apart.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if either service or either port cannot be created.
    pub fn open(node: &Node<CerService>, recorder: impl Into<String>) -> TransportResult<Self> {
        let request_service = open_request_service(node)?;
        let outcome_service = open_outcome_service(node)?;
        let requests = request_service.subscriber_builder().create().map_err(|e| {
            TransportError::Internal {
                reason: format!(
                    "flashback channel: could not create the request subscriber: {e:?}"
                ),
            }
        })?;
        let outcomes = outcome_service
            .publisher_builder()
            // The slice ceiling a loan is checked against. Without it iceoryx2's
            // default is a single element, so EVERY publish fails
            // `ExceedsMaxLoanSize` — the same reason `run_registry` sets one.
            .initial_max_slice_len(MAX_FLASHBACK_RECORD_LEN)
            .create()
            .map_err(|e| TransportError::Internal {
                reason: format!("flashback channel: could not create the outcome publisher: {e:?}"),
            })?;
        Ok(Self {
            requests,
            outcomes,
            recorder: recorder.into(),
            _node: None,
            request_decode_latch: Mutex::new(FailureRegimeLatch::new()),
        })
    }

    /// Drain every request waiting.
    ///
    /// A MALFORMED frame is dropped and the drain CONTINUES: this is a
    /// machine-local control channel, but a peer from a future build — or a
    /// stray writer — must not be able to stop a recorder from hearing the
    /// operator's next request. The same tolerance `mirror_registry`'s drain
    /// has, for the same reason.
    ///
    /// # The drop is a WARN, and it is LATCHED
    ///
    /// It used to be a `debug!`, which on a robot means nothing at all: every
    /// crate in that binary's tree enables `tracing/release_max_level_info`. So
    /// a producer asked for a capture, this recorder threw the ask away, and
    /// the operator's only evidence was a black box that never appeared —
    /// indistinguishable from a producer that never fired. That is exactly the
    /// class already raised at the producers, on the same argument;
    /// this is the consumer end of the same lost ask.
    ///
    /// It is LATCHED rather than raised bare because the condition REPEATS by
    /// construction: a skewed producer re-publishes under `request_and_linger`
    /// every [`FLASHBACK_REQUEST_RETRY_INTERVAL`] for the whole linger, on
    /// every death. A bare per-frame `warn!` on a drain loop is the
    /// disk-fill class, so this rides the repo's ONE shared flood-suppression
    /// machine — loud head, `debug!` repeats carrying the suppressed count, a
    /// loud re-announcement each decade, one recovery line when a readable
    /// frame arrives.
    pub fn drain_requests(&self) -> Vec<FlashbackRequestFrame> {
        let mut out = Vec::new();
        // A readable frame is what CLOSES the regime — but an EMPTY drain is
        // not an observation either way, so recovery is reported only when
        // something actually decoded. Tracked as a flag so the latch is locked
        // at most twice per drain rather than once per frame.
        let mut decoded_any = false;
        while let Ok(Some(sample)) = self.requests.receive() {
            match decode_request(&sample) {
                Ok(frame) => {
                    decoded_any = true;
                    out.push(frame);
                }
                Err(e) => report_unreadable_request(&self.request_decode_latch, &e, sample.len()),
            }
        }
        if decoded_any {
            report_request_decode_recovery(&self.request_decode_latch);
        }
        out
    }

    /// Publish one verdict. Best-effort: a failure is logged and the recorder
    /// carries on, because a lost verdict costs the operator an answer while a
    /// failed capture would cost them the moment.
    pub fn publish_outcome(&self, request_id: u64, outcome: FlashbackOutcome) {
        let frame = FlashbackOutcomeFrame {
            request_id,
            recorder: self.recorder.clone(),
            outcome,
        };
        let bytes = match encode_outcome(&frame) {
            Ok(b) => b,
            Err(e) => {
                tracing::warn!(
                    error = %e,
                    "flashback channel: could not encode a verdict — the requester will \
                     see no answer from this recorder"
                );
                return;
            }
        };
        match self.outcomes.loan_slice_uninit(bytes.len()) {
            Ok(sample) => {
                let sample = sample.write_from_slice(&bytes);
                if let Err(e) = sample.send() {
                    tracing::warn!(
                        error = ?e,
                        "flashback channel: could not send a verdict"
                    );
                }
            }
            Err(e) => tracing::warn!(
                error = ?e,
                "flashback channel: could not loan a verdict slot"
            ),
        }
    }
}

/// Report one unreadable REQUEST frame through the shared latch.
///
/// Split out of [`FlashbackResponder::drain_requests`] so the four line kinds
/// (loud head, decade re-announcement, suppressed repeat, and the recovery in
/// [`report_request_decode_recovery`]) sit together and can be read as one
/// policy. `total_failures` is the repo-wide spelling for the running total —
/// operators grep by key, so this quantity takes the same one everywhere.
fn report_unreadable_request(
    latch: &Mutex<FailureRegimeLatch>,
    error: &dyn std::fmt::Display,
    len: usize,
) {
    let decision = lock_regime_latch(latch).on_failure();
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            error = %error,
            len,
            kind = "request",
            "flashback channel: dropping an unreadable capture REQUEST — something \
             asked this recorder for a black box and it could not read the ask, so no capture \
             will be made for it. The writer is from a different build; redeploy both ends. \
             Repeats are suppressed to debug until a readable request arrives."
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            error = %error,
            len,
            kind = "request",
            suppressed,
            total_failures = total,
            "flashback channel: STILL dropping every capture request — the running \
             total has crossed another decade since the last loud report. Redeploy both ends."
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            error = %error,
            len,
            kind = "request",
            suppressed,
            "flashback channel: unreadable capture request suppressed (regime still \
             open)"
        ),
    }
}

/// One recovery line when a readable REQUEST closes an open regime.
///
/// Reported only when the regime it closed actually downgraded something — a
/// lone-drop regime re-arms silently, so a channel that sees one bad frame an
/// hour does not also write an hourly recovery line.
fn report_request_decode_recovery(latch: &Mutex<FailureRegimeLatch>) {
    let closed = lock_regime_latch(latch).on_success();
    if let Some(suppressed) = closed {
        tracing::info!(
            suppressed_count = suppressed,
            kind = "request",
            "flashback channel: capture requests are readable again"
        );
    }
}

/// Report one unreadable VERDICT frame through the shared latch.
fn report_unreadable_verdict(
    latch: &Mutex<FailureRegimeLatch>,
    error: &dyn std::fmt::Display,
    len: usize,
) {
    let decision = lock_regime_latch(latch).on_failure();
    match decision {
        RegimeDecision::Loud => tracing::warn!(
            error = %error,
            len,
            kind = "verdict",
            "flashback channel: dropping an unreadable capture VERDICT — a recorder \
             DID answer and this build could not read the answer, so the ask will be reported \
             as unanswered. The recorder is from a different build; redeploy both ends. \
             Repeats are suppressed to debug until a readable verdict arrives."
        ),
        RegimeDecision::StillFailing { total, suppressed } => tracing::warn!(
            error = %error,
            len,
            kind = "verdict",
            suppressed,
            total_failures = total,
            "flashback channel: STILL dropping every capture verdict — the running \
             total has crossed another decade since the last loud report. Redeploy both ends."
        ),
        RegimeDecision::Suppressed { suppressed } => tracing::debug!(
            error = %error,
            len,
            kind = "verdict",
            suppressed,
            "flashback channel: unreadable capture verdict suppressed (regime still \
             open)"
        ),
    }
}

/// One recovery line when a readable VERDICT closes an open regime.
fn report_verdict_decode_recovery(latch: &Mutex<FailureRegimeLatch>) {
    let closed = lock_regime_latch(latch).on_success();
    if let Some(suppressed) = closed {
        tracing::info!(
            suppressed_count = suppressed,
            kind = "verdict",
            "flashback channel: capture verdicts are readable again"
        );
    }
}

/// The OBSERVER's end: write a request, read the verdicts.
pub struct FlashbackRequester {
    requests: Publisher<CerService, [u8], ()>,
    outcomes: Subscriber<CerService, [u8], ()>,
    /// Kept alive for the ports' lifetime when this end minted its own node.
    _node: Option<Node<CerService>>,
    /// Flood suppression for unreadable VERDICT frames. See
    /// [`FlashbackResponder::request_decode_latch`] for the `Mutex`.
    verdict_decode_latch: Mutex<FailureRegimeLatch>,
}

impl FlashbackRequester {
    /// Attach an observer to the channel on the namespace `mgr` runs on.
    ///
    /// # Errors
    ///
    /// As [`FlashbackRequester::open`].
    pub fn open_on_manager(mgr: &crate::transport::TransportManager) -> TransportResult<Self> {
        Self::open_on_config(&mgr.iox_config())
    }

    /// [`FlashbackRequester::open_on_manager`] against an explicit config.
    ///
    /// # Errors
    ///
    /// As [`FlashbackRequester::open`].
    pub fn open_on_config(config: &iceoryx2::config::Config) -> TransportResult<Self> {
        // The ONE seam that makes both producers' "could not open the
        // trigger channel" branch reachable. Above `build_channel_node` on
        // purpose — an isolated iceoryx2 namespace always opens, so a test can
        // only reach that branch by never attempting the open at all. Compiled
        // out of a shipping build; see `flashback::fault_injection`.
        #[cfg(any(test, feature = "test-helpers"))]
        if let Some(injected) = crate::flashback::fault_injection::channel_open_error() {
            return Err(injected);
        }
        let node = build_channel_node(config, "cerulion-flashback-client")?;
        let mut this = Self::open(&node)?;
        this._node = Some(node);
        Ok(this)
    }

    /// Attach an observer to the channel.
    ///
    /// The outcome SUBSCRIBER is created before the request publisher, and the
    /// order is the contract rather than a style: a verdict published before this
    /// subscriber existed is not missed by timing, it is structurally
    /// unreachable.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if either service or either port cannot be created.
    pub fn open(node: &Node<CerService>) -> TransportResult<Self> {
        let outcome_service = open_outcome_service(node)?;
        let outcomes = outcome_service.subscriber_builder().create().map_err(|e| {
            TransportError::Internal {
                reason: format!(
                    "flashback channel: could not create the outcome subscriber: {e:?}"
                ),
            }
        })?;
        let request_service = open_request_service(node)?;
        let requests = request_service
            .publisher_builder()
            .initial_max_slice_len(MAX_FLASHBACK_RECORD_LEN)
            .create()
            .map_err(|e| TransportError::Internal {
                reason: format!("flashback channel: could not create the request publisher: {e:?}"),
            })?;
        Ok(Self {
            requests,
            outcomes,
            _node: None,
            verdict_decode_latch: Mutex::new(FailureRegimeLatch::new()),
        })
    }

    /// Publish `request` under a freshly minted id, and return that id.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if the request cannot be encoded, loaned or sent.
    pub fn request(&self, request: &CaptureRequest) -> TransportResult<u64> {
        let request_id = mint_request_id();
        self.request_with_id(request, request_id)?;
        Ok(request_id)
    }

    /// Publish `request` under an EXISTING id.
    ///
    /// The re-publish a caller needs when nothing has answered: iceoryx2 pub/sub
    /// keeps no history for a subscriber that attaches later, so a request that
    /// raced a recorder's startup is not delivered, and the id must be REUSED or
    /// the verdicts would belong to an ask nobody is listening for.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if the request cannot be encoded, loaned or sent.
    pub fn request_with_id(
        &self,
        request: &CaptureRequest,
        request_id: u64,
    ) -> TransportResult<()> {
        // The seam for both producers' "could not publish the request"
        // branch. It sits on `request_with_id` rather than on `request` because
        // that is the funnel — `request` and every `request_batch_and_linger`
        // re-publish come through here — so an armed fault fails a whole batch,
        // which is what the PER-REQUEST reporting contract needs to be pinned
        // against. Compiled out of a shipping build.
        #[cfg(any(test, feature = "test-helpers"))]
        if let Some(injected) = crate::flashback::fault_injection::publish_error() {
            return Err(injected);
        }
        let frame = FlashbackRequestFrame {
            request_id,
            request: request.clone(),
        };
        let bytes = encode_request(&frame).map_err(|e| TransportError::Internal {
            reason: format!("flashback channel: could not encode the request: {e}"),
        })?;
        let sample =
            self.requests
                .loan_slice_uninit(bytes.len())
                .map_err(|e| TransportError::Internal {
                    reason: format!("flashback channel: could not loan a request slot: {e:?}"),
                })?;
        sample
            .write_from_slice(&bytes)
            .send()
            .map_err(|e| TransportError::Internal {
                reason: format!("flashback channel: could not send the request: {e:?}"),
            })?;
        Ok(())
    }

    /// Publish `request` and KEEP THIS END ALIVE long enough for a recorder to
    /// take it — the shape a FIRE-AND-FORGET producer needs.
    ///
    /// # Why a producer cannot simply publish and return
    ///
    /// iceoryx2 reclaims a departing publisher's unread samples, so a requester
    /// that is dropped before the recorder's next drive pass takes its own
    /// request back out of the queue. That is MEASURED rather than inferred (see
    /// [`FLASHBACK_UNWATCHED_LINGER`]), and it makes "publish and return" a race
    /// the producer usually loses: a recorder idles at a ~10 ms tick, while a
    /// requester built per call drops microseconds after `send`.
    ///
    /// So this holds the ports open for `linger`, RE-PUBLISHING under the SAME id
    /// every [`FLASHBACK_REQUEST_RETRY_INTERVAL`] — which also covers the other
    /// half of the problem the `cerulion flashback` verb found: a request
    /// published into the window between a recorder's process starting and its
    /// control subscriber existing is not delivered, because this channel keeps
    /// no history for a late subscriber.
    ///
    /// # It still does not WAIT ON THE ANSWER, and that distinction is load-bearing
    ///
    /// Nothing here touches the outcome subscriber, decodes an outcome, or
    /// changes what the caller returns. That keeps the verdict-9 back-compat
    /// condition recorded in [`encode_outcome`] UNBOUND — it is owed the first
    /// time something publishes a switchable kind *and waits on the answer*, and
    /// keeping a publisher alive is not waiting.
    ///
    /// Re-publishing is safe for the same reason it is safe in the manual verb: a
    /// repeat landing inside an ACTIVE capture COALESCES into it (one bag, causes
    /// as a list) rather than opening a second one, and reusing the id keeps every
    /// verdict attached to this one ask.
    ///
    /// `stop` is checked on the same slice as the sleep so a shutting-down caller
    /// is not held for the full span.
    ///
    /// # Errors
    ///
    /// [`TransportError`] if the FIRST publish fails. A failed RE-publish is
    /// deliberately not an error: the first one may well have been delivered, and
    /// reporting a failure would claim more than this call can establish.
    pub fn request_and_linger(
        &self,
        request: &CaptureRequest,
        linger: Duration,
        stop: &dyn Fn() -> bool,
    ) -> TransportResult<u64> {
        let request_id = self.request(request)?;
        let deadline = Instant::now() + linger;
        let mut next_retry = Instant::now() + FLASHBACK_REQUEST_RETRY_INTERVAL;
        while !stop() && Instant::now() < deadline {
            if Instant::now() >= next_retry {
                let _ = self.request_with_id(request, request_id);
                next_retry = Instant::now() + FLASHBACK_REQUEST_RETRY_INTERVAL;
            }
            std::thread::sleep(FLASHBACK_POLL_INTERVAL);
        }
        Ok(request_id)
    }

    /// Publish EVERY request of a batch, then re-publish EVERY one of them for
    /// `linger` — one channel open, one linger, for the whole batch.
    ///
    /// # Why the whole batch has to be re-published, not just the last
    ///
    /// The trigger service is opened with NO history (`open_service` sets
    /// `subscriber_max_buffer_size` and nothing else, and iceoryx2 delivers no
    /// late-joiner history unless a service asks for it), so a frame published
    /// before the recorder's subscriber exists reaches nobody and is gone. That
    /// is precisely what the linger exists to survive.
    ///
    /// Lingering on only the LAST request of a batch therefore rescued exactly
    /// one death and silently lost the rest: the gate would still COALESCE the
    /// survivors into one bag, so the capture still happens and the loss is
    /// invisible — but the bag's cause list, which is what an operator reads to
    /// learn WHICH nodes died, names one node instead of five. The evidence goes
    /// missing while the artifact looks complete, which is the worst shape a
    /// black box can fail in.
    ///
    /// Re-publishing all of them costs nothing extra: same open, same wall, same
    /// retry ticks — only more frames per tick, into a queue that is per
    /// publisher-connection (see [`FLASHBACK_QUEUE_DEPTH`]) and sized far above
    /// any batch a ledger cap admits.
    ///
    /// # Returns
    ///
    /// One result PER REQUEST, in the caller's order, so a caller can report
    /// exactly which of its requests failed to publish. A request whose FIRST
    /// publish fails is not re-published — there is no id to re-publish under —
    /// and its `Err` is reported here rather than aborting its siblings, which
    /// is the whole reason this returns a vector instead of one `Result`.
    pub fn request_batch_and_linger(
        &self,
        requests: &[CaptureRequest],
        linger: Duration,
        stop: &dyn Fn() -> bool,
    ) -> Vec<TransportResult<u64>> {
        let first: Vec<TransportResult<u64>> = requests.iter().map(|r| self.request(r)).collect();
        // Only the ones that really went out get re-published.
        let live: Vec<(&CaptureRequest, u64)> = requests
            .iter()
            .zip(first.iter())
            .filter_map(|(r, res)| res.as_ref().ok().map(|id| (r, *id)))
            .collect();
        if live.is_empty() {
            return first;
        }
        let deadline = Instant::now() + linger;
        let mut next_retry = Instant::now() + FLASHBACK_REQUEST_RETRY_INTERVAL;
        while !stop() && Instant::now() < deadline {
            if Instant::now() >= next_retry {
                for (request, id) in &live {
                    // A failed RE-publish is deliberately not reported: the
                    // first one may well have landed, and saying otherwise would
                    // claim more than this call can establish.
                    let _ = self.request_with_id(request, *id);
                }
                next_retry = Instant::now() + FLASHBACK_REQUEST_RETRY_INTERVAL;
            }
            std::thread::sleep(FLASHBACK_POLL_INTERVAL);
        }
        first
    }

    /// Drain the verdicts for `request_id` that have arrived.
    ///
    /// Frames for OTHER request ids are discarded: two operators asking at once
    /// must not read each other's answers. That discard is SILENT and stays so
    /// — it is the channel working exactly as designed, and says nothing about
    /// this end's health.
    ///
    /// # An UNREADABLE verdict is a WARN, and it is LATCHED
    ///
    /// An undecodable frame is a different fact from a frame addressed
    /// elsewhere, and it used to log at `debug!` — invisible in a release
    /// build. The caller then reports "no recorder answered", when what
    /// actually happened is that one DID and this build could not read it: a
    /// version skew, with a redeploy as the remedy, presented to the operator
    /// as an absence with no remedy at all.
    ///
    /// Latched for the reason [`FlashbackResponder::drain_requests`] gives — a
    /// caller waiting on a verdict re-drains on every [`FLASHBACK_POLL_INTERVAL`]
    /// tick until its deadline, so a skewed recorder answering repeatedly would
    /// otherwise write a warn per poll for the whole wait.
    pub fn drain_outcomes(&self, request_id: u64) -> Vec<FlashbackOutcomeFrame> {
        let mut out = Vec::new();
        // Any DECODABLE frame closes the regime, including one addressed to
        // another asker: what the latch tracks is whether this end can read
        // what the channel carries, not whether the answer was for us.
        let mut decoded_any = false;
        while let Ok(Some(sample)) = self.outcomes.receive() {
            match decode_outcome(&sample) {
                Ok(frame) if frame.request_id == request_id => {
                    decoded_any = true;
                    out.push(frame);
                }
                Ok(_) => decoded_any = true,
                Err(e) => report_unreadable_verdict(&self.verdict_decode_latch, &e, sample.len()),
            }
        }
        if decoded_any {
            report_verdict_decode_recovery(&self.verdict_decode_latch);
        }
        out
    }
}

/// The two DRAIN-side drops used to log at `debug!`, which on a robot
/// means nothing at all. These arms pin the flip AND the flood suppression that
/// makes the flip affordable.
///
/// Every predicate matches the LEVEL TOKEN as well as the message: a text-only
/// filter would still pass if `warn!` reverted to `debug!`, since that leaves
/// the wording, the fields and the counters exactly as they were.
///
/// RELEASE-SAFE: `cerulion_core` enables `tracing/release_max_level_info`, so a
/// release build compiles every `debug!` out and a DEBUG-line count reads 0
/// whatever the latch did. The suppressed-repeat count therefore goes through
/// `crate::testing::debug_lines_expected` — the ONE gate helper the source walk in
/// `tests/debug_count_discipline_test.rs` now REQUIRES at every DEBUG-count site in
/// `cerulion_core` and `rmw_cerulion` — and the suppression contract rests on two
/// level-free legs in BOTH profiles — [`suppressed_never_loud`] and the latch's
/// own `total_failures` / `is_failing`, read directly — never on the DEBUG count
/// alone. The first cut asserted that count unconditionally and turned the ONE
/// job that runs `cargo test -p cerulion_core --lib --release` (`Latency
/// Threshold (Linux)`, push-to-main plus manual dispatch — never a PR) red for
/// 26 consecutive main runs while every PR shard stayed green.
///
/// Both drive the REAL drain over REAL transport, with a raw publisher putting
/// undecodable bytes on the channel's own service — the only way to reach these
/// branches, since neither `FlashbackRequester` nor `FlashbackResponder` can
/// encode a frame the other end cannot read.
///
/// Parallel-safe: each mints its own isolated iceoryx2 namespace.
#[cfg(test)]
mod drain_drop_tests {
    use super::*;
    use crate::testing::debug_lines_expected;

    /// Lines at `level` whose message contains `marker` — the ONE shared body
    /// ([`crate::testing::lines_at_exclusively`]), which also refuses when a
    /// line carrying the same marker sits at another level.
    ///
    /// Whole whitespace tokens read from the line HEADER, never a substring:
    /// `tracing-test` renders the test function's own name into every line, so
    /// `line.contains("WARN")` would be satisfied by a test whose NAME contains
    /// it. `Result` rather than a panic: every call site is inside a `logs_assert`
    /// closure that already returns `Result<(), String>`, and a panic there
    /// bypasses `logs_assert`'s own message path and poisons the capture mutex
    /// for the rest of the binary.
    fn lines_at<'a>(lines: &[&'a str], level: &str, marker: &str) -> Result<Vec<&'a str>, String> {
        crate::testing::lines_at_exclusively(lines, level, &[marker])
    }

    /// `true` when `line` carries `key=value` as a whole whitespace token.
    ///
    /// A whole-token match because a substring one makes `suppressed=8` match
    /// `suppressed=80` and `total_failures=10` match `total_failures=100`.
    fn has_field(line: &str, key: &str, value: &str) -> bool {
        let want = format!("{key}={value}");
        line.split_whitespace().any(|t| t == want)
    }

    /// UNCONDITIONAL, level-independent: a suppressed repeat must never be LOUD.
    ///
    /// The half of the suppression contract that survives
    /// `release_max_level_info`. See `crate::testing::debug_lines_expected` for why the
    /// DEBUG count cannot carry it alone.
    fn suppressed_never_loud(lines: &[&str], marker: &str) -> Result<(), String> {
        crate::testing::never_loud(lines, marker)
    }

    /// Lines carrying `marker` at any level.
    fn lines_with(lines: &[&str], marker: &str) -> usize {
        lines.iter().filter(|l| l.contains(marker)).count()
    }

    /// Bytes no decoder on this channel can read (the magic is wrong).
    const GARBAGE: &[u8] = &[0xFF; 16];

    /// How many bad frames the burst carries.
    ///
    /// DERIVED from the latch's own decade base rather than typed as `10`, so
    /// the burst reaches the re-announcement boundary by construction. A literal
    /// would silently stop covering the `StillFailing` arm if that base ever
    /// moved.
    const BURST: usize = crate::transport::failure_regime_latch::DECADE_BASE as usize;

    /// A raw publisher on `factory`'s service.
    ///
    /// Held for the WHOLE test by every caller, never built per burst:
    /// iceoryx2 reclaims a DEPARTING publisher's unread samples, so a publisher
    /// dropped before the drain takes its own frames back out of the queue and
    /// the drain sees an empty channel. (That is the same MEASURED hazard
    /// `request_and_linger` exists for — without it,
    /// every arm reads `all lines: []`.)
    fn raw_publisher(
        factory: &PortFactory<CerService, [u8], ()>,
    ) -> Publisher<CerService, [u8], ()> {
        factory
            .publisher_builder()
            .initial_max_slice_len(MAX_FLASHBACK_RECORD_LEN)
            .create()
            .expect("a raw publisher on the control service")
    }

    /// Publish `frames` on `publisher`, once per element.
    fn publish_all(publisher: &Publisher<CerService, [u8], ()>, frames: &[Vec<u8>]) {
        for bytes in frames {
            publisher
                .loan_slice_uninit(bytes.len())
                .expect("loan a slot")
                .write_from_slice(bytes)
                .send()
                .expect("send the frame");
        }
    }

    /// **An unreadable capture REQUEST is a latched WARN, through the full cycle.**
    ///
    /// The whole policy in one body, because the arms are only meaningful
    /// together: a loud head alone could be an unlatched flood, suppressed
    /// repeats alone could be a latch that never speaks, and a recovery line
    /// alone says nothing about either.
    ///
    /// The recovery + re-arm half is also the anti-tautology control: a readable
    /// request must produce NO warn, so a reporter that fired on every frame
    /// fails here rather than passing three assertions about counts.
    #[test]
    #[tracing_test::traced_test]
    fn an_unreadable_capture_request_is_a_latched_warn_through_the_full_cycle() {
        const MARKER: &str = "dropping an unreadable capture REQUEST";
        const SUPPRESSED: &str = "unreadable capture request suppressed";
        let config = crate::testing::iceoryx_test_config();
        // The responder FIRST: this channel keeps no history for a late
        // subscriber, so a frame published before it exists is unreachable.
        let responder =
            FlashbackResponder::open_on_config(&config, "recorder").expect("a responder");
        let writer_node = build_channel_node(&config, "raw-writer").expect("a node");
        let requests = open_request_service(&writer_node).expect("the request service");
        let writer = raw_publisher(&requests);

        let burst: Vec<Vec<u8>> = (0..BURST).map(|_| GARBAGE.to_vec()).collect();
        publish_all(&writer, &burst);
        assert!(
            responder.drain_requests().is_empty(),
            "PRECONDITION: not one garbage frame may be handed to the recorder as a request"
        );
        // Level-free leg: the latch counted every frame, whatever the log did.
        {
            let latch = lock_regime_latch(&responder.request_decode_latch);
            assert_eq!(
                latch.total_failures(),
                BURST as u64,
                "every unreadable frame is a counted failure, independent of log level"
            );
            assert!(latch.is_failing(), "the regime is open after the burst");
        }

        // A readable request closes the regime AND is delivered.
        let good = encode_request(&FlashbackRequestFrame {
            request_id: 42,
            request: CaptureRequest::manual("operator asked"),
        })
        .expect("encode a good request");
        publish_all(&writer, &[good]);
        let delivered = responder.drain_requests();
        assert_eq!(
            delivered.len(),
            1,
            "PRECONDITION: a readable request still gets through — the drain CONTINUES past bad \
             frames, which is the tolerance this channel is built on"
        );
        {
            let latch = lock_regime_latch(&responder.request_decode_latch);
            assert!(!latch.is_failing(), "a readable request closes the regime");
            assert_eq!(
                latch.total_failures(),
                BURST as u64,
                "a readable request is not a failure — the running total must not move"
            );
        }

        // …and the regime re-arms: the next bad frame is loud again.
        publish_all(&writer, &[GARBAGE.to_vec()]);
        assert!(responder.drain_requests().is_empty());
        {
            let latch = lock_regime_latch(&responder.request_decode_latch);
            assert!(
                latch.is_failing(),
                "the regime re-armed on the next bad frame"
            );
            assert_eq!(
                latch.total_failures(),
                BURST as u64 + 1,
                "the running total is never reset by recovery"
            );
        }

        logs_assert(|lines: &[&str]| {
            let loud = lines_at(lines, "WARN", MARKER)?;
            if loud.len() != 2 {
                return Err(format!(
                    "expected 2 loud heads (first of regime, then RE-ARMED after recovery), got \
                     {} — all lines: {:?}",
                    loud.len(),
                    lines
                ));
            }
            suppressed_never_loud(lines, SUPPRESSED)?;
            let suppressed = lines_at(lines, "DEBUG", SUPPRESSED)?;
            let want_suppressed = debug_lines_expected(BURST - 2);
            if suppressed.len() != want_suppressed {
                return Err(format!(
                    "expected {want_suppressed} DEBUG repeats (the burst, less its loud head and \
                     its decade re-announcement; 0 where `debug!` is compiled out), got {}",
                    suppressed.len()
                ));
            }
            let still = lines_at(lines, "WARN", "STILL dropping every capture request")?;
            if still.len() != 1 {
                return Err(format!(
                    "the open regime must re-announce itself at the decade — the counter is not \
                     reachable from a log-reading operator, so the line is their only window. \
                     Got {} such lines",
                    still.len()
                ));
            }
            if !has_field(still[0], "total_failures", &BURST.to_string()) {
                return Err(format!(
                    "the re-announcement must carry the RUNNING TOTAL as a field: {}",
                    still[0]
                ));
            }
            let recovery = lines_at(lines, "INFO", "capture requests are readable again")?;
            if recovery.len() != 1 {
                return Err(format!(
                    "expected exactly one recovery line, got {}",
                    recovery.len()
                ));
            }
            // The head and the re-announcement were LOUD, so neither was
            // suppressed: the recovery reports what the operator MISSED.
            if !has_field(recovery[0], "suppressed_count", &(BURST - 2).to_string()) {
                return Err(format!(
                    "recovery must report the SUPPRESSED count, not the total: {}",
                    recovery[0]
                ));
            }
            Ok(())
        });
    }

    /// **An unreadable capture VERDICT is a latched WARN, through the full cycle.**
    ///
    /// The requester's end of the same condition, and it needs its own arm: it
    /// is a separate call site with a separate latch, and one open regime must
    /// never swallow the other's loud head.
    ///
    /// The `Ok(_)` arm — a verdict addressed to ANOTHER asker — stays silent and
    /// still closes the regime, which is asserted here because the two facts
    /// pull in opposite directions: it must not be reported (it is the channel
    /// working correctly) and it must count as evidence this build can read what
    /// the channel carries.
    #[test]
    #[tracing_test::traced_test]
    fn an_unreadable_capture_verdict_is_a_latched_warn_through_the_full_cycle() {
        const MARKER: &str = "dropping an unreadable capture VERDICT";
        const SUPPRESSED: &str = "unreadable capture verdict suppressed";
        const MINE: u64 = 7;
        const SOMEONE_ELSES: u64 = 8;
        let config = crate::testing::iceoryx_test_config();
        let requester = FlashbackRequester::open_on_config(&config).expect("a requester");
        let writer_node = build_channel_node(&config, "raw-recorder").expect("a node");
        let outcomes = open_outcome_service(&writer_node).expect("the outcome service");
        let writer = raw_publisher(&outcomes);

        let burst: Vec<Vec<u8>> = (0..BURST).map(|_| GARBAGE.to_vec()).collect();
        publish_all(&writer, &burst);
        assert!(
            requester.drain_outcomes(MINE).is_empty(),
            "PRECONDITION: not one garbage frame may be handed back as a verdict"
        );
        // Level-free leg: the latch counted every frame, whatever the log did.
        {
            let latch = lock_regime_latch(&requester.verdict_decode_latch);
            assert_eq!(
                latch.total_failures(),
                BURST as u64,
                "every unreadable frame is a counted failure, independent of log level"
            );
            assert!(latch.is_failing(), "the regime is open after the burst");
        }

        // A verdict for ANOTHER asker: silently discarded, but it is still
        // evidence this build can read what the channel carries, so it closes
        // the regime.
        let theirs = encode_outcome(&FlashbackOutcomeFrame {
            request_id: SOMEONE_ELSES,
            recorder: "other".to_string(),
            outcome: FlashbackOutcome::Accepted {
                seq: 1,
                ends_in_ms: 5_000,
                path: "/tmp/other.mcap".to_string(),
            },
        })
        .expect("encode a foreign verdict");
        publish_all(&writer, &[theirs]);
        assert!(
            requester.drain_outcomes(MINE).is_empty(),
            "PRECONDITION: two operators asking at once must not read each other's answers"
        );
        {
            let latch = lock_regime_latch(&requester.verdict_decode_latch);
            assert!(
                !latch.is_failing(),
                "a verdict for another asker still proves this build can READ the channel, so \
                 it closes the regime"
            );
            assert_eq!(
                latch.total_failures(),
                BURST as u64,
                "a readable verdict is not a failure — the running total must not move"
            );
        }

        publish_all(&writer, &[GARBAGE.to_vec()]);
        assert!(requester.drain_outcomes(MINE).is_empty());
        {
            let latch = lock_regime_latch(&requester.verdict_decode_latch);
            assert!(
                latch.is_failing(),
                "the regime re-armed on the next bad frame"
            );
            assert_eq!(
                latch.total_failures(),
                BURST as u64 + 1,
                "the running total is never reset by recovery"
            );
        }

        logs_assert(|lines: &[&str]| {
            let loud = lines_at(lines, "WARN", MARKER)?;
            if loud.len() != 2 {
                return Err(format!(
                    "expected 2 loud heads (first of regime, then RE-ARMED after a readable \
                     verdict closed it), got {} — all lines: {:?}",
                    loud.len(),
                    lines
                ));
            }
            suppressed_never_loud(lines, SUPPRESSED)?;
            let suppressed = lines_at(lines, "DEBUG", SUPPRESSED)?;
            let want_suppressed = debug_lines_expected(BURST - 2);
            if suppressed.len() != want_suppressed {
                return Err(format!(
                    "expected {want_suppressed} DEBUG repeats (0 where `debug!` is compiled \
                     out), got {}",
                    suppressed.len()
                ));
            }
            let still = lines_at(lines, "WARN", "STILL dropping every capture verdict")?;
            if still.len() != 1 {
                return Err(format!(
                    "the open regime must re-announce itself at the decade; got {} such lines",
                    still.len()
                ));
            }
            if !has_field(still[0], "total_failures", &BURST.to_string()) {
                return Err(format!(
                    "the re-announcement must carry the RUNNING TOTAL as a field: {}",
                    still[0]
                ));
            }
            let recovery = lines_at(lines, "INFO", "capture verdicts are readable again")?;
            if recovery.len() != 1 {
                return Err(format!(
                    "a verdict for another asker still proves this build can READ the channel, \
                     so it must close the regime — expected one recovery line, got {}",
                    recovery.len()
                ));
            }
            if !has_field(recovery[0], "suppressed_count", &(BURST - 2).to_string()) {
                return Err(format!(
                    "recovery must report the SUPPRESSED count, not the total: {}",
                    recovery[0]
                ));
            }
            Ok(())
        });
    }

    /// **A healthy drain logs NOTHING.**
    ///
    /// The control that keeps every "exactly N" count above meaningful: without it,
    /// a reporter that fired on readable frames too would still satisfy them
    /// only by accident, and a recovery line emitted on every drain would look
    /// like a working latch.
    ///
    /// An EMPTY drain is included deliberately: it is not an observation either
    /// way, and a recovery line written for one would be a claim about health
    /// that nothing observed.
    #[test]
    #[tracing_test::traced_test]
    fn a_healthy_drain_writes_no_drop_line_and_no_recovery() {
        let config = crate::testing::iceoryx_test_config();
        let responder =
            FlashbackResponder::open_on_config(&config, "recorder").expect("a responder");
        let writer_node = build_channel_node(&config, "raw-writer").expect("a node");
        let requests = open_request_service(&writer_node).expect("the request service");
        let writer = raw_publisher(&requests);

        assert!(
            responder.drain_requests().is_empty(),
            "PRECONDITION: nothing published yet"
        );
        let good = encode_request(&FlashbackRequestFrame {
            request_id: 1,
            request: CaptureRequest::manual("operator asked"),
        })
        .expect("encode a good request");
        publish_all(&writer, &[good]);
        assert_eq!(
            responder.drain_requests().len(),
            1,
            "PRECONDITION: the readable request is delivered — without this the silence below \
             proves only that nothing happened"
        );

        logs_assert(|lines: &[&str]| {
            // One marker per line, each with its zero ATTACHED, so the discipline
            // walk reads this as the silence control it is.
            if lines_with(lines, "dropping an unreadable capture REQUEST") != 0
                || lines_with(lines, "unreadable capture request suppressed") != 0
                || lines_with(lines, "capture requests are readable again") != 0
            {
                return Err(format!(
                    "a healthy drain must write nothing — a regime that never opened has \
                     nothing to recover from. Got {lines:?}"
                ));
            }
            Ok(())
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manual_frame() -> FlashbackRequestFrame {
        FlashbackRequestFrame {
            request_id: 0x0102_0304_0506_0708,
            request: CaptureRequest::manual("operator asked"),
        }
    }

    #[test]
    fn a_request_round_trips_through_hand_written_bytes() {
        let frame = manual_frame();
        let bytes = encode_request(&frame).expect("encodes");

        // HAND oracle: the header, byte for byte. A round-trip alone would pass
        // against any self-consistent pair of functions, including one that
        // changed the wire.
        assert_eq!(&bytes[0..2], &[0xCE, 0x82], "magic");
        assert_eq!(bytes[2], 1, "version");
        assert_eq!(&bytes[3..11], &0x0102_0304_0506_0708u64.to_le_bytes());
        assert_eq!(bytes[11], 1, "manual kind");
        assert_eq!(bytes[12], 0, "not pinned");
        assert_eq!(&bytes[13..15], &0u16.to_le_bytes(), "empty subject");
        assert_eq!(
            &bytes[15..17],
            &("operator asked".len() as u16).to_le_bytes()
        );
        assert_eq!(&bytes[17..], b"operator asked");

        assert_eq!(decode_request(&bytes).expect("decodes"), frame);
    }

    #[test]
    fn every_trigger_kind_and_the_pin_survive_the_wire() {
        for kind in [
            TriggerKind::Manual,
            TriggerKind::ProcessFault,
            TriggerKind::MonitorVerdict,
        ] {
            for pin in [false, true] {
                let frame = FlashbackRequestFrame {
                    request_id: 7,
                    request: CaptureRequest {
                        kind,
                        subject: "rank:1".into(),
                        detail: "worker exited".into(),
                        pin,
                    },
                };
                let bytes = encode_request(&frame).expect("encodes");
                assert_eq!(decode_request(&bytes).expect("decodes"), frame);
            }
        }
    }

    #[test]
    fn a_request_is_refused_rather_than_truncated_when_a_field_is_too_long() {
        let frame = FlashbackRequestFrame {
            request_id: 1,
            request: CaptureRequest::process_fault("x".repeat(MAX_FLASHBACK_SUBJECT_LEN + 1), "d"),
        };
        assert_eq!(
            encode_request(&frame),
            Err(FlashbackRecordError::TooLong),
            "a subject past its bound must be refused, never silently cut — a truncated regime \
             key is a DIFFERENT regime, so the latch would stop recognising repeats"
        );
    }

    /// The span trailer is ADDITIVE in the direction that
    /// matters — an older reader handed one of these frames reads exactly the
    /// verdict it always did.
    ///
    /// Both halves are driven, because "additive" is a claim about TWO readers:
    ///
    /// - a recorder with nothing to say appends nothing, so its bytes are
    ///   IDENTICAL to a pre-trailer frame (asserted on the LENGTH, which is the
    ///   only thing a trailer can change); and
    /// - an older reader, modelled by decoding the frame TRUNCATED to the length
    ///   it would have computed for itself, gets the same seq, bytes, path and
    ///   resimmable verdict — and `None` for the field it cannot name.
    ///
    /// That second half is the load-bearing one. `decode_outcome` bounds the text
    /// at `header + recorder_len + text_len` and checks `bytes.len() < end`, so
    /// it has always TOLERATED trailing bytes; if it had required an exact
    /// length, this trailer would make every new capture's verdict unreadable to
    /// every old requester on the machine.
    #[test]
    fn the_span_trailer_is_additive_in_both_directions() {
        let with_span = FlashbackOutcomeFrame {
            request_id: 7,
            recorder: "demo".into(),
            outcome: FlashbackOutcome::Finished {
                seq: 5,
                bytes: 155_000_000,
                path: "/w/f/a.mcap".into(),
                resimmable: Some(true),
                span: Some(FinishedSpan {
                    claimed_span_ms: 45_000,
                    achieved_span_ms: 1_990,
                    truncated_frames: 63_451,
                }),
            },
        };
        let mut without_span = with_span.clone();
        if let FlashbackOutcome::Finished { span, .. } = &mut without_span.outcome {
            *span = None;
        }

        let bare = encode_outcome(&without_span).expect("encodes");
        let full = encode_outcome(&with_span).expect("encodes");
        let old_len = OUTCOME_HEADER_LEN + "demo".len() + "/w/f/a.mcap".len();
        assert_eq!(
            bare.len(),
            old_len,
            "a recorder with no claim to make emits a pre-trailer frame, byte for byte"
        );
        assert_eq!(
            full.len(),
            old_len + OUTCOME_SPAN_TRAILER_LEN,
            "…and one with a claim appends exactly the trailer"
        );
        assert_eq!(
            &full[..old_len],
            &bare[..],
            "the trailer is APPENDED — nothing before it moves"
        );

        // An OLD reader: it computes `end` from the two length fields and stops
        // there, so this is the byte range it would ever look at.
        let old_view = decode_outcome(&full[..old_len]).expect("an old reader still decodes it");
        assert_eq!(old_view.outcome, without_span.outcome);

        // A NEW reader gets the numbers back exactly.
        let new_view = decode_outcome(&full).expect("decodes");
        assert_eq!(new_view.outcome, with_span.outcome);
        let FlashbackOutcome::Finished { span, .. } = new_view.outcome else {
            panic!("a Finished verdict");
        };
        let span = span.expect("the trailer");
        assert_eq!(span.shortfall_ms(), 43_010);

        // A verdict that carries no span never grows a trailer — otherwise an old
        // reader of an ACCEPTED frame would be handed bytes nobody meant to send.
        let accepted = FlashbackOutcomeFrame {
            request_id: 7,
            recorder: "demo".into(),
            outcome: FlashbackOutcome::Accepted {
                seq: 5,
                ends_in_ms: 15_000,
                path: "/w/f/a.mcap".into(),
            },
        };
        assert_eq!(
            encode_outcome(&accepted).expect("encodes").len(),
            old_len,
            "only the Finished verdict carries the trailer"
        );
    }

    /// The SHORTFALL is derived, never carried — so it cannot disagree with the
    /// two numbers it is a difference of, and an achievement somehow exceeding
    /// the claim reads as zero rather than as an enormous number.
    ///
    /// The EVICTION verdict is a separate question and is asked of a separate
    /// number: a shortfall does not imply eviction, and conflating the two is
    /// the defect this test pins.
    #[test]
    fn the_shortfall_is_a_saturating_difference_of_the_two_reported_spans() {
        let span = |claimed, achieved, truncated| FinishedSpan {
            claimed_span_ms: claimed,
            achieved_span_ms: achieved,
            truncated_frames: truncated,
        };
        assert_eq!(span(45_000, 1_990, 0).shortfall_ms(), 43_010);
        assert_eq!(span(45_000, 45_000, 0).shortfall_ms(), 0);
        assert_eq!(span(1, 9, 0).shortfall_ms(), 0);

        // The two are INDEPENDENT, in both directions: a large shortfall with
        // nothing evicted (an early trigger, sparse topics, a quiet interval)
        // and a capture that lost frames while still covering its whole claim.
        assert!(!span(45_000, 1_990, 0).evicted_during_capture());
        assert!(span(45_000, 1_990, 1).evicted_during_capture());
        assert!(span(45_000, 45_000, 12).evicted_during_capture());
    }

    /// A PARTIAL trailer is a torn frame and is
    /// REFUSED; bytes past a WHOLE one are the forward-compat mechanism and are
    /// IGNORED.
    ///
    /// Both halves in one body, because the pair is the whole point — a reader
    /// that refuses everything unexpected forbids the next additive trailer
    /// (which is how THIS one landed), and one that accepts everything reads a
    /// torn frame as an older writer's and hides corruption behind back-compat.
    ///
    /// Driven at the BOUNDARY rather than at literals, so the arms follow
    /// `OUTCOME_SPAN_TRAILER_LEN` if a future field widens it.
    #[test]
    fn a_partial_trailer_is_refused_while_bytes_past_a_whole_one_are_ignored() {
        let frame = FlashbackOutcomeFrame {
            request_id: 7,
            recorder: "demo".into(),
            outcome: FlashbackOutcome::Finished {
                seq: 5,
                bytes: 1024,
                path: "/w/f/a.mcap".into(),
                resimmable: Some(true),
                span: Some(FinishedSpan {
                    claimed_span_ms: 45_000,
                    achieved_span_ms: 1_990,
                    truncated_frames: 63_451,
                }),
            },
        };
        let full = encode_outcome(&frame).expect("encodes");
        let end = full.len() - OUTCOME_SPAN_TRAILER_LEN;

        // EVERY partial residual is refused — not merely one sampled length.
        for residual in 1..OUTCOME_SPAN_TRAILER_LEN {
            assert_eq!(
                decode_outcome(&full[..end + residual]),
                Err(FlashbackRecordError::Truncated),
                "a {residual}-byte residual is a TORN frame, not an older writer's"
            );
        }
        // …and the two ends of that range are the ones a reader trips on.
        assert_eq!(
            decode_outcome(&full[..end])
                .expect("no trailer decodes")
                .outcome,
            FlashbackOutcome::Finished {
                seq: 5,
                bytes: 1024,
                path: "/w/f/a.mcap".into(),
                resimmable: Some(true),
                span: None,
            },
            "ZERO residual is a pre-trailer frame and makes no claim"
        );
        let whole = decode_outcome(&full).expect("a whole trailer decodes");
        let FlashbackOutcome::Finished { span, .. } = whole.outcome else {
            panic!("a Finished verdict");
        };
        assert_eq!(span.expect("the trailer").truncated_frames, 63_451);

        // FORWARD COMPAT: a future writer's next additive trailer rides past
        // this one, and this build reads what it knows and ignores the rest.
        // Refusing here would forbid the extension that made this field
        // possible, which is why the two residual classes are answered apart.
        for extra in [1usize, 7, OUTCOME_SPAN_TRAILER_LEN, 4096] {
            let mut future = full.clone();
            future.extend(std::iter::repeat_n(0xAB, extra));
            let view = decode_outcome(&future).expect("a future trailer is tolerated");
            assert_eq!(
                view.outcome, frame.outcome,
                "{extra} unknown trailing bytes must not change what this build reads"
            );
        }
    }

    #[test]
    fn every_outcome_round_trips_and_the_suppression_numbers_survive() {
        let cases = [
            FlashbackOutcome::Accepted {
                seq: 3,
                ends_in_ms: 15_000,
                path: "/w/recordings/flashbacks/a.mcap".into(),
            },
            FlashbackOutcome::Extended {
                seq: 3,
                ends_in_ms: 12_500,
                path: "/w/recordings/flashbacks/a.mcap".into(),
            },
            FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed: 41 }),
            FlashbackOutcome::Suppressed(SuppressReason::Refractory {
                retry_in_ns: 42_000_000_000,
            }),
            FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 20,
                cap: 20,
                reserved_for_manual: 2,
            }),
            // The resimmable verdict rides `b` as a TRI-STATE, so all
            // three readings must round-trip — an old recorder's UNKNOWN
            // especially, which must never decode as a confident "no".
            FlashbackOutcome::Finished {
                seq: 3,
                bytes: 155_000_000,
                path: "/w/recordings/flashbacks/a.mcap".into(),
                resimmable: None,
                // The pre-trailer shape: no trailer, no claim.
                span: None,
            },
            FlashbackOutcome::Finished {
                seq: 4,
                bytes: 1,
                path: "/w/recordings/flashbacks/b.mcap".into(),
                resimmable: Some(false),
                span: Some(FinishedSpan {
                    claimed_span_ms: 45_000,
                    achieved_span_ms: 1_990,
                    truncated_frames: 63_451,
                }),
            },
            FlashbackOutcome::Finished {
                seq: 5,
                bytes: 2,
                path: "/w/recordings/flashbacks/c.mcap".into(),
                resimmable: Some(true),
                // A capture that lost nothing still CARRIES the pair — the
                // absence arm means "an older recorder", not "no shortfall".
                span: Some(FinishedSpan {
                    claimed_span_ms: 45_000,
                    achieved_span_ms: 45_000,
                    truncated_frames: 0,
                }),
            },
            FlashbackOutcome::Failed {
                seq: Some(3),
                reason: "no space left on device".into(),
            },
            // …and the arm with NO capture behind it, which must survive as
            // `None` rather than collapsing onto sequence 0.
            FlashbackOutcome::Failed {
                seq: None,
                reason: "a previous flashback is still being written".into(),
            },
            // The UNCERTAIN terminal — distinct from a failure on the wire, or
            // a reader would render "the recorder stopped waiting" as "the
            // capture failed" and be wrong whenever the writer merely ran late.
            FlashbackOutcome::Abandoned {
                seq: 11,
                reason: "the recorder stopped while this was still being written".into(),
            },
            // The refusal, a third terminal, distinct on
            // the wire from both of the two above. A reader that decoded it as
            // `Failed` would send an operator hunting a fault, and one that
            // decoded it as `Abandoned` would send them looking for a bag that
            // was deliberately never written.
            FlashbackOutcome::Refused {
                seq: 12,
                reason: "the capture plane cannot hold one whole checkpoint generation".into(),
            },
        ];
        for outcome in cases {
            let frame = FlashbackOutcomeFrame {
                request_id: 99,
                recorder: "demo".into(),
                outcome: outcome.clone(),
            };
            let bytes = encode_outcome(&frame).expect("encodes");
            assert_eq!(
                decode_outcome(&bytes).expect("decodes"),
                frame,
                "outcome {outcome:?} must survive the wire unchanged"
            );
        }
    }

    #[test]
    fn the_rate_cap_verdict_carries_both_numbers_apart() {
        // By design, the loud failure must reach the CLI user, and a
        // refusal an operator cannot quantify is indistinguishable from the
        // feature being broken. Both numbers are asserted apart, and they are
        // deliberately DIFFERENT here so a swap cannot pass.
        let frame = FlashbackOutcomeFrame {
            request_id: 5,
            recorder: "demo".into(),
            outcome: FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 20,
                cap: 17,
                // A THIRD distinct number, so a field swap between any two of the
                // three cannot round-trip and pass.
                reserved_for_manual: 3,
            }),
        };
        let bytes = encode_outcome(&frame).expect("encodes");
        match decode_outcome(&bytes).expect("decodes").outcome {
            FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window,
                cap,
                reserved_for_manual,
            }) => {
                assert_eq!(captures_in_window, 20);
                assert_eq!(cap, 17);
                assert_eq!(reserved_for_manual, 3);
            }
            other => panic!("expected a rate-cap verdict, got {other:?}"),
        }
    }

    #[test]
    fn a_hostile_frame_is_refused_by_name_rather_than_panicking() {
        assert_eq!(decode_request(&[]), Err(FlashbackRecordError::Truncated));
        assert_eq!(
            decode_request(&[0; REQUEST_HEADER_LEN]),
            Err(FlashbackRecordError::BadMagic)
        );

        let mut bad_version = encode_request(&manual_frame()).expect("encodes");
        bad_version[2] = 9;
        assert_eq!(
            decode_request(&bad_version),
            Err(FlashbackRecordError::BadVersion(9))
        );

        let mut bad_kind = encode_request(&manual_frame()).expect("encodes");
        bad_kind[11] = 0;
        assert_eq!(
            decode_request(&bad_kind),
            Err(FlashbackRecordError::UnknownKind(0))
        );

        // A declared length that runs past the frame must be REFUSED, not read:
        // this is the arm a slice index would panic on.
        let mut overflowing = encode_request(&manual_frame()).expect("encodes");
        overflowing[15..17].copy_from_slice(&(MAX_FLASHBACK_DETAIL_LEN as u16).to_le_bytes());
        assert_eq!(
            decode_request(&overflowing),
            Err(FlashbackRecordError::LengthOverflow)
        );

        let mut not_utf8 = encode_request(&manual_frame()).expect("encodes");
        let last = not_utf8.len() - 1;
        not_utf8[last] = 0xFF;
        assert_eq!(
            decode_request(&not_utf8),
            Err(FlashbackRecordError::NotUtf8)
        );

        let mut bad_verdict = encode_outcome(&FlashbackOutcomeFrame {
            request_id: 1,
            recorder: "r".into(),
            outcome: FlashbackOutcome::Failed {
                seq: None,
                reason: "x".into(),
            },
        })
        .expect("encodes");
        bad_verdict[11] = 200;
        assert_eq!(
            decode_outcome(&bad_verdict),
            Err(FlashbackRecordError::UnknownVerdict(200))
        );
    }

    #[test]
    fn the_two_service_names_are_reserved_and_carry_no_data_suffix() {
        // Both properties matter and neither implies the other: the reserved
        // prefix is what keeps these off `cerulion topic list`'s ROBOTS/TOPICS
        // sections, and the absent `/data` suffix is what keeps bagd's live
        // enumeration from TAPPING the channel a recorder is listening on.
        for name in [
            FLASHBACK_REQUEST_SERVICE_NAME,
            FLASHBACK_OUTCOME_SERVICE_NAME,
        ] {
            assert!(
                name.starts_with("/__cerulion/"),
                "{name} must live under the reserved prefix"
            );
            assert!(
                !name.ends_with("/data"),
                "{name} must not look like a topic's data service"
            );
        }
        assert_ne!(
            FLASHBACK_REQUEST_SERVICE_NAME, FLASHBACK_OUTCOME_SERVICE_NAME,
            "the request and reply halves must be different services, or a recorder would \
             read its own verdicts back as requests"
        );
    }

    #[test]
    fn two_requests_from_one_process_do_not_share_an_id() {
        // The whole point of the id: two operators (or one operator twice) must
        // not read each other's verdicts.
        let a = mint_request_id();
        let b = mint_request_id();
        assert_ne!(a, b);
    }
}
