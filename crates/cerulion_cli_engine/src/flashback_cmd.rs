// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion flashback` — capture the moment, now.
//!
//! # What the verb does
//!
//! Every serving `graph run` holds a rolling window of the last ~30 s (decision
//! 75). This publishes ONE manual request onto the `/__cerulion/flashback`
//! trigger channel, collects the verdicts, and tells the operator where their
//! capture landed.
//!
//! # Why it waits, and why it can be told not to
//!
//! A capture is `[T−30s, T+15s]`: the post window is REAL recording, so the bag
//! does not exist when the request is accepted. The verb therefore reports the
//! acceptance immediately (with the path and an ETA) and then waits for the
//! FINISHED verdict, so `cerulion flashback` hands you a file rather than a
//! promise, which is what "hands you ONE bag" means.
//!
//! `--no-wait` returns at the acceptance. It exists because an agent or a script
//! firing a capture during an incident should not block for fifteen seconds, and
//! the acceptance already carries the path the bag will have.
//!
//! # A refusal is LOUD, here, with its numbers
//!
//! A manual request is exempt from the per-cause latch and the refractory floor
//! (an operator asking twice means it twice), but NOT from the global rate cap —
//! that arm is the disk backstop. When it fires, the operator sees both numbers
//! and a nonzero exit, because a verb that printed "asked" and exited 0 while the
//! robot captured nothing is the misleading-surface class this repo refuses.
//!
//! # Several recorders answer, and all of them are reported
//!
//! The request is broadcast, so on a desk running two graphs BOTH capture. Every
//! verdict names its recorder and every one is printed; reporting only the first
//! would be silently wrong on exactly the machine where it matters.

use std::collections::BTreeSet;
use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::flashback::channel::{
    FlashbackOutcome, FlashbackOutcomeFrame, FlashbackRequester, FLASHBACK_FINALIZE_GRACE,
    FLASHBACK_POLL_INTERVAL, FLASHBACK_REQUEST_RETRY_INTERVAL, FLASHBACK_VERDICT_WINDOW,
};
use cerulion_core::flashback::trigger::{CaptureRequest, SuppressReason};
use cerulion_core::transport::TransportManager;

use crate::error::{CliError, CliResult};

/// What the operator asked for.
#[derive(Debug, Clone, Default)]
pub struct FlashbackOptions {
    /// A human note recorded into the capture's own manifest, so a bag found
    /// three weeks later says what it was about.
    pub note: Option<String>,
    /// Exclude the capture from retention eviction.
    pub pin: bool,
    /// Return at the acceptance rather than waiting for the bag.
    pub no_wait: bool,
}

/// PURE: what a set of verdicts adds up to.
///
/// A named verdict rather than a bare exit code, so the DECISION is oracle-
/// testable with no transport and the exit mapping lives in one place.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashbackVerdict {
    /// At least one recorder is capturing (or already was, and this joined it).
    Captured,
    /// Every recorder that answered REFUSED. The operator gets a nonzero exit,
    /// because nothing was captured and a verb must not claim otherwise.
    Refused,
    /// Nobody answered. Distinct from `Refused`: a refusal means a recorder
    /// heard and said no, while this means no serving graph on this machine is
    /// holding a window at all — a different problem with a different remedy.
    NoRecorder,
    /// The operator interrupted the wait.
    ///
    /// EXIT 0, and never a claim about the capture: a cancelled wait is an
    /// INTERRUPTION rather than a verdict. The capture may well be finishing on
    /// the robot right now.
    Interrupted,
}

impl FlashbackVerdict {
    /// Does this verdict mean the command failed?
    pub fn is_failure(self) -> bool {
        matches!(self, Self::Refused | Self::NoRecorder)
    }
}

/// PURE: classify the verdicts collected for one request.
///
/// `interrupted` wins over everything, on one rule: a wait the operator
/// stopped concluded nothing, so reporting a refusal (or an absence) would be
/// asserting something the run never established.
pub fn classify(outcomes: &[FlashbackOutcomeFrame], interrupted: bool) -> FlashbackVerdict {
    if interrupted {
        return FlashbackVerdict::Interrupted;
    }
    if outcomes.is_empty() {
        return FlashbackVerdict::NoRecorder;
    }
    // A `Failed` that NAMES a capture also proves one was accepted — the
    // recorder only reports a sequence for a capture it opened. Treating it as a
    // refusal would make a fast writer failure read as "the robot said no",
    // which is a different problem with a different remedy; the failure is
    // reported and the exit code comes from `terminal_failure`.
    let any_captured = outcomes.iter().any(|f| {
        matches!(
            f.outcome,
            FlashbackOutcome::Accepted { .. }
                | FlashbackOutcome::Extended { .. }
                | FlashbackOutcome::Finished { .. }
                | FlashbackOutcome::Failed { seq: Some(_), .. }
                | FlashbackOutcome::Abandoned { .. } // (`Abandoned` always names its capture — see the variant.)
        )
    });
    if any_captured {
        FlashbackVerdict::Captured
    } else {
        FlashbackVerdict::Refused
    }
}

/// PURE: a path as ONE shell word, quoted only when it has to be.
///
/// The RESIMMABLE line hands the operator a command to paste, and the path in it
/// is not ours: `CERULION_FLASHBACK_DIR` is taken verbatim, so a directory with
/// a space in it (`~/Robot Logs/`) yields a command that runs `bag play` against
/// a truncated path and reports a bag that is not there — a copyable line that
/// is WRONG rather than merely ugly.
///
/// SINGLE quotes, because inside them every shell metacharacter is literal;
/// POSIX has no escape for `'` within them, so an embedded quote closes,
/// escapes and reopens (`'\''`). An ordinary path is left BARE — quoting every
/// path would make the common line noisier to read for a hazard it does not
/// have.
fn shell_quote(path: &str) -> String {
    // The conservative safe set: anything outside it gets quoted, so a character
    // nobody thought about is quoted rather than passed through.
    let safe =
        |c: char| c.is_ascii_alphanumeric() || matches!(c, '/' | '.' | '_' | '-' | '=' | '+');
    if !path.is_empty() && path.chars().all(safe) {
        return path.to_string();
    }
    format!("'{}'", path.replace('\'', r"'\''"))
}

/// PURE: render one verdict for a human.
///
/// Every refusal quotes its NUMBERS. A suppression an operator cannot quantify
/// is indistinguishable from the feature being broken — the rule
/// `SuppressReason`'s own doc states, honoured at the one surface that shows it
/// to a person.
pub fn render_outcome(frame: &FlashbackOutcomeFrame) -> String {
    let who = &frame.recorder;
    match &frame.outcome {
        FlashbackOutcome::Accepted {
            seq,
            ends_in_ms,
            path,
        } => format!(
            "[{who}] capturing (#{seq}) — recording for another {:.1}s, then writing {path}",
            *ends_in_ms as f64 / 1000.0
        ),
        FlashbackOutcome::Extended {
            seq,
            ends_in_ms,
            path,
        } => format!(
            "[{who}] joined the capture already running (#{seq}) — it now records for another \
             {:.1}s, then writes {path}",
            *ends_in_ms as f64 / 1000.0
        ),
        // The RESIMMABLE verdict rides the line an operator
        // reads during an incident. It lived only in the bag's own manifest, so
        // learning whether the capture could be resumed meant opening it — after
        // the moment had passed. UNKNOWN prints nothing rather than a hedge: a
        // recorder that predates the verdict makes no claim, and inventing a
        // caveat for it would put a permanent question mark on every capture
        // that is fine.
        // The same line also carries how much of what it CLAIMED the bag actually
        // holds. `span_clause` is appended to whichever verdict line follows,
        // rather than being a fourth arm, so a capture cannot report itself
        // resimmable while staying silent about covering a fraction of its
        // window — the two facts are about the same bag and belong on one line.
        FlashbackOutcome::Finished {
            seq,
            bytes,
            path,
            resimmable,
            span,
        } => {
            let mb = *bytes as f64 / (1024.0 * 1024.0);
            // UNKNOWN prints NOTHING (an earlier recorder made no claim, and
            // a hedge there would put a permanent question mark on every capture
            // that is fine), and so does a capture that lost nothing — an
            // operator reading an ordinary capture should not have to parse a
            // clause saying so.
            //
            // A shortfall is reported NEUTRALLY, and the CAUSE only on evidence.
            // The two are independent: a capture triggered inside its first
            // span, a robot with sparse topics, or an interval nobody published
            // in each carry a positive shortfall with nothing evicted, and
            // blaming the byte ceiling there sends an operator to raise a cap
            // that never bound. `truncated_frames > 0` is what earns the causal
            // sentence; without it the line states the measurement and points at
            // the bag, which is where the per-topic detail lives.
            let span_clause = match span {
                Some(s) if s.shortfall_ms() > 0 => {
                    let mut clause = format!(
                        " — COVERS {:.1}s of the {:.1}s it claims ({:.1}s short)",
                        s.achieved_span_ms as f64 / 1000.0,
                        s.claimed_span_ms as f64 / 1000.0,
                        s.shortfall_ms() as f64 / 1000.0,
                    );
                    if s.evicted_during_capture() {
                        clause.push_str(&format!(
                            ": the window's byte ceiling evicted {} frame(s) during the capture. \
                             Raise CERULION_FLASHBACK_WINDOW_MAX_MB, or reduce or downsample the \
                             heaviest topics",
                            s.truncated_frames
                        ));
                    }
                    clause
                }
                _ => String::new(),
            };
            let line = match resimmable {
                Some(true) => format!(
                    // The path goes AFTER clap's `--`, not before the flags.
                    // `shell_quote` makes the shell pass a leading-dash path as
                    // ONE word, which is a different problem from clap then
                    // reading that word as a flag: `CERULION_FLASHBACK_DIR=-logs`
                    // yields `-logs/….mcap`, and the hinted command fails on the
                    // one recorder whose output most needs replaying.
                    "[{who}] captured (#{seq}): {path} ({mb:.1} MB) — RESIMMABLE: \
                     `cerulion bag play --resim all -- {}`",
                    shell_quote(path)
                ),
                Some(false) => format!(
                    "[{who}] captured (#{seq}): {path} ({mb:.1} MB) — NOT resimmable (the bag is \
                     readable; see `anchor.resimmable_reason` in its \
                     __cerulion/flashback.json)"
                ),
                None => format!("[{who}] captured (#{seq}): {path} ({mb:.1} MB)"),
            };
            format!("{line}{span_clause}")
        }
        FlashbackOutcome::Failed { reason, .. } => format!("[{who}] capture FAILED: {reason}"),
        // Deliberately NOT the word "failed": the recorder did not observe an
        // outcome, it stopped waiting, and the bag may be perfectly fine.
        FlashbackOutcome::Abandoned { reason, .. } => {
            format!("[{who}] capture UNCONFIRMED: {reason}")
        }
        // A refusal gets its OWN word. Not "failed" (nothing broke) and
        // not "suppressed" (this one ran) — the plane is too small to produce a
        // resimmable capture, and the reason carries the knob that fixes it.
        FlashbackOutcome::Refused { reason, .. } => {
            format!("[{who}] capture REFUSED: {reason}")
        }
        // # Why the RESERVE is rendered, and only when it applied
        //
        // A MANUAL request reaching this arm has hit the FULL cap — the reserve
        // is withheld from the automatic kinds, never from this verb — so it
        // reports `reserved_for_manual: 0` and reads exactly as it did before
        // the manual reserve existed. An AUTOMATIC kind is refused at the EFFECTIVE cap, and
        // rendering only that number is a half-explanation: an operator who set
        // `CERULION_FLASHBACK_MAX_PER_HOUR=20` and reads "the cap is 18" has been
        // told their knob is being ignored, which is the opposite of what
        // happened. So the missing slots are named, WITH what they are for —
        // otherwise the natural remedy is to raise the cap, when the budget was
        // never the operator's to reclaim.
        //
        // The clause is gated on NONZERO rather than on the kind, because zero is
        // exactly the set of refusals the reserve did not shape: a manual
        // request, and any policy whose cap is small enough that
        // `effective_cap`'s clamp collapsed the reserve away. Naming a reserve of
        // 0 would describe a mechanism that did not apply.
        FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
            captures_in_window,
            cap,
            reserved_for_manual,
        }) => {
            let reserve = if *reserved_for_manual > 0 {
                format!(
                    " ({reserved_for_manual} of this robot's hourly budget are reserved for \
                     manual captures, so an automatic trigger stops at {cap})"
                )
            } else {
                String::new()
            };
            format!(
                "[{who}] REFUSED: this robot has already captured {captures_in_window} \
                 flashback(s) in the last hour and the cap is {cap}{reserve}. Nothing was \
                 captured. Raise the cap with CERULION_FLASHBACK_MAX_PER_HOUR, or wait for the \
                 oldest capture to age out of the window"
            )
        }
        FlashbackOutcome::Suppressed(SuppressReason::Disabled { switch }) => format!(
            "[{who}] REFUSED: this trigger is switched OFF on this robot. Nothing was captured. \
             Turn it back on with {}=on",
            switch.env_var()
        ),
        FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed }) => format!(
            "[{who}] REFUSED: this condition is already captured and has not recovered \
             ({suppressed} request(s) suppressed so far). Nothing was captured"
        ),
        FlashbackOutcome::Suppressed(SuppressReason::Refractory { retry_in_ns }) => format!(
            "[{who}] REFUSED: this condition captured recently; it may capture again in {:.0}s. \
             Nothing was captured",
            *retry_in_ns as f64 / 1e9
        ),
    }
}

/// PURE: which CAPTURE a verdict is about — `(recorder, seq)`.
///
/// # Why the wait is sized by identity and never by a frame count
///
/// Two mistakes are possible here. They are the same mistake seen from opposite
/// directions, and both are caused by this command's own retry:
///
/// * a retry that reaches a recorder ALREADY capturing yields `Accepted` (the
///   first request) and then `Extended` (the retry coalescing into the same
///   capture, because bagd dedupes the request id in `attach_requester`). Counted
///   as frames that is TWO captures to wait for, while exactly ONE terminal
///   verdict is ever sent — so the command would time out and print "no bag was
///   reported" over a bag that had been written;
/// * an adjacency-only `dedup_by` collapses those only when they arrive next to
///   each other, so on a two-recorder desk an interleaved duplicate survives and
///   inflates the same count.
///
/// A SET keyed on the capture itself collapses both, and preserves what must
/// stay distinct: different recorders, and different captures on one recorder.
///
/// `None` for a verdict that is about no capture (a suppression), which is
/// exactly the set of frames the wait must not be sized by.
fn capture_identity(frame: &FlashbackOutcomeFrame) -> Option<(String, u64)> {
    let seq = match &frame.outcome {
        FlashbackOutcome::Accepted { seq, .. }
        | FlashbackOutcome::Extended { seq, .. }
        | FlashbackOutcome::Finished { seq, .. } => *seq,
        // A failure carries the sequence of the capture it belongs to,
        // so it SETTLES that acceptance. Mapped to `u64::MAX` instead, it
        // could never match the `(recorder, seq)`
        // an `Accepted` was tracked under — so a capture the recorder had
        // already reported failed would stay outstanding, and the command would wait
        // out its whole deadline before printing that no bag was reported.
        //
        // A failure with NO sequence never had an acceptance to settle (a
        // request refused while a previous capture was still being written), and
        // is treated like a suppression here for exactly that reason: it must
        // not invent an outstanding capture, and it must not settle somebody
        // else's.
        FlashbackOutcome::Failed { seq: Some(seq), .. }
        | FlashbackOutcome::Abandoned { seq, .. }
        // A refusal ALWAYS names its capture (only an accepted one can be
        // refused at finalize), so it settles the acceptance it belongs to,
        // exactly as `Abandoned` does.
        | FlashbackOutcome::Refused { seq, .. } => *seq,
        FlashbackOutcome::Failed { seq: None, .. } => return None,
        FlashbackOutcome::Suppressed(_) => return None,
    };
    Some((frame.recorder.clone(), seq))
}

/// PURE: one verdict per capture, whatever order they arrived in.
///
/// Order-independent by construction (a `BTreeSet` of identities), unlike the
/// adjacency-only `dedup_by` it replaces. Suppressions are kept whole — they
/// carry no capture identity, and two recorders refusing for different reasons
/// are two things an operator needs to read.
pub fn collapse_verdicts(frames: Vec<FlashbackOutcomeFrame>) -> Vec<FlashbackOutcomeFrame> {
    // Keyed on the LIFECYCLE STAGE as well as the capture.
    // A `Failed` carries its capture's sequence, so it shares an
    // identity with the `Accepted` it settles — and collapsing on identity alone
    // would DROP whichever arrived second. In the initial window that is the
    // terminal one, so a capture that failed (or finished) fast would lose its verdict
    // entirely: the command would wait out its deadline, print that no bag was
    // reported, and could exit 0 with nothing on disk.
    //
    // Announcements collapse among themselves (that is the retry/interleave rule)
    // and terminals collapse among themselves (one verdict per capture), but an
    // announcement never collapses a terminal.
    let mut seen: BTreeSet<(String, u64, bool)> = BTreeSet::new();
    let mut out = Vec::with_capacity(frames.len());
    for frame in frames {
        match capture_identity(&frame) {
            Some((recorder, seq)) => {
                if seen.insert((recorder, seq, is_terminal(&frame))) {
                    out.push(frame);
                }
            }
            None => out.push(frame),
        }
    }
    out
}

/// PURE: is this verdict the LAST word about its capture?
///
/// The distinction the collapse turns on, and the one the wait is settled by:
/// an `Accepted`/`Extended` announces a capture, a `Finished`/`Failed` ends it.
pub fn is_terminal(frame: &FlashbackOutcomeFrame) -> bool {
    matches!(
        frame.outcome,
        FlashbackOutcome::Finished { .. }
            | FlashbackOutcome::Failed { .. }
            | FlashbackOutcome::Abandoned { .. }
            // A refusal is the LAST word about its
            // capture — nothing further will be published for it.
            | FlashbackOutcome::Refused { .. }
    )
}

/// What the terminal verdicts collected so far add up to.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct OutcomeTally {
    /// Bags this command watched being written.
    pub produced: usize,
    /// Captures that were accepted and then FAILED, with their reasons.
    pub failures: Vec<String>,
    /// Captures the recorder STOPPED WAITING for, with their reasons.
    pub unconfirmed: Vec<String>,
    /// Captures the recorder REFUSED to finalize because they could not resim,
    /// with the knob that fixes it.
    ///
    /// Its own bucket, apart from `failures`: a refusal is the POLICY working on
    /// a machine that is too small, and it carries an exact remedy. Folded into
    /// `failures` it would read as a broken robot and send an operator looking
    /// for a fault instead of setting a knob.
    pub refused: Vec<String>,
}

/// PURE: the MIXED-OUTCOME rule, in one place, applied on EVERY exit route.
///
/// `None` means exit 0; `Some(message)` is the error the verb returns.
///
/// # The rule, decided explicitly
///
/// 1. **`produced > 0` ⇒ SUCCESS**, whatever else happened. A sibling recorder
///    wrote a bag, so the operator HAS the moment; every failure and every
///    unconfirmed capture was already printed. Failing here would say the ask did
///    not happen when it did — the original rationale, unchanged.
/// 2. **`produced == 0` and anything UNCONFIRMED ⇒ the UNCONFIRMED message**,
///    even when there are failures beside it. "Unknown" outranks "failed" when
///    nothing succeeded: telling an operator that every capture failed sends them
///    away from a bag that may be sitting on disk complete, and a bag they do not
///    go looking for is a bag they do not have. Both lists are printed, so
///    nothing is hidden by the precedence.
/// 3. **`produced == 0` and only failures ⇒ the all-failed message.**
/// 4. Nothing terminal at all ⇒ exit 0 (the wait's own timeout message, printed
///    by the caller, is what covers that case).
///
/// Split out so that it cannot be applied at ONE of the verb's exit routes and missed
/// on `--no-wait`, and so that the failure arm cannot return FIRST and
/// hide an unconfirmed capture behind a false "every capture failed".
/// One rule, one place, every route.
pub fn terminal_report(tally: &OutcomeTally) -> Option<String> {
    if tally.produced > 0 {
        return None;
    }
    if !tally.unconfirmed.is_empty() {
        let mut lines = tally.unconfirmed.clone();
        lines.extend(tally.failures.iter().cloned());
        return Some(format!(
            // ONE word for this state, the same one `render_outcome` prints, so an
            // operator greps for `UNCONFIRMED` and finds both the per-recorder
            // line and this summary.
            "the capture is UNCONFIRMED — a recorder stopped waiting for it, so its bag may be \
             complete or may be torn. Look for it before assuming either:\n  {}",
            lines.join("\n  ")
        ));
    }
    // A REFUSAL is a definite answer with an exact
    // remedy, so it outranks a failure as the headline — but NOT an unconfirmed
    // capture, on rule 2's reasoning: "unknown" still outranks a definite
    // negative, because a bag that may be on disk is one the operator should go
    // and look for. Both lists print either way, so the precedence hides
    // nothing.
    if !tally.refused.is_empty() {
        let mut lines = tally.refused.clone();
        lines.extend(tally.failures.iter().cloned());
        return Some(format!(
            // The same word `render_outcome` prints, so one grep finds the
            // per-recorder line and this summary.
            "the capture was REFUSED — this robot's plane cannot hold a whole checkpoint \
             generation, so a bag would not have been resimmable:\n  {}",
            lines.join("\n  ")
        ));
    }
    if !tally.failures.is_empty() {
        return Some(format!(
            "every accepted capture FAILED to produce a bag — nothing was captured:\n  {}",
            tally.failures.join("\n  ")
        ));
    }
    None
}

/// PURE: what to say when nobody answered.
///
/// Its own function because it is the message an operator on a healthy desk
/// with no graph running will see, and it must name the two things that could
/// be true rather than implying the feature is broken.
pub fn no_recorder_message() -> String {
    format!(
        "no serving graph on this machine answered — nothing is holding a Flashback window.\n  \
         Start a graph (`cerulion graph run <name>`), or check that the plane is not switched \
         off ({}=off).",
        cerulion_core::flashback::FLASHBACK_ENV
    )
}

/// Capture the moment.
///
/// # Errors
///
/// [`CliError`] when the transport or the trigger channel cannot be opened, or
/// when the verdict is a failure (see [`FlashbackVerdict::is_failure`]).
pub fn flashback_capture(
    opts: FlashbackOptions,
    running: Arc<AtomicBool>,
    out: &mut dyn Write,
) -> CliResult<()> {
    let manager = TransportManager::get_or_init()
        .map_err(|e| CliError::Validation(format!("could not open the local transport: {e}")))?;
    // Opens the outcome SUBSCRIBER before the request publisher — see
    // `FlashbackRequester::open`. A verdict published before that subscriber
    // exists is structurally unreachable, not merely likely to be missed.
    let requester = FlashbackRequester::open_on_manager(&manager).map_err(|e| {
        CliError::Validation(format!(
            "could not open the Flashback trigger channel: {e}\n  \
             This is the machine-local control plane every serving graph listens on."
        ))
    })?;

    let detail = opts
        .note
        .clone()
        .unwrap_or_else(|| "captured by an operator".to_string());
    let mut request = CaptureRequest::manual(detail);
    if opts.pin {
        request = request.pinned();
    }
    let stop = || !running.load(Ordering::Relaxed);
    // RE-PUBLISHED while nothing has answered, and the e2e is what found the
    // reason: iceoryx2 pub/sub keeps no history for a subscriber that attaches
    // later, so a request published in the window between a recorder's process
    // starting and its control subscriber existing is not delivered — and a
    // one-shot verb whose request can be silently dropped is exactly the verb
    // that lies about having run.
    //
    // Safe to repeat: a manual request landing inside an ACTIVE capture
    // COALESCES into it (one bag, causes as a list), which is the designed
    // behaviour rather than a second capture. The id is REUSED so every verdict
    // still belongs to this ask.
    let request_id = requester
        .request(&request)
        .map_err(|e| CliError::Validation(format!("could not publish the request: {e}")))?;
    let mut accepted: Vec<FlashbackOutcomeFrame> = Vec::new();
    let deadline = Instant::now() + FLASHBACK_VERDICT_WINDOW;
    let mut next_retry = Instant::now() + FLASHBACK_REQUEST_RETRY_INTERVAL;
    loop {
        accepted.extend(requester.drain_outcomes(request_id));
        // THE WHOLE WINDOW, never "until the first answer".
        // The request is BROADCAST: two recorders on one
        // desk answer independently and a suppression can easily arrive before
        // another recorder's acceptance — measured at 175 ms apart on a
        // two-recorder desk — so breaking on the first verdict would report
        // `Refused`, exit nonzero, and abandon a capture that was really
        // running. Retrying stops as soon as SOMETHING answered (the request
        // demonstrably landed); collecting does not.
        let answered = !accepted.is_empty();
        if stop() || Instant::now() >= deadline {
            break;
        }
        if !answered && Instant::now() >= next_retry {
            // A failed RE-publish is not fatal: the first one may well have been
            // delivered, and reporting an absence here would claim more than the
            // wait has established.
            let _ = requester.request_with_id(&request, request_id);
            next_retry = Instant::now() + FLASHBACK_REQUEST_RETRY_INTERVAL;
        }
        std::thread::sleep(FLASHBACK_POLL_INTERVAL);
    }
    accepted = collapse_verdicts(accepted);
    for frame in &accepted {
        writeln!(out, "{}", render_outcome(frame)).ok();
    }

    let interrupted = stop();
    let verdict = classify(&accepted, interrupted);
    if verdict == FlashbackVerdict::Interrupted {
        writeln!(
            out,
            "interrupted while waiting for a verdict — nothing was concluded about the capture"
        )
        .ok();
        return Ok(());
    }
    if verdict == FlashbackVerdict::NoRecorder {
        return Err(CliError::Validation(no_recorder_message()));
    }
    if verdict == FlashbackVerdict::Refused {
        return Err(CliError::Validation(
            "every recorder that answered refused this request — nothing was captured".to_string(),
        ));
    }
    // The first window's terminal verdicts, tallied BEFORE any exit route:
    // `--no-wait` returns here, so an `Abandoned` or a
    // `Failed` that arrived in that window would be PRINTED and then exit 0 —
    // the verb-that-lies class again, one route down.
    let mut tally = OutcomeTally::default();
    let mut settled: BTreeSet<(String, u64)> = BTreeSet::new();
    let mut seen: BTreeSet<(String, u64, bool)> = BTreeSet::new();
    for frame in &accepted {
        if !is_terminal(frame) {
            continue;
        }
        let Some(id) = capture_identity(frame) else {
            continue;
        };
        seen.insert((id.0.clone(), id.1, true));
        settled.insert(id);
        record_terminal(&mut tally, frame);
    }
    if opts.no_wait {
        return match terminal_report(&tally) {
            Some(message) => Err(CliError::Validation(message)),
            None => Ok(()),
        };
    }

    // WAIT for the bag. The deadline is the longest ETA any recorder quoted plus
    // the finalize grace — derived from what the robot SAID rather than from a
    // constant, so a capture extended by a concurrent burst is still waited out.
    let longest_ms = accepted
        .iter()
        .filter_map(|f| match &f.outcome {
            FlashbackOutcome::Accepted { ends_in_ms, .. }
            | FlashbackOutcome::Extended { ends_in_ms, .. } => Some(*ends_in_ms),
            _ => None,
        })
        .max()
        .unwrap_or(0);
    let deadline = Instant::now() + Duration::from_millis(longest_ms) + FLASHBACK_FINALIZE_GRACE;
    // The set of CAPTURES this command is waiting for, keyed by
    // `(recorder, seq)`. A COUNT of frames is the wrong size: both
    // miscounts are that one mistake from opposite directions — see
    // [`capture_identity`].
    // Every capture named in the first window — by an announcement OR by a
    // terminal verdict, because a terminal proves the capture existed just as
    // well as an acceptance does.
    let mut awaiting: BTreeSet<(String, u64)> =
        accepted.iter().filter_map(capture_identity).collect();

    loop {
        for frame in requester.drain_outcomes(request_id) {
            let id = capture_identity(&frame);
            match &frame.outcome {
                FlashbackOutcome::Finished { .. }
                | FlashbackOutcome::Failed { .. }
                | FlashbackOutcome::Abandoned { .. }
                | FlashbackOutcome::Refused { .. } => {
                    // ONE terminal verdict per capture, even if the recorder
                    // published to several requester ids that coalesced.
                    let key = id.clone().map(|(r, q)| (r, q, true));
                    if let Some(key) = key {
                        if !seen.insert(key) {
                            continue;
                        }
                    }
                    writeln!(out, "{}", render_outcome(&frame)).ok();
                    if let Some(id) = id {
                        settled.insert(id);
                    }
                    record_terminal(&mut tally, &frame);
                }
                // An acceptance arriving late — a recorder that answered after
                // the first window closed — RAISES THE BAR. Adding it to a SET
                // rather than a counter is what makes a retry's `Accepted` then
                // `Extended` (same capture) one thing to wait for rather than
                // two: bagd dedupes the request id in `attach_requester`, so it
                // sends exactly ONE terminal verdict for it.
                FlashbackOutcome::Accepted { .. } | FlashbackOutcome::Extended { .. } => {
                    if let Some(id) = id {
                        if awaiting.insert(id) {
                            writeln!(out, "{}", render_outcome(&frame)).ok();
                        }
                    }
                }
                FlashbackOutcome::Suppressed(_) => {
                    writeln!(out, "{}", render_outcome(&frame)).ok();
                }
            }
        }
        if awaiting.is_subset(&settled) || stop() || Instant::now() >= deadline {
            break;
        }
        std::thread::sleep(FLASHBACK_POLL_INTERVAL);
    }
    let unsettled = awaiting.difference(&settled).count();
    if stop() {
        writeln!(
            out,
            "interrupted while waiting for the bag — the capture is still being written on the \
             robot; look in the flashbacks directory"
        )
        .ok();
        return Ok(());
    }
    if unsettled > 0 {
        writeln!(
            out,
            "the capture was accepted but no bag was reported within the wait — it may still be \
             writing; look in the flashbacks directory (or re-run with --no-wait to skip this \
             wait)"
        )
        .ok();
    }
    // ONE decision, the same one every other exit route takes.
    match terminal_report(&tally) {
        Some(message) => Err(CliError::Validation(message)),
        None => Ok(()),
    }
}

/// Fold one terminal verdict into the tally.
///
/// The three buckets are kept APART because collapsing any two asserts an
/// outcome nobody observed: a bag that was written, one whose write returned an
/// error, and one the recorder stopped waiting for are three different facts,
/// and only the middle one is a failure.
fn record_terminal(tally: &mut OutcomeTally, frame: &FlashbackOutcomeFrame) {
    match &frame.outcome {
        FlashbackOutcome::Finished { .. } => tally.produced += 1,
        FlashbackOutcome::Failed { reason, .. } => {
            tally.failures.push(format!("{}: {reason}", frame.recorder));
        }
        FlashbackOutcome::Abandoned { reason, .. } => {
            tally
                .unconfirmed
                .push(format!("{}: {reason}", frame.recorder));
        }
        FlashbackOutcome::Refused { reason, .. } => {
            tally.refused.push(format!("{}: {reason}", frame.recorder));
        }
        // Not terminal — the caller filters, and folding one here would count a
        // capture that has not ended.
        FlashbackOutcome::Accepted { .. }
        | FlashbackOutcome::Extended { .. }
        | FlashbackOutcome::Suppressed(_) => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(recorder: &str, outcome: FlashbackOutcome) -> FlashbackOutcomeFrame {
        FlashbackOutcomeFrame {
            request_id: 1,
            recorder: recorder.to_string(),
            outcome,
        }
    }

    #[test]
    fn nobody_answering_is_not_the_same_verdict_as_a_refusal() {
        assert_eq!(classify(&[], false), FlashbackVerdict::NoRecorder);
        assert_eq!(
            classify(
                &[frame(
                    "demo",
                    FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                        captures_in_window: 20,
                        cap: 20,
                        reserved_for_manual: 0
                    })
                )],
                false
            ),
            FlashbackVerdict::Refused
        );
        // Both FAIL, and that is the shared half — but they are different
        // problems with different remedies, so they must not be one verdict.
        assert!(FlashbackVerdict::NoRecorder.is_failure());
        assert!(FlashbackVerdict::Refused.is_failure());
    }

    #[test]
    fn one_recorder_capturing_is_enough_even_when_another_refuses() {
        // The two-graph desk. A verdict that required EVERY recorder to accept
        // would fail a command that really did capture the moment.
        let outcomes = [
            frame(
                "alpha",
                FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed: 3 }),
            ),
            frame(
                "beta",
                FlashbackOutcome::Accepted {
                    seq: 0,
                    ends_in_ms: 15_000,
                    path: "/w/recordings/flashbacks/b.mcap".into(),
                },
            ),
        ];
        assert_eq!(classify(&outcomes, false), FlashbackVerdict::Captured);
        assert!(!FlashbackVerdict::Captured.is_failure());
    }

    /// An interruption is never a verdict. It wins over an EMPTY answer, which is
    /// the case that would otherwise be reported as "no serving graph answered"
    /// when the truth is that the operator did not wait for one.
    #[test]
    fn an_interrupted_wait_concludes_nothing_and_does_not_fail() {
        assert_eq!(classify(&[], true), FlashbackVerdict::Interrupted);
        assert_eq!(
            classify(
                &[frame(
                    "demo",
                    FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed: 1 })
                )],
                true
            ),
            FlashbackVerdict::Interrupted
        );
        assert!(!FlashbackVerdict::Interrupted.is_failure());
    }

    /// The rate-cap refusal reaches the OPERATOR with BOTH
    /// numbers and the knob. The two are deliberately DIFFERENT here so a swap
    /// cannot pass.
    #[test]
    fn the_rate_cap_refusal_quotes_both_numbers_and_names_the_knob() {
        let text = render_outcome(&frame(
            "go2",
            FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 17,
                cap: 12,
                reserved_for_manual: 0,
            }),
        ));
        assert!(text.contains("17"), "{text}");
        assert!(text.contains("12"), "{text}");
        assert!(text.contains("Nothing was captured"), "{text}");
        assert!(text.contains("CERULION_FLASHBACK_MAX_PER_HOUR"), "{text}");
        assert!(
            text.contains("[go2]"),
            "the answering recorder must be named: {text}"
        );
    }

    #[test]
    fn every_verdict_renders_its_recorder_and_its_own_facts() {
        let accepted = render_outcome(&frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 4,
                ends_in_ms: 15_000,
                path: "/w/recordings/flashbacks/x.mcap".into(),
            },
        ));
        assert!(
            accepted.contains("#4") && accepted.contains("15.0s"),
            "{accepted}"
        );
        assert!(accepted.contains("/w/recordings/flashbacks/x.mcap"));

        let finished = render_outcome(&frame(
            "demo",
            FlashbackOutcome::Finished {
                seq: 4,
                bytes: 155 * 1024 * 1024,
                path: "/w/recordings/flashbacks/x.mcap".into(),
                resimmable: None,
                // The older shape: no span claim (this arm is not about one).
                span: None,
            },
        ));
        assert!(finished.contains("155.0 MB"), "{finished}");

        let failed = render_outcome(&frame(
            "demo",
            FlashbackOutcome::Failed {
                seq: Some(0),
                reason: "no space left on device".into(),
            },
        ));
        assert!(
            failed.contains("FAILED") && failed.contains("no space left"),
            "{failed}"
        );

        let refractory = render_outcome(&frame(
            "demo",
            FlashbackOutcome::Suppressed(SuppressReason::Refractory {
                retry_in_ns: 42_000_000_000,
            }),
        ));
        assert!(refractory.contains("42s"), "{refractory}");
    }

    /// The absence message must name BOTH things that could be true. An operator
    /// whose graph is running and whose plane is switched off would otherwise be
    /// told only that nothing answered.
    /// An accepted capture that FAILED is a failure; one that produced a bag
    /// beside it is not. Both halves, because the rule is the pairing.
    /// Duplicates must collapse whatever ORDER they arrive in. An
    /// adjacency-only `dedup_by` passes the A,A,B vector and fails
    /// the interleaved A,B,A one — which is exactly what a two-recorder desk
    /// produces when a retry races.
    #[test]
    fn interleaved_duplicate_acceptances_collapse_to_one_capture_each() {
        let a1 = frame(
            "alpha",
            FlashbackOutcome::Accepted {
                seq: 0,
                ends_in_ms: 15_000,
                path: "/w/a.mcap".into(),
            },
        );
        let b1 = frame(
            "beta",
            FlashbackOutcome::Accepted {
                seq: 0,
                ends_in_ms: 15_000,
                path: "/w/b.mcap".into(),
            },
        );
        // The SAME capture on alpha, arriving again as an `Extended` because the
        // retry coalesced into it — a DIFFERENT frame, the same capture.
        let a2 = frame(
            "alpha",
            FlashbackOutcome::Extended {
                seq: 0,
                ends_in_ms: 14_800,
                path: "/w/a.mcap".into(),
            },
        );
        let collapsed = collapse_verdicts(vec![a1.clone(), b1.clone(), a2]);
        assert_eq!(
            collapsed.len(),
            2,
            "two recorders, one capture each — got {collapsed:?}"
        );
        assert_eq!(collapsed[0], a1);
        assert_eq!(collapsed[1], b1);

        // A SECOND capture on one recorder is genuinely distinct and survives.
        let a_second = frame(
            "alpha",
            FlashbackOutcome::Accepted {
                seq: 1,
                ends_in_ms: 15_000,
                path: "/w/a2.mcap".into(),
            },
        );
        assert_eq!(collapse_verdicts(vec![a1, a_second]).len(), 2);
    }

    /// A suppression is about no capture, so it must not be collapsed with one —
    /// and two recorders refusing for different reasons are two things to read.
    #[test]
    fn suppressions_carry_no_capture_identity_and_are_never_collapsed() {
        let x = frame(
            "alpha",
            FlashbackOutcome::Suppressed(SuppressReason::RegimeOpen { suppressed: 1 }),
        );
        let y = frame(
            "alpha",
            FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 20,
                cap: 20,
                reserved_for_manual: 0,
            }),
        );
        assert!(capture_identity(&x).is_none());
        assert_eq!(collapse_verdicts(vec![x.clone(), y.clone(), x]).len(), 3);
        let _ = y;
    }

    /// A retry's `Accepted` + `Extended` are ONE capture to wait
    /// for. bagd sends exactly one terminal verdict for them, so sizing the wait
    /// by frames would make the command time out over a bag that had been written.
    #[test]
    fn a_retry_that_coalesced_is_one_capture_to_wait_for_not_two() {
        let accepted = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 7,
                ends_in_ms: 15_000,
                path: "/w/x.mcap".into(),
            },
        );
        let extended = frame(
            "demo",
            FlashbackOutcome::Extended {
                seq: 7,
                ends_in_ms: 14_700,
                path: "/w/x.mcap".into(),
            },
        );
        let finished = frame(
            "demo",
            FlashbackOutcome::Finished {
                seq: 7,
                bytes: 1,
                path: "/w/x.mcap".into(),
                resimmable: None,
                // The older shape: no span claim (this arm is not about one).
                span: None,
            },
        );
        let awaiting: BTreeSet<_> = [&accepted, &extended]
            .into_iter()
            .filter_map(capture_identity)
            .collect();
        assert_eq!(
            awaiting.len(),
            1,
            "one capture, however many frames announced it"
        );
        let settled: BTreeSet<_> = [&finished]
            .into_iter()
            .filter_map(capture_identity)
            .collect();
        assert!(
            awaiting.is_subset(&settled),
            "…and that ONE terminal verdict settles it"
        );
    }

    /// A post-accept FAILURE must SETTLE the acceptance it
    /// belongs to.
    ///
    /// Keyed to `u64::MAX` it would never match the
    /// `(recorder, seq)` the `Accepted` was tracked under, so the command would wait
    /// out its whole deadline and print "no bag was reported" over a capture it
    /// had already been told failed. The sequence rides the verdict instead.
    #[test]
    fn a_post_accept_failure_settles_the_capture_it_belongs_to() {
        let accepted = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 7,
                ends_in_ms: 15_000,
                path: "/w/x.mcap".into(),
            },
        );
        let failed = frame(
            "demo",
            FlashbackOutcome::Failed {
                seq: Some(7),
                reason: "no space left on device".into(),
            },
        );
        assert_eq!(
            capture_identity(&failed),
            Some(("demo".to_string(), 7)),
            "the failure must name the capture it is about"
        );
        let awaiting: BTreeSet<_> = [&accepted]
            .into_iter()
            .filter_map(capture_identity)
            .collect();
        let settled: BTreeSet<_> = [&failed].into_iter().filter_map(capture_identity).collect();
        assert!(
            awaiting.is_subset(&settled),
            "a failure settles its acceptance — otherwise the wait can never end"
        );

        // …and a failure from ANOTHER recorder settles nothing here.
        let other = frame(
            "beta",
            FlashbackOutcome::Failed {
                seq: Some(7),
                reason: "no space left on device".into(),
            },
        );
        let elsewhere: BTreeSet<_> = [&other].into_iter().filter_map(capture_identity).collect();
        assert!(!awaiting.is_subset(&elsewhere));
    }

    /// A failure that never had an acceptance carries NO identity — it must not
    /// invent an outstanding capture, and it must not settle somebody else's.
    #[test]
    fn a_failure_with_no_capture_behind_it_settles_nothing() {
        let refused = frame(
            "demo",
            FlashbackOutcome::Failed {
                seq: None,
                reason: "a previous flashback is still being written".into(),
            },
        );
        assert_eq!(capture_identity(&refused), None);
        // It is still a REFUSAL for classification, so the verb never claims a
        // capture happened.
        assert_eq!(classify(&[refused], false), FlashbackVerdict::Refused);
    }

    /// An announcement must never COLLAPSE its own terminal
    /// verdict.
    ///
    /// A `Failed` carries its capture's sequence, so it shares an
    /// identity with the `Accepted` it settles — and collapsing on identity
    /// alone dropped whichever arrived second, which in the first window is the
    /// terminal one. A capture that failed fast therefore lost its verdict
    /// entirely: the command waited out its deadline, printed that no bag was
    /// reported, and could exit 0 with nothing on disk.
    #[test]
    fn a_terminal_verdict_is_never_collapsed_by_its_own_announcement() {
        let accepted = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 3,
                ends_in_ms: 15_000,
                path: "/w/x.mcap".into(),
            },
        );
        let failed = frame(
            "demo",
            FlashbackOutcome::Failed {
                seq: Some(3),
                reason: "no space left on device".into(),
            },
        );
        let finished = frame(
            "demo",
            FlashbackOutcome::Finished {
                seq: 4,
                bytes: 9,
                path: "/w/y.mcap".into(),
                resimmable: None,
                // The older shape: no span claim (this arm is not about one).
                span: None,
            },
        );
        let accepted_4 = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 4,
                ends_in_ms: 15_000,
                path: "/w/y.mcap".into(),
            },
        );

        // Both lifecycles survive, in arrival order.
        let out = collapse_verdicts(vec![
            accepted.clone(),
            failed.clone(),
            accepted_4.clone(),
            finished.clone(),
        ]);
        assert_eq!(
            out.len(),
            4,
            "an announcement must not eat its terminal: {out:?}"
        );

        // …while duplicates WITHIN a stage still collapse — the dedup rule is
        // intact, which is what makes this a refinement rather than a revert.
        let dup = collapse_verdicts(vec![
            accepted.clone(),
            frame(
                "demo",
                FlashbackOutcome::Extended {
                    seq: 3,
                    ends_in_ms: 14_000,
                    path: "/w/x.mcap".into(),
                },
            ),
            failed.clone(),
            failed.clone(),
        ]);
        assert_eq!(dup.len(), 2, "one announcement + one terminal: {dup:?}");
        assert!(!is_terminal(&accepted) && is_terminal(&failed));
    }

    /// A capture whose WHOLE lifecycle lands in the first window is
    /// already settled — the wait must not start over for it.
    ///
    /// Driven through `classify` + the identity sets the wait is built from,
    /// because the seeding itself lives inside `flashback_capture` (which needs
    /// a transport). Each half is what the loop reads.
    #[test]
    fn a_capture_settled_inside_the_first_window_needs_no_further_wait() {
        let accepted = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 1,
                ends_in_ms: 15_000,
                path: "/w/x.mcap".into(),
            },
        );
        let finished = frame(
            "demo",
            FlashbackOutcome::Finished {
                seq: 1,
                bytes: 5,
                path: "/w/x.mcap".into(),
                resimmable: None,
                // The older shape: no span claim (this arm is not about one).
                span: None,
            },
        );
        let window = collapse_verdicts(vec![accepted, finished]);

        let awaiting: BTreeSet<_> = window.iter().filter_map(capture_identity).collect();
        let settled: BTreeSet<_> = window
            .iter()
            .filter(|f| is_terminal(f))
            .filter_map(capture_identity)
            .collect();
        assert_eq!(awaiting.len(), 1);
        assert!(
            awaiting.is_subset(&settled),
            "the first window already answered — waiting again is what printed 'no bag was \
             reported' over a bag that existed"
        );
    }

    /// A first-window FAILURE that names its capture proves a capture happened,
    /// so it is not a REFUSAL — the exit code comes from `terminal_failure`
    /// instead, which is a different message with a different remedy.
    #[test]
    fn a_failure_naming_its_capture_is_not_classified_as_a_refusal() {
        let failed = frame(
            "demo",
            FlashbackOutcome::Failed {
                seq: Some(2),
                reason: "no space left on device".into(),
            },
        );
        assert_eq!(
            classify(std::slice::from_ref(&failed), false),
            FlashbackVerdict::Captured
        );
        // …and it still exits nonzero, via the rule that owns that decision.
        let mut tally = OutcomeTally::default();
        record_terminal(&mut tally, &failed);
        assert!(terminal_report(&tally).is_some());
    }

    /// A capture the recorder STOPPED WAITING for must not
    /// be reported as FAILED.
    ///
    /// The recorder's shutdown wait is bounded, but a writer that merely ran
    /// LATE finishes a moment after the bound and leaves a complete, finalized
    /// bag — while a `Failed` verdict had already told the requester it failed
    /// and made the verb exit nonzero. A verdict that contradicts the disk is
    /// worse than no verdict, so the outcome says what is actually known.
    #[test]
    fn an_abandoned_capture_is_reported_as_unconfirmed_never_as_failed() {
        let abandoned = frame(
            "demo",
            FlashbackOutcome::Abandoned {
                seq: 5,
                reason: "the recorder stopped while this was still being written — check /w/x.mcap"
                    .into(),
            },
        );
        let text = render_outcome(&abandoned);
        assert!(
            text.contains("UNCONFIRMED"),
            "the operator must be told this is unknown, not failed: {text}"
        );
        assert!(
            !text.contains("FAILED"),
            "…and the word FAILED must not appear: {text}"
        );
        assert!(
            text.contains("/w/x.mcap"),
            "…with somewhere to look: {text}"
        );

        // It is TERMINAL and it SETTLES its acceptance, so nothing waits out a
        // deadline for a verdict that will never come.
        assert!(is_terminal(&abandoned));
        assert_eq!(capture_identity(&abandoned), Some(("demo".to_string(), 5)));
        let accepted = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 5,
                ends_in_ms: 15_000,
                path: "/w/x.mcap".into(),
            },
        );
        let awaiting: BTreeSet<_> = [&accepted]
            .into_iter()
            .filter_map(capture_identity)
            .collect();
        let settled: BTreeSet<_> = [&abandoned]
            .into_iter()
            .filter_map(capture_identity)
            .collect();
        assert!(awaiting.is_subset(&settled));

        // A capture happened — so this is not a REFUSAL either.
        assert_eq!(
            classify(std::slice::from_ref(&abandoned), false),
            FlashbackVerdict::Captured
        );

        // …and it is not counted as a FAILURE. It gets the UNCONFIRMED message,
        // never the all-failed one — a late-but-fine writer must not be reported
        // as having failed.
        let mut tally = OutcomeTally::default();
        record_terminal(&mut tally, &abandoned);
        assert_eq!(tally.failures.len(), 0);
        assert_eq!(tally.unconfirmed.len(), 1);
        let message = terminal_report(&tally).expect("nothing was produced, so this is not a pass");
        assert!(message.contains("UNCONFIRMED"), "{message}");
        assert!(!message.contains("FAILED"), "{message}");
    }

    /// …and an announcement still cannot collapse it, exactly as for the other
    /// two terminals.
    #[test]
    fn an_abandoned_verdict_is_never_collapsed_by_its_own_announcement() {
        let accepted = frame(
            "demo",
            FlashbackOutcome::Accepted {
                seq: 5,
                ends_in_ms: 15_000,
                path: "/w/x.mcap".into(),
            },
        );
        let abandoned = frame(
            "demo",
            FlashbackOutcome::Abandoned {
                seq: 5,
                reason: "stopped waiting".into(),
            },
        );
        let out = collapse_verdicts(vec![accepted, abandoned]);
        assert_eq!(out.len(), 2, "{out:?}");
    }

    fn tally(produced: usize, failures: &[&str], unconfirmed: &[&str]) -> OutcomeTally {
        OutcomeTally {
            produced,
            failures: failures.iter().map(|s| (*s).to_string()).collect(),
            unconfirmed: unconfirmed.iter().map(|s| (*s).to_string()).collect(),
            refused: Vec::new(),
        }
    }

    /// THE MIXED-OUTCOME RULE, pinned in all four arms.
    #[test]
    fn the_mixed_outcome_rule_is_produced_wins_then_unknown_then_failed() {
        // 1. A produced bag WINS, whatever else happened — the operator has the
        //    moment and every other outcome was printed.
        assert_eq!(
            terminal_report(&tally(1, &["a: disk"], &["b: stopped"])),
            None
        );
        assert_eq!(terminal_report(&tally(2, &[], &[])), None);

        // 2. Nothing produced + anything UNCONFIRMED ⇒ the unconfirmed message,
        //    EVEN beside a failure. Were the failure arm
        //    checked first, a two-recorder run with one Failed and one
        //    Abandoned would tell the operator every capture failed and send them away
        //    from a bag that may be sitting there complete.
        let mixed = terminal_report(&tally(0, &["a: disk"], &["b: stopped"]))
            .expect("nothing produced ⇒ nonzero");
        assert!(mixed.contains("UNCONFIRMED"), "{mixed}");
        assert!(
            !mixed.contains("every accepted capture FAILED"),
            "an unconfirmed capture must not be hidden behind a false all-failed claim: {mixed}"
        );
        // …and NOTHING is hidden by the precedence: both are listed.
        assert!(
            mixed.contains("b: stopped") && mixed.contains("a: disk"),
            "{mixed}"
        );

        // 3. Nothing produced + only failures ⇒ the all-failed message.
        let all_failed =
            terminal_report(&tally(0, &["a: disk"], &[])).expect("nothing produced ⇒ nonzero");
        assert!(
            all_failed.contains("every accepted capture FAILED"),
            "{all_failed}"
        );

        // 4. Nothing terminal at all ⇒ exit 0; the wait's own timeout line covers
        //    that case, and inventing a failure here would claim one nobody saw.
        assert_eq!(terminal_report(&OutcomeTally::default()), None);
    }

    /// EVERY exit route runs the tally through `terminal_report`.
    ///
    /// A STRUCTURAL pin, because the routes live inside `flashback_capture`,
    /// which needs a transport — and the defect it guards is precisely one route
    /// (`--no-wait`) returning BEFORE the rule, printing UNCONFIRMED and exiting
    /// 0. The rule itself is oracle-pinned above; this asserts it is not skipped.
    ///
    /// Read over a comment-stripped view so the prose here — which names the
    /// route and the function — cannot satisfy it.
    #[test]
    fn every_exit_route_runs_the_terminal_report() {
        let code: String = include_str!("flashback_cmd.rs")
            .lines()
            .map(|line| match line.find("//") {
                Some(i) => &line[..i],
                None => line,
            })
            .collect::<Vec<_>>()
            .join("\n");
        // Assembled so this test's own source is not the match.
        let needle = format!("{}{}", "if opts.", "no_wait {");
        let start = code.find(&needle).expect("the --no-wait route must exist");
        let route = &code[start..(start + 400).min(code.len())];
        assert!(
            route.contains("terminal_report"),
            "the --no-wait route must apply the same exit rule as every other:\n{route}"
        );
        // …and the route does not hand-roll its own message instead.
        assert!(
            !route.contains("every accepted capture"),
            "the --no-wait route must DELEGATE, not restate the rule:\n{route}"
        );
    }

    /// The three buckets are chosen by the VARIANT, not by anything else — the
    /// anti-tautology half of the rule above, which is only meaningful if a
    /// verdict lands where it belongs.
    #[test]
    fn each_terminal_verdict_lands_in_its_own_bucket() {
        let mut t = OutcomeTally::default();
        record_terminal(
            &mut t,
            &frame(
                "a",
                FlashbackOutcome::Finished {
                    seq: 0,
                    bytes: 1,
                    path: "/w/a.mcap".into(),
                    resimmable: None,
                    // The older shape: no span claim (this arm is not about one).
                    span: None,
                },
            ),
        );
        record_terminal(
            &mut t,
            &frame(
                "b",
                FlashbackOutcome::Failed {
                    seq: Some(0),
                    reason: "disk".into(),
                },
            ),
        );
        record_terminal(
            &mut t,
            &frame(
                "c",
                FlashbackOutcome::Abandoned {
                    seq: 0,
                    reason: "stopped waiting".into(),
                },
            ),
        );
        // An ANNOUNCEMENT is not terminal and must fold to nothing — counting one
        // here would settle a capture that has not ended.
        record_terminal(
            &mut t,
            &frame(
                "d",
                FlashbackOutcome::Accepted {
                    seq: 0,
                    ends_in_ms: 1,
                    path: "/w/d.mcap".into(),
                },
            ),
        );
        assert_eq!(t, tally(1, &["b: disk"], &["c: stopped waiting"]));
    }

    #[test]
    fn the_absence_message_names_the_two_things_that_could_be_true() {
        let text = no_recorder_message();
        assert!(text.contains("cerulion graph run"), "{text}");
        assert!(text.contains("CERULION_FLASHBACK=off"), "{text}");
    }

    /// An AUTOMATIC refusal explains where the missing slots went.
    ///
    /// The effective cap alone is a half-explanation, and the half it omits is
    /// the one that decides what the operator does next: somebody who set
    /// `CERULION_FLASHBACK_MAX_PER_HOUR=20` and reads "the cap is 18" has been
    /// told their knob is being ignored, so the natural remedy is to raise it —
    /// when the reserved budget was never theirs to reclaim.
    ///
    /// The ZERO-reserve control is in the same body, because either half alone
    /// reads as the other's bug: "the reserve is named" without it is satisfied
    /// by a renderer that names one unconditionally, which would describe a
    /// mechanism that did not apply on every manual refusal.
    #[test]
    fn an_automatic_rate_cap_refusal_names_the_reserve_and_a_manual_one_does_not() {
        // AUTOMATIC: refused at the EFFECTIVE cap, with two slots withheld.
        let automatic = render_outcome(&frame(
            "go2",
            FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 18,
                cap: 18,
                reserved_for_manual: 2,
            }),
        ));
        assert!(automatic.contains("18"), "{automatic}");
        assert!(
            automatic.contains('2'),
            "the WITHHELD count must appear, or the missing slots are \
             unexplained: {automatic}"
        );
        assert!(
            automatic.contains("reserved for manual"),
            "…and it must say what they are FOR, or an operator reads it as the \
             cap being wrong: {automatic}"
        );
        // The knob is still named — the reserve narrows the budget, it does not
        // replace the remedy.
        assert!(
            automatic.contains("CERULION_FLASHBACK_MAX_PER_HOUR"),
            "{automatic}"
        );
        assert!(automatic.contains("Nothing was captured"), "{automatic}");

        // MANUAL (or any policy whose clamp collapsed the reserve): byte-for-byte
        // the sentence with no reserve clause. Naming a reserve of zero would describe a
        // mechanism that did not apply to this refusal.
        let manual = render_outcome(&frame(
            "go2",
            FlashbackOutcome::Suppressed(SuppressReason::RateCapped {
                captures_in_window: 20,
                cap: 20,
                reserved_for_manual: 0,
            }),
        ));
        assert!(
            !manual.contains("reserved for manual"),
            "a manual refusal must not mention a reserve it never paid: {manual}"
        );
        assert!(manual.contains("20"), "{manual}");
        assert!(
            manual.contains("CERULION_FLASHBACK_MAX_PER_HOUR"),
            "{manual}"
        );
    }
}

#[cfg(test)]
mod shell_quote_tests {
    use super::*;

    /// The RESIMMABLE line hands over a COPYABLE command, so the bag path in it
    /// must be one shell word.
    ///
    /// `CERULION_FLASHBACK_DIR` is taken verbatim, so `~/Robot Logs/` is an
    /// ordinary operator setup — and unquoted it produces a command that runs
    /// `bag play` against a truncated path and reports a bag that is not there.
    /// A line that is WRONG when pasted is worse than one that is merely ugly.
    #[test]
    fn a_path_that_needs_quoting_is_quoted_and_an_ordinary_one_is_not() {
        // An ordinary path stays BARE — quoting everything would make the
        // common line noisier to read for a hazard it does not have.
        assert_eq!(
            shell_quote("/home/op/.cerulion/flashbacks/cap-7.mcap"),
            "/home/op/.cerulion/flashbacks/cap-7.mcap"
        );
        // A space is the shipped hazard.
        assert_eq!(
            shell_quote("/home/op/Robot Logs/cap-7.mcap"),
            "'/home/op/Robot Logs/cap-7.mcap'"
        );
        // Anything outside the conservative safe set is quoted, so a character
        // nobody thought about is quoted rather than passed through.
        for hostile in ["a;rm -rf b", "a$(x)b", "a`x`b", "a&b", "a|b", "a*b", "a\nb"] {
            let quoted = shell_quote(hostile);
            assert!(
                quoted.starts_with('\'') && quoted.ends_with('\''),
                "{hostile:?} must be quoted, got {quoted}"
            );
        }
        // POSIX has no escape for `'` inside single quotes: close, escape,
        // reopen. This is the one shape a naive wrapper gets wrong.
        assert_eq!(shell_quote("a'b"), r#"'a'\''b'"#);
        // An EMPTY path must still be one word, or it vanishes from the command.
        assert_eq!(shell_quote(""), "''");
    }

    /// The SHORTFALL rides the line an operator reads during an incident, and
    /// only when there is one.
    ///
    /// Three readings that must stay apart, driven in one body:
    ///
    /// - a shortfall is NAMED, with both spans, beside whatever verdict the
    ///   capture earned — a bag can be perfectly resimmable and still cover two
    ///   seconds of a window it claims thirty for, and reporting only the first
    ///   is how an operator resumes a recording that does not hold the lead-up;
    /// - a capture that lost NOTHING says nothing about it, so an ordinary
    ///   capture's line is unchanged; and
    /// - an UNKNOWN span (an earlier recorder) also says nothing, because a
    ///   hedge there would put a permanent question mark on every capture that is
    ///   fine.
    #[test]
    fn the_finished_line_names_a_shortfall_and_stays_quiet_when_there_is_none() {
        let short = render_outcome(&finished_with(Some(span(45_000, 1_990, 63_451))));
        assert!(
            short.contains("COVERS 2.0s of the 45.0s it claims"),
            "{short}"
        );
        assert!(short.contains("43.0s short"), "{short}");
        // …and it does NOT displace the verdict: both facts are about this bag.
        assert!(short.contains("RESIMMABLE"), "{short}");

        let whole = render_outcome(&finished_with(Some(span(45_000, 45_000, 0))));
        assert!(
            !whole.contains("COVERS"),
            "a capture that lost nothing must not explain itself: {whole}"
        );

        let unknown = render_outcome(&finished_with(None));
        assert!(
            !unknown.contains("COVERS"),
            "an older recorder made no claim, and a hedge is not a claim either: {unknown}"
        );
    }

    /// The CAUSE is claimed only on evidence.
    ///
    /// A shortfall does not imply eviction — a capture triggered inside its
    /// first span, a robot with sparse topics, or an interval nobody published
    /// in all produce one with NOTHING evicted — so blaming the byte ceiling
    /// unconditionally sends an operator to raise a cap that never bound, which
    /// is worse than saying nothing: it is a wrong instruction on the one line
    /// they read during an incident.
    ///
    /// Both arms in one body, over the SAME spans, differing only in the
    /// evidence — so neither can be satisfied by a renderer that ignores it.
    #[test]
    fn the_finished_line_blames_the_byte_ceiling_only_when_frames_were_evicted() {
        // ZERO truncation, real shortfall: the measurement, no cause, no cap.
        let no_evidence = render_outcome(&finished_with(Some(span(45_000, 15_000, 0))));
        assert!(
            no_evidence.contains("COVERS 15.0s of the 45.0s it claims (30.0s short)"),
            "the measurement is still reported: {no_evidence}"
        );
        assert!(
            !no_evidence.contains("evict"),
            "nothing was evicted, so nothing may be blamed on eviction: {no_evidence}"
        );
        assert!(
            !no_evidence.contains("CERULION_FLASHBACK_WINDOW_MAX_MB"),
            "…and the operator must not be sent to raise a cap that never bound: {no_evidence}"
        );

        // The SAME spans with evidence: the causal sentence and the remedy.
        let evidenced = render_outcome(&finished_with(Some(span(45_000, 15_000, 63_451))));
        assert!(evidenced.contains("(30.0s short)"), "{evidenced}");
        assert!(
            evidenced.contains("byte ceiling evicted 63451 frame(s)"),
            "{evidenced}"
        );
        assert!(
            evidenced.contains("CERULION_FLASHBACK_WINDOW_MAX_MB"),
            "{evidenced}"
        );
    }

    fn span(
        claimed_span_ms: u64,
        achieved_span_ms: u64,
        truncated_frames: u64,
    ) -> cerulion_core::flashback::channel::FinishedSpan {
        cerulion_core::flashback::channel::FinishedSpan {
            claimed_span_ms,
            achieved_span_ms,
            truncated_frames,
        }
    }

    fn finished_with(
        span: Option<cerulion_core::flashback::channel::FinishedSpan>,
    ) -> FlashbackOutcomeFrame {
        FlashbackOutcomeFrame {
            request_id: 1,
            recorder: "demo".to_string(),
            outcome: FlashbackOutcome::Finished {
                seq: 7,
                bytes: 1024,
                path: "/w/f/cap-7.mcap".to_string(),
                resimmable: Some(true),
                span,
            },
        }
    }

    /// …and the rendered line really carries the quoted form.
    #[test]
    fn the_resimmable_line_quotes_the_path_it_tells_you_to_run() {
        let frame = FlashbackOutcomeFrame {
            request_id: 1,
            recorder: "demo".to_string(),
            outcome: FlashbackOutcome::Finished {
                seq: 7,
                bytes: 1024,
                path: "/home/op/Robot Logs/cap-7.mcap".to_string(),
                resimmable: Some(true),
                // The older shape: no span claim (this arm is not about one).
                span: None,
            },
        };
        let line = render_outcome(&frame);
        assert!(
            line.contains("`cerulion bag play --resim all -- '/home/op/Robot Logs/cap-7.mcap'`"),
            "the copyable command must survive a space in the capture directory: {line}"
        );
    }

    /// …and a path that STARTS WITH `-` must not be read by clap as a flag.
    ///
    /// Quoting alone does not fix this: it makes the SHELL pass `-logs/….mcap`
    /// as one word, and clap then rejects that word as an unknown option. The
    /// path therefore goes after `--`, which is also why the flags moved ahead
    /// of it. `CERULION_FLASHBACK_DIR=-logs` is all it takes, and the hinted
    /// command failed on exactly the recorder whose output most needs replaying.
    #[test]
    fn the_resimmable_line_survives_a_path_that_starts_with_a_dash() {
        let frame = FlashbackOutcomeFrame {
            request_id: 1,
            recorder: "demo".to_string(),
            outcome: FlashbackOutcome::Finished {
                seq: 7,
                bytes: 1024,
                path: "-logs/cap-7.mcap".to_string(),
                resimmable: Some(true),
                // The older shape: no span claim (this arm is not about one).
                span: None,
            },
        };
        let line = render_outcome(&frame);
        // NOT quoted, and correctly so: `shell_quote` quotes only what the
        // SHELL would mangle, and this path has nothing shell-special in it.
        // The hazard here is CLAP, not the shell, and `--` is what answers it.
        assert!(
            line.contains("--resim all -- -logs/cap-7.mcap"),
            "a leading-dash path must land AFTER clap's `--`: {line}"
        );
        // The ORDER is the fix, so it is asserted as an order: nothing may sit
        // between `play` and the flags that could be eaten as an option value.
        assert!(
            !line.contains("bag play '-logs"),
            "the path must never precede the flags: {line}"
        );
    }

    /// A REFUSAL is its own answer, everywhere a verdict is read.
    ///
    /// Three states an operator acts on differently — a bag was written, the
    /// recorder broke, the machine is too small — and this verb decides all
    /// three in four separate places. Every one is asserted here, because
    /// collapsing any of them sends an operator to the wrong remedy: a refusal
    /// reported as a FAILURE reads as a broken robot (hunt a fault), and one
    /// reported as CAPTURED claims a bag that does not exist.
    #[test]
    fn a_refusal_is_terminal_named_and_never_counted_as_a_capture() {
        let refused = FlashbackOutcomeFrame {
            request_id: 1,
            recorder: "demo".to_string(),
            outcome: FlashbackOutcome::Refused {
                seq: 7,
                reason: "the plane cannot hold one generation. Set FOO=123".into(),
            },
        };

        // (1) It is the LAST word, so the wait settles instead of timing out.
        assert!(is_terminal(&refused));
        // (2) It NAMES its capture, so it settles the acceptance it belongs to.
        assert_eq!(capture_identity(&refused), Some(("demo".to_string(), 7)));
        // (3) It renders under its OWN word, carrying the remedy.
        let line = render_outcome(&refused);
        assert!(line.contains("REFUSED"), "{line}");
        assert!(
            line.contains("Set FOO=123"),
            "the remedy is the whole content of this verdict: {line}"
        );
        assert!(
            !line.contains("FAILED"),
            "a refusal must not read as a failure — different remedy: {line}"
        );

        // (4) It is NOT a capture. `classify` decides the verb's headline, and
        // counting a refusal as `Captured` would claim a bag that was
        // deliberately never written.
        assert_eq!(
            classify(std::slice::from_ref(&refused), false),
            FlashbackVerdict::Refused
        );

        // …and it lands in its OWN tally bucket, which decides the exit message.
        let mut tally = OutcomeTally::default();
        record_terminal(&mut tally, &refused);
        assert_eq!(tally.produced, 0, "no bag was produced");
        assert!(tally.failures.is_empty(), "nothing FAILED");
        assert!(tally.unconfirmed.is_empty(), "nothing is UNKNOWN");
        assert_eq!(tally.refused.len(), 1);

        let report = terminal_report(&tally).expect("a refusal is a nonzero exit");
        assert!(report.contains("REFUSED"), "{report}");
        assert!(report.contains("Set FOO=123"), "{report}");

        // PRECEDENCE, both directions. An UNCONFIRMED capture still outranks it
        // (a bag may be on disk and the operator should look), while a refusal
        // outranks a plain failure (it carries an exact remedy).
        let mut mixed = OutcomeTally::default();
        record_terminal(&mut mixed, &refused);
        mixed.unconfirmed.push("other: stopped waiting".into());
        let report = terminal_report(&mixed).expect("still nonzero");
        assert!(
            report.contains("UNCONFIRMED"),
            "'unknown' outranks a definite negative: {report}"
        );

        let mut with_failure = OutcomeTally::default();
        record_terminal(&mut with_failure, &refused);
        with_failure.failures.push("other: no space left".into());
        let report = terminal_report(&with_failure).expect("still nonzero");
        assert!(report.contains("REFUSED"), "{report}");
        assert!(
            report.contains("no space left"),
            "…and the failure beside it is still printed, so the precedence hides \
             nothing: {report}"
        );

        // ANTI-TAUTOLOGY: a PRODUCED bag still wins over everything, so the arms
        // above are the refusal's doing and not "any tally is nonzero".
        let mut produced = OutcomeTally::default();
        record_terminal(&mut produced, &refused);
        produced.produced = 1;
        assert_eq!(terminal_report(&produced), None);
    }
}
